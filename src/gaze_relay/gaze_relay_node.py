#!/usr/bin/env python

import tf
import math
import rospy
import actionlib

from threading import Lock
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from hace_msgs.msg import MinimalHumans, MinimalHuman
from hlrc_server.msg import lookattargetAction, lookattargetGoal
from openface2_ros_msgs.msg import Faces
from visualization_msgs.msg import Marker
from gaze_relay.msg import GazeRelayTarget

# init ROS
rospy.init_node('gaze_relay')

lookat_client = actionlib.SimpleActionClient('/floka/set/lookat', lookattargetAction)
rospy.loginfo('waiting for floka action server')
lookat_client.wait_for_server()
rospy.loginfo('Action server found!')

# hyperparameters
max_target_time = 0.5 # in sec
control_rate = 10

ppl = None
ppl_frame = None
ppl_mtx = Lock()

faces = None
faces_mtx = Lock()

iotp = None
iotp_time = None
iotp_frame = None
iotp_mtx = Lock()

hand_tf = {'left': None, 'right': None}
hand_tf_mtx = Lock()

target = None
target_time = None
target_mtx = Lock()

min_dur = None

# returns True if there is a person in people with ID pid
def _person_id_in_list(pid):

    if ppl is None:
        return False
    else:
        return any(filter(lambda x: x.uuid == pid, ppl.humans))


# returns a person given a pid
def _get_person_by_id(pid):
    for p in ppl:
        if p.id == pid:
            return p
    rospy.logerr('person with id {} not found. returning None.'.format(pid))
    return None


# callback for new list of people
def _on_new_people(new_ppl):

    global ppl
    global ppl_frame

    # we savely store the new people
    ppl_mtx.acquire(True)
    ppl = new_ppl.pose.position
    ppl_frame = new_ppl.header.frame_id
    # rospy.loginfo('got {} new people'.format(len(ppl.humans)))
    # rospy.loginfo('got new people')
    ppl_mtx.release()


# callback for new list of people
def _on_new_faces(new_faces):

    global faces

    # we savely store the new faces
    faces_mtx.acquire(True)
    if new_faces.count > 0:
        faces = new_faces
        # rospy.loginfo('got {} new faces'.format(faces.count))
    faces_mtx.release()


# callback for integrated object transfer point
def _on_new_iotp(new_iotp):

    global iotp
    global iotp_time
    global iotp_frame

    # we savely store the new iotp
    iotp_mtx.acquire(True)
    iotp = new_iotp.pose.position
    iotp_time = rospy.Time.now()
    iotp_frame = new_iotp.header.frame_id
    iotp_mtx.release()
    rospy.logdebug('got new iotp')


# callback for new gaze target
def _on_new_gaze_target(tar):

    global target
    global target_time
    global min_dur

    # either first target or new target after duration expired
    if target_time is None or (rospy.Time.now() - target_time) > min_dur:
        target_mtx.acquire()

        target = tar
        min_dur = rospy.Duration(tar.min_duration)

        target_time = rospy.Time.now()
        target_mtx.release()

        print('### ACCEPTING NEW TARGET %d for %.2f secs' % (tar.gaze_target, min_dur.secs+min_dur.nsecs*1e-9))

    # set new target regardless of whether the current one was active for its minimum durations
    elif tar.force:

        target_mtx.acquire()

        target = tar
        min_dur = rospy.Duration(tar.min_duration)

        target_time = rospy.Time.now()
        target_mtx.release()

        print('### FORCEFULLY ACCEPTING NEW TARGET %d for %.2f secs' % (tar.gaze_target, min_dur.secs+min_dur.nsecs*1e-9))
    # else reject target
    else:
        left = (min_dur - (rospy.Time.now() - target_time))
        print('### REJECTING TARGET %d; %.2f sec left' % (tar.gaze_target, left.secs+left.nsecs*1e-9))


def check_timeout():
    global target

    global target_time

    if target_time is not None and rospy.Time.now() - target_time > rospy.Duration(max_target_time):
        print('resetting target')
        target_time = None
        target = None


# subscribers
people_sub = rospy.Subscriber('/otpprediction/handtracking/hand_marker', Marker, _on_new_people)
gtarget_sub = rospy.Subscriber('/gaze_relay/target', GazeRelayTarget, _on_new_gaze_target)
iotp_sub = rospy.Subscriber('/otpprediction/prediction/iOTP', Marker, _on_new_iotp)
faces_sub = rospy.Subscriber('/openface2/faces', Faces, _on_new_faces)

tf_listener = tf.TransformListener(True, rospy.Duration(10))

rospy.loginfo('starting control loop!')

while not rospy.is_shutdown():
    rospy.Rate(control_rate).sleep()

    # now we check the target string
    target_mtx.acquire()

    if target is not None:
        print('got target ' + str(target.gaze_target) + ' with age ' + str(rospy.Time.now() - target_time))

        # check whether we even have pplz
        ppl_mtx.acquire()
        # if (ppl is None or len(ppl.humans) == 0):
        if ppl is None:
            if target.gaze_target == GazeRelayTarget.TORSO or target.gaze_target == GazeRelayTarget.RIGHT_HAND or target.gaze_target == GazeRelayTarget.LEFT_HAND:
                rospy.loginfo('We don\'t have any ppl stored.')
                ppl_mtx.release()
                check_timeout()
                target_mtx.release()
                continue
        ppl_mtx.release()

        # check whether we even have faces
        faces_mtx.acquire()
        if (faces is None or faces.count == 0):
            if target.gaze_target == GazeRelayTarget.FACE:
                rospy.loginfo('We don\'t have any faces stored.')
                faces_mtx.release()
                check_timeout()
                target_mtx.release()
                continue
        faces_mtx.release()

        if target.gaze_target == GazeRelayTarget.NEUTRAL:
            target_point = Point(x=2.0, y=0, z=0.30)
            person = MinimalHuman()
            person.header.frame_id = 'floka_BASE_LINK'
            person.header.stamp = rospy.get_time()
            rospy.loginfo('looking neutral')

        elif target.gaze_target == GazeRelayTarget.FACE:
            faces_mtx.acquire()
            if (faces is None or faces.count == 0):
                rospy.loginfo('We don\'t have any faces stored.')
                faces_mtx.release()
                check_timeout()
                target_mtx.release()
                continue
            else:
                target_point = faces.faces[0].head_pose.position
            faces_mtx.release()
        elif target.gaze_target == GazeRelayTarget.ROBOT_LEFT_HAND or target.gaze_target == GazeRelayTarget.ROBOT_RIGHT_HAND\
                or target.gaze_target == GazeRelayTarget.IOTP or target.gaze_target == GazeRelayTarget.RIGHT_HAND:
            target_point = Point(x=0, y=0, z=0)
        else:
            # if we dont find the person just use id = 0
            if _person_id_in_list(target.person_id):
                person = _get_person_by_id(target.person_id)
                rospy.loginfo('found person with id {}.'.format(target.person_id))
            else:
                person = ppl.humans[0]
                rospy.loginfo('no person with id {}! Using first person in list.'.format(target.person_id))

            if target.gaze_target == GazeRelayTarget.LEFT_HAND:
                target_point = person.left_hand.position
            elif target.gaze_target == GazeRelayTarget.RIGHT_HAND:
                target_point = person.right_hand.position
            elif target.gaze_target == GazeRelayTarget.TORSO:
                target_point = person.torso.position
            else:
                rospy.logerr('unknown gaze target {}. Use constants defined in GazeRelayTarget msg.')

        # check goal for nan values
        for v in (target_point.x, target_point.y, target_point.z):
            if math.isnan(v):
                rospy.logerr('target was nan')
                check_timeout()
                target_mtx.release()
                continue

        # if we got here, we have a goal to look at
        # construct goal
        lookat_goal = lookattargetGoal()
        p = lookat_goal.point

        if target.gaze_target == GazeRelayTarget.FACE:
            faces_mtx.acquire()
            p.header = Header(frame_id=faces.header.frame_id, stamp=rospy.Time.now())
            faces_mtx.release()
        elif target.gaze_target == GazeRelayTarget.ROBOT_LEFT_HAND:
            p.header = Header(frame_id='index_tip_left', stamp=rospy.Time.now())
        elif target.gaze_target == GazeRelayTarget.ROBOT_RIGHT_HAND:
            p.header = Header(frame_id='index_tip_right', stamp=rospy.Time.now())
        elif target.gaze_target == GazeRelayTarget.IOTP:
            iotp_mtx.acquire()
            if iotp is None:
                iotp_mtx.release()
                check_timeout()
                target_mtx.release()
                continue
            if rospy.Time.now() - iotp_time > rospy.Duration(max_target_time):
                print('iotp too old, skipping')
                iotp_mtx.release()
                check_timeout()
                target_mtx.release()
                continue
            target_point = iotp
            p.header = Header(frame_id=iotp_frame, stamp=rospy.Time.now())
            iotp_mtx.release()

        elif target.gaze_target == GazeRelayTarget.RIGHT_HAND:
            ppl_mtx.acquire()
            if ppl is None:
                ppl_mtx.release()
                check_timeout()
                target_mtx.release()
                continue
            target_point = ppl
            p.header = Header(frame_id=ppl_frame, stamp=rospy.Time.now())
            ppl_mtx.release()
        else:
            p.header = Header(frame_id=person.header.frame_id, stamp=rospy.Time.now())
        p.point.x = float(target_point.x)
        p.point.y = float(target_point.y)
        p.point.z = float(target_point.z)

        # print('looking at \n'+str(lookat_goal)+'\n')

        # send the goal to the server
        lookat_client.send_goal(lookat_goal)

    # before we continue, we check times
    check_timeout()
    target_mtx.release()
    continue
