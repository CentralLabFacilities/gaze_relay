#!/usr/bin/env python

import math
import rospy
import actionlib

from threading import Lock
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from hace_msgs.msg import MinimalHumans, MinimalHuman
from hlrc_server.msg import lookattargetAction, lookattargetGoal
from openface2_ros_msgs.msg import Faces
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
ppl_mtx = Lock()

faces = None
faces_mtx = Lock()

target = None
target_time = None
target_mtx = Lock()

target_pt = None
target_pt_time = None
target_pt_mtx = Lock()

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

    # we savely store the new people
    ppl_mtx.acquire(True)
    ppl = new_ppl
    rospy.loginfo('got {} new people'.format(len(ppl.humans)))
    ppl_mtx.release()


# callback for new list of people
def _on_new_faces(new_faces):

    global faces

    # we savely store the new faces
    faces_mtx.acquire(True)
    if new_faces.count > 0:
        faces = new_faces
        rospy.loginfo('got {} new faces'.format(faces.count))
    faces_mtx.release()


# callback for new gaze target
def _on_new_gaze_target(tar):
    print('### NEW TARGET ' + str(tar.gaze_target))

    global target
    global target_pt
    global target_time

    target_mtx.acquire()
    target_pt_mtx.acquire()

    target = tar
    target_pt = None

    target_time = rospy.Time.now()

    target_pt_mtx.release()
    target_mtx.release()


# callback for target points
def _on_new_gaze_target_point(point_stamped):
    print('### NEW TARGET POINT \n' + str(point_stamped))

    global target
    global target_pt
    global target_pt_time

    target_mtx.acquire()
    target_pt_mtx.acquire()

    target = None
    target_pt = point_stamped

    target_pt_time = rospy.Time.now()

    target_pt_mtx.release()
    target_mtx.release()


def check_timeout():
    global target
    global target_pt

    global target_time
    global target_pt_time

    if target_pt_time is not None and rospy.Time.now() - target_pt_time > rospy.Duration(max_target_time):
        print('resetting target point')
        target_pt_time = None
        target_pt = None

    if target_time is not None and rospy.Time.now() - target_time > rospy.Duration(max_target_time):
        print('resetting target')
        target_time = None
        target = None


# subscribers
people_sub = rospy.Subscriber('/hace/people', MinimalHumans, _on_new_people)
gtarget_sub = rospy.Subscriber('/gaze_relay/target', GazeRelayTarget, _on_new_gaze_target)
targetpoint_sub = rospy.Subscriber('/gaze_relay/target_point', PointStamped, _on_new_gaze_target_point)
faces_sub = rospy.Subscriber('/openface2/faces', Faces, _on_new_faces)

rospy.loginfo('starting control loop!')

while not rospy.is_shutdown():
    rospy.Rate(control_rate).sleep()

    # if we have a target point stored, just send it
    target_pt_mtx.acquire()

    if target_pt is not None:
        print('got target point ' + str(target_pt))

        lookat_goal = lookattargetGoal()
        lookat_goal.point = target_pt

        print('looking at \n'+str(lookat_goal)+'\n')

        lookat_client.send_goal(lookat_goal)

        # before we continue, we check times
        target_mtx.acquire()
        check_timeout()
        target_mtx.release()
        target_pt_mtx.release()
        continue

    target_pt_mtx.release()

    # now we check the target string
    target_mtx.acquire()

    if target is not None:
        print('got target ' + str(target.gaze_target) + ' with age ' + str(rospy.Time.now() - target_time))

        # check whether we even have pplz
        ppl_mtx.acquire()
        if (ppl is None or len(ppl.humans) == 0):
            if target.gaze_target != GazeRelayTarget.NEUTRAL and target.gaze_target != GazeRelayTarget.FACE:
                rospy.loginfo('We don\'t have any ppl stored.')
                ppl_mtx.release()
                target_pt_mtx.acquire()
                check_timeout()
                target_pt_mtx.release()
                target_mtx.release()
                continue
        ppl_mtx.release()

        # check whether we even have faces
        faces_mtx.acquire()
        if (faces is None or faces.count == 0):
            if target.gaze_target != GazeRelayTarget.NEUTRAL: # todo or == FACE?
                rospy.loginfo('We don\'t have any faces stored.')
                faces_mtx.release()
                target_pt_mtx.acquire()
                check_timeout()
                target_pt_mtx.release()
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
                target_pt_mtx.acquire()
                check_timeout()
                target_pt_mtx.release()
                target_mtx.release()
                continue
            else:
                target_point = faces.faces[0].head_pose.position
            faces_mtx.release()
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
                target_pt_mtx.acquire()
                check_timeout()
                target_pt_mtx.release()
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
        else:
            p.header = Header(frame_id=person.header.frame_id, stamp=rospy.Time.now())
        p.point.x = float(target_point.x)
        p.point.y = float(target_point.y)
        p.point.z = float(target_point.z)

        print('looking at \n'+str(lookat_goal)+'\n')

        # send the goal to the server
        lookat_client.send_goal(lookat_goal)

    # before we continue, we check times
    target_pt_mtx.acquire()
    check_timeout()
    target_pt_mtx.release()
    target_mtx.release()
    continue
