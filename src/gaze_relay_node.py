#!/usr/bin/env python

import math
import rospy
import actionlib

from threading import Lock
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from hace_msgs.msg import MinimalHumans
from hlrc_server.msg import lookattargetAction, lookattargetGoal
from gaze_relay.msg import GazeRelayTarget

ppl = None
ppl_mtx = Lock()

rospy.init_node('gaze_relay')

lookat_client = actionlib.SimpleActionClient('/floka/set/lookat', lookattargetAction)
rospy.loginfo('waiting for floka action server')
lookat_client.wait_for_server()
rospy.loginfo('Action server found!')

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

# callback for new gaze target
def _on_new_gaze_target(tar):

    # check whether we even have pplz
    ppl_mtx.acquire()
    if ppl is None or len(ppl.humans) == 0:
        rospy.loginfo('We don\'t have any ppl stored.')
        ppl_mtx.release()
        return
    ppl_mtx.release()

    # if we dont find the person
    if _person_id_in_list(tar.person_id):
        person = _get_person_by_id(tar.person_id)
        rospy.loginfo('found person with id {}.'.format(tar.person_id))
    else:
        person = ppl.humans[0]
        rospy.loginfo('no person with id {}! Using first person in list.'.format(tar.person_id))

    if tar.gaze_target == GazeRelayTarget.NEUTRAL:
        target_point = Point(x=0, y=0, z=1.60)
        rospy.logerr('current neutral target point is just 0 0 0. THIS NEEDS TO CHANGE!')
    elif tar.gaze_target == GazeRelayTarget.FACE:
        target_point = person.face.position
    elif tar.gaze_target == GazeRelayTarget.LEFT_HAND:
        target_point = person.left_hand.position
    elif tar.gaze_target == GazeRelayTarget.RIGHT_HAND:
        target_point = person.right_hand.position
    elif tar.gaze_target == GazeRelayTarget.TORSO:
        target_point = person.torso.position
    else:
        rospy.logerr('unknown gaze target {}. Use constants defined in GazeRelayTarget msg.')
        return

    for v in (target_point.x, target_point.y, target_point.z):
        if math.isnan(v):
            rospy.logerr('target was nan')
            return

    # construct goal
    lookat_goal = lookattargetGoal()
    p = lookat_goal.point

    p.header = Header(frame_id=person.header.frame_id, stamp=rospy.Time.now())
    p.point.x = float(target_point.x)
    p.point.y = float(target_point.y)
    p.point.z = float(target_point.z)

    # send the goal to the server
    lookat_client.send_goal(lookat_goal)

# callback for target points
def _on_new_gaze_target_point(point_stamped):
    lookat_goal = lookattargetGoal()
    lookat_goal.point = point_stamped

    lookat_client.send_goal(lookat_goal)

# subscribers
people_sub = rospy.Subscriber('/hace/people', MinimalHumans, _on_new_people)
gtarget_sub = rospy.Subscriber('/gaze_relay/target', GazeRelayTarget, _on_new_gaze_target)
targetpoint_sub = rospy.Subscriber('/gaze_relay/target_point', PointStamped, _on_new_gaze_target_point)

rospy.loginfo('We\'re spinning now ...')
# spinner
rospy.spin()
