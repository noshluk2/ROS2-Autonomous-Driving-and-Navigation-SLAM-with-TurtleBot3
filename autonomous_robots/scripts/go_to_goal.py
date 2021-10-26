#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    argv = sys.argv[1:]
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(argv[0])
    goal.target_pose.pose.position.y = float(argv[1])
    goal.target_pose.pose.position.z = 0.000

    goal.target_pose.pose.orientation.w = 0.19012465928
    goal.target_pose.pose.orientation.z = 0.998917586



    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server DOWN ;/ ")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebaseClient')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal is EXECUTED :l ")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation DONE ")