#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
def movebase_client():
    arguments=sys.argv[1:]
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(arguments[0]) - 0.5
    goal.target_pose.pose.position.y = float(arguments[1])   
    goal.target_pose.pose.position.z = 0

    goal.target_pose.pose.orientation.w = -0.3
    goal.target_pose.pose.orientation.z = 0.95



    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server DOWN ;/ ")
    # else:
    #     return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebaseClient')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal is EXECUTED :D ")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation DONE ")