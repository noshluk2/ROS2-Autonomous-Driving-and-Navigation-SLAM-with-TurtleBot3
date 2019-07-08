#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 3.6014963
    goal.target_pose.pose.position.y = -1.156947665
    goal.target_pose.pose.position.z = -0.00100955

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