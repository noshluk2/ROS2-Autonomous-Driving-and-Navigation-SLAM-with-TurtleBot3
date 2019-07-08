#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    # Sequence
    goal_seq=[1.8878 , 0.031  , -0.001,
              3.5923 , -1.200 , -0.0010,
              1.4576 , -2.878 , -0.001,
              -1.000 , -2.623 ,-0.0010,
              0.0029 , 0.0040 , -0.001] # at the end get back to where it was parked :)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Looping in sequence
    for i in range(0,5):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_seq[0+i*3]
        goal.target_pose.pose.position.y = goal_seq[1+i*3]
        goal.target_pose.pose.position.z = goal_seq[2+i*3]
        goal.target_pose.pose.orientation.w = 0.09012465928 # fixed angles
        goal.target_pose.pose.orientation.z = 0.098917586 # of robot position for all goals

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server DOWN ;/ ")
        else:
            print("A Goal is Executed") 
        # looping each goal update as 
        # it end here if no loop is used
    return 1
if __name__ == '__main__':
    try:
        rospy.init_node('movebaseClient')
        result = movebase_client()
        if result:
            rospy.loginfo("All Goals executed ")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation DONE ")