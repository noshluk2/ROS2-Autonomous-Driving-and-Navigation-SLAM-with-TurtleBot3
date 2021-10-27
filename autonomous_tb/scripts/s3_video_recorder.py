#!/usr/bin/env python3

####
# - Main purpse is save video of the enviornment using CAMERA sensor plugin 
# - Image are recieved as ROS mesg type and converted to OPENCV type then saved into a video
# - Code is a Subscribe "camera/rgb/image_raw" topic to obtain frames
#
#  Written by Muhammad Luqman
# 
#  13/6/21
#
###
import rospy
import cv2 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 


class Video_get():
  def __init__(self):
    ## Created a subscriber 
    self.subscriber = rospy.Subscriber("/camera/rgb/image_raw",Image,self.process_data,10)
    ## setting for writing the frames into a video
    self.out = cv2.VideoWriter('/home/luqman/output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))
    self.bridge = CvBridge() # converting ros images to opencv data
 
  ## Subscriber callback function 
  def process_data(self, data,a): 
    # performing conversion
    frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
    # write the frames to a video
    self.out.write(frame)
    # displaying what is being recorded 
    cv2.imshow("output", frame) 
    # will save video until it is interrupted
    cv2.waitKey(1) 
  

  
def main(args=None):
  print("Video Recording")
  rospy.init_node('Video_Recording_Node')
  image_subscriber = Video_get()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
  main()