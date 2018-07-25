#!/usr/bin/env python

import sys
from std_msgs.msg import String
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("gray_msg", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("cam_img_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    except CvBridgeError as e:
      print(e)

    cv2.imshow("GRAY", gray)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('BGR_to_GRAY', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
