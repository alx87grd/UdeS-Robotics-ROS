#!/usr/bin/env python

import math
import time
from image_setup.msg import CamState
from image_setup.msg import ImuStates
import numpy as np
import rospy
import sys


class Observer:
  ##########################################################

  def __init__(self):

    self.image_sub = rospy.Subscriber("ycam_msg",CamState,self.callback1)
    self.image_sub = rospy.Subscriber("imu_conversion",ImuStates,self.callback2)
  

  ##########################################################

  
  def callback1(self,cam):
    try:
      cam_y     = cam.state_y
      print(cam_y)
    except KeyboardInterrupt:
      print("Shut down")
      
  ##########################################################

  
  def callback2(self,imu):
    try:
      imu_V     = imu.state_V
      imu_y     = imu.state_y
      imu_theta = imu.state_theta
    except KeyboardInterrupt:
      print("Shut down")
  


  ##########################################################   

def main(args):
  ic = Observer()
  rospy.init_node('OBSERVER', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
