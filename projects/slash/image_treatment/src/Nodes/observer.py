#!/usr/bin/env python

import math
import time
from image_treatment.msg import CamState
from image_treatment.msg import ImuStates
from geometry_msgs.msg import Twist

import numpy as np
import rospy
import sys


class Observer:
  ##########################################################

  def __init__(self):

    self.image_sub = rospy.Subscriber("ycam_msg",CamState,self.callback1)
    self.imu_sub = rospy.Subscriber("imu_msg",ImuStates,self.callback2)
    self.input_sub = rospy.Subscriber("cmd_vel",Twist,self.callback3)
    
    self.test_pub = rospy.Publisher("test_gain", ImuStates, queue_size=1)


    self.tgain = -10
    self.dgain = 15
    
  

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
      imu_accx     = imu.state_accx
      imu_accy     = imu.state_accy
      imu_theta = imu.state_theta
    except KeyboardInterrupt:
      print("Shut down")

  ##########################################################

  
  def callback3(self, cmd):
    try:
      torque_cmd = cmd.linear.x
      torque     = self.tgain*torque_cmd # N/m
      delta_cmd  = cmd.angular.z
      delta      = self.dgain*delta_cmd  # degree

      msgTest = ImuStates()
      msgTest.state_accx = torque
      msgTest.state_accy = delta
      msgTest.state_theta = 0
      
      self.test_pub.publish(msgTest)

      print(torque, delta)
      
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
