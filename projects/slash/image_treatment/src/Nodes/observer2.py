#!/usr/bin/env python

import math
import time
from image_treatment.msg import CamState
from image_treatment.msg import ImuStates
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from numpy import linspace
from scipy import signal



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
      self.cam_y     = cam.state_y
      print(self.cam_y)
    except KeyboardInterrupt:
      print("Shut down")
      
  ##########################################################

  
  def callback2(self,imu):
    try:
      self.imu_accx     = imu.state_accx
      self.imu_accy     = imu.state_accy
      self.imu_theta = imu.state_theta
    except KeyboardInterrupt:
      print("Shut down")

  ##########################################################

  
  def callback3(self, cmd):
    try:
      torque_cmd = cmd.linear.x
      self.torque     = self.tgain*torque_cmd # N/m
      delta_cmd  = cmd.angular.z
      self.delta      = self.dgain*delta_cmd  # degree

      msgTest = ImuStates()
      msgTest.state_accx = torque
      msgTest.state_accy = delta
      msgTest.state_theta = 0
      
      self.test_pub.publish(msgTest)

      print(torque, delta)
      
    except KeyboardInterrupt:
      print("Shut down")

  #######################################################

  def obs(self, cam_y, imu_accx, imu_accy, imu_theta):

     a = 0.3
     b = 0.8
     v = 1
     dt = 0.05 #a definir???????????
  #######################################################
  #-----------------------States------------------------#
  #######################################################

     x = [[0],[0],[0]] # Initialisation des etats

  #######################################################
  #---------------------State-Space---------------------#
  #######################################################

     A = [[0,0,0],[0,0,1],[0,0,0]]
     B = [[0.0,0.666],[1.0,0.0],[6.666,0.0]]
     C = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
     D = [[0.0,0.0],[0.0,0.0],[0.0,0.0]]
     L = [[0.1],[0.001]]

  #######################################################
  #-----------------------Inputs------------------------#
  #######################################################

     delta = ??
     T_m = ??
     u = [[delta],[Tm]]

  #######################################################
  #----------------------Observer-----------------------#
  #######################################################

     y_est = C*x + D*u
  # x_dot = A*x + B*u + L*(y-y_est)


     x(t+1) = (1+A*dt)*x(t) + B*u(t)
     ixe = (1+A*dt)*x[-1] + B*u
     x.append(ixe) # Fait en sorte que le dernier element est le nouveau x



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
