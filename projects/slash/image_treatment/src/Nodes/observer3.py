#!/usr/bin/env python

import math
import time
from image_treatment.msg import CamState
from image_treatment.msg import ImuStates
from image_treatment.msg import StatesHat
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


import numpy as np
import rospy
import sys


class Observer:
  ##########################################################

  def __init__(self):

    self.image_sub = rospy.Subscriber("ycam_msg",CamState,self.callback1)
    self.input_sub = rospy.Subscriber("cmd_vel",Twist,self.callback3)
    self.imu_sub = rospy.Subscriber("imu_msg",ImuStates,self.callback2)

    
    self.states_pub = rospy.Publisher("x_hat", StatesHat, queue_size=1)

    self.tlast = rospy.get_rostime()
    self.tgain = -10
    self.dgain = 15*3.1416/180
    self.x = np.array([[0],[0],[0]])
    self.torque = 0
    self.delta = 0

  ##########################################################

  
  def callback1(self,cam):
      self.cam_y     = cam.state_y

      
  ##########################################################

  
  def callback3(self, cmd):
      torque_cmd = cmd.linear.x
      self.torque     = self.tgain*torque_cmd # N/m
      delta_cmd  = cmd.angular.z
      self.delta      = self.dgain*delta_cmd  # degree


  ##########################################################

  
  def callback2(self,imu):
      self.imu_accx     = imu.state_accx
      self.imu_accy     = imu.state_accy
      self.imu_theta = imu.state_theta
      self.t = rospy.get_rostime()
      self.obs(self.cam_y,self.imu_accx,self.imu_accy,self.imu_theta,self.torque,self.delta, self.t)
     

  #######################################################

  def obs(self, cam_y, imu_accx, imu_accy, imu_theta, torque, delta, t):

     a = 0.3
     b = 0.8
     c1 = 1
     c2 = 1
     v = 1
     dt = (t-self.tlast).to_sec()
 
  #######################################################
  #---------------------State-Space---------------------#
  #######################################################

     A = np.array([[-c2,0.0,0.0],[0.0,0.0,v],[0.0,0.0,0.0]])
     B = np.array([[c1,0.0],[0.0,0.0],[0.0,v/b]])
     C = np.array([[0.0,1.0,0.0],[-c2,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,1.0]])
     D = np.array([[0.0,0.0],[c1,0.0],[0.0,-v**2],[0.0,0.0]])
     L = np.array([[0, 0, 0, 0],[1,0,0,0],[0,0,0,0]])

  #######################################################
  #-----------------------Inputs------------------------#
  #######################################################

     u = np.array([[torque],[delta]])

  #######################################################
  #-----------------------Sensors-----------------------#
  #######################################################

     y = np.array([[cam_y],[imu_accx],[imu_accy],[imu_theta]])

  #######################################################
  #----------------------Observer-----------------------#
  #######################################################
     
     
     x_hat = np.dot(B,u)*dt + np.dot(A,self.x)*dt + self.x + dt*(np.dot(L,y) - np.dot(L,np.dot(D,u)) - np.dot(L,np.dot(C,self.x)))
     self.x = x_hat 
     self.tlast = t

  #######################################################
  #----------------------Publisher----------------------#
  #######################################################

     msg = StatesHat()
     msg.header.stamp = rospy.Time.now()
     msg.state_Vx = x_hat[0]
     msg.state_y= x_hat[1]
     msg.state_theta = x_hat[2]
     self.states_pub.publish(msg)

  ##########################################################   

if __name__ == '__main__': 
    rospy.init_node('OBSERVER',anonymous=False)
    node = Observer()
    rospy.spin()
