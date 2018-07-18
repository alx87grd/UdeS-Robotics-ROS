#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys
import numpy as np
import scipy.integrate

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from scipy import integrate
acc=[]
t=[]
k=0
class Donnees_IMU:

  def __init__(self):
     self.jean_simon = rospy.Subscriber("imu",Imu, self.callback)

  def callback(self,words):
    try: 
      #k +=1   
      a=words.orientation
      c=words.linear_acceleration
      t.append(words.header.stamp)
      print k
      if len(t) <=2:
        print 'n.a.'
      else:
        dt=int(str(t[-1]-t[-2]))*10e-9
        acc.append(words.linear_acceleration.x+0.8)
        #print ('carl=',acc)

        if acc[-1] <=0.2 and acc[-1]>=-0.2:
          print 'carl'
	
        else:
        ## Determination de la vitesse
          v=integrate.cumtrapz(acc,x=None,dx=dt,initial=0)##scipy.integrate.simps(acc)
	## Determination de la vitesse
      #p=scipy.integrate.simps(v)
          print ('carl=',v)

    except KeyboardInterrupt:
      print("Shutting down")

def main(args):
  ic = Donnees_IMU()
  rospy.init_node('NODE_IMU', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
