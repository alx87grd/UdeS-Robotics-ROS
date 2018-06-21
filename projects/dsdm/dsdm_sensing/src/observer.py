#!/usr/bin/env python
"""
Created on Sat Aug 20 14:47:23 2016

@author: alex
"""

import rospy
import numpy as np
from dsdm_msgs.msg import dsdm_actuator_sensor_feedback
from std_msgs.msg  import Float64MultiArray

from AlexRobotics.dynamic  import CustomManipulator    as CM
from AlexRobotics.dynamic  import Prototypes           as Proto

import matplotlib
import matplotlib.pyplot as plt

#import mtTkinter as Tkinter

#########################################
class DSDM_OBS(object):
    """
    Observer Node for DSDM :
    
    listen to DSDM actuator outputs
    publish state estimates
    
    """
    
    #################################
    def __init__(self):
        
        self.verbose = False
        self.plot    = True
        
        # Message outgoing
        self.pub_x          = rospy.Publisher("x_hat", Float64MultiArray , queue_size=1      )
        
        # Messages Inputs        
        self.sub_a0         = rospy.Subscriber("a0/y", dsdm_actuator_sensor_feedback , self.feedback_callback_a0 , queue_size=1 )
        self.sub_a1         = rospy.Subscriber("a1/y", dsdm_actuator_sensor_feedback , self.feedback_callback_a1 , queue_size=1 )
        self.sub_a2         = rospy.Subscriber("a2/y", dsdm_actuator_sensor_feedback , self.feedback_callback_a2 , queue_size=1 )
        
        # Param Timer
        self.timer          = rospy.Timer( rospy.Duration.from_sec(2.0),    self.load_params  )
        
        # Load params
        self.load_params( None )
        
        # Load robot params
        if self.robot_type == 'BoeingArm':
            self.R                   = CM.BoeingArm()
            self.plot_partial_config = True
            
        elif self.robot_type == 'pendulum':
            #self.R                   = CM.TestPendulum()
            #self.plot_partial_config = False
            self.R                   = CM.TestPendulum()
            self.plot_partial_config = True
        
        else:
            print('Error loading robot type')
            self.plot_partial_config = False

        # Variable Init
        self.a      = np.zeros( self.R.dof )       
        self.da     = np.zeros( self.R.dof )
        self.q      = np.zeros( self.R.dof )       
        self.dq     = np.zeros( self.R.dof )
        self.x_hat  = self.R.q2x( self.q , self.dq )
        
        if self.plot:
            self.R.show_3D( self.q )
            
            
        # Partial Manipulator
        if self.plot_partial_config :
            
            if self.robot_config == 'wrist-only':
                self.Rp = Proto.SingleRevoluteDSDM()
                self.qp = np.array( [ 0 ] )
                
            elif self.robot_config == 'dual-plane' :
                self.Rp = Proto.TwoPlanarSerialDSDM()
                self.qp = np.array( [ 0 , 0 ] )
                
            self.Rp.show( self.qp )
            
        
            
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.robot_type     = rospy.get_param("robot_type",  'BoeingArm'  )
        self.robot_config   = rospy.get_param("robot_config",  'wrist-only'  )
        a0_zero             = rospy.get_param("a0_zero",  0.05 )
        a1_zero             = rospy.get_param("a1_zero",  0.00 )
        a2_zero             = rospy.get_param("a2_zero",  0.00 )
        
        self.a_zero = np.array( [ a0_zero , a1_zero , a2_zero ] ) 
            
            
    ###################################
    def main_callback(self):
        """ """
        
        # Extimate state vector
        self.estimate_state()
            
        # Publish state estimate
        self.pub_state_estimate()
    
    ###################################
    def estimate_state(self ):
        """ """
            
        # Just pure kinematic for boeing Arm
            
        # Avoid interpolation error blocking the feedback loop code
        try:
            self.q     = self.R.a2q(   self.a  + self.a_zero )
            self.dq    = self.R.da2dq( self.da , self.q )
        except:
            
            print('Kinematic of 4-bar mechanism outside of interpol range')
            self.q     = np.zeros( self.R.dof ) 
            self.dq    = np.zeros( self.R.dof ) 
        else:
            pass
        
        
        self.x_hat = self.R.q2x( self.q , self.dq   )  

        if self.plot_partial_config :
            
            if self.robot_config == 'wrist-only':
                self.qp = np.array( [ self.q[2] ] )
                
            elif self.robot_config == 'dual-plane' :
                self.qp = np.array( [ self.q[1] , self.q[2]  ] )
            
            
        
        
    ###################################
    def plot_update_callback(self , event ):
        """ """
        
        # Update graphic
        if self.plot:
            self.R.update_show_3D( self.q )
            
            if self.plot_partial_config :
                self.Rp.update_show( self.qp  )
        
    
    #######################################   
    def pub_state_estimate( self ):
        """ pub control inputs """

        msg = Float64MultiArray()
        
        msg.data = self.x_hat
        
        self.pub_x.publish( msg )
        
        
    ###################################
    def feedback_callback_a0(self, msg ):
        """ """
        
        self.a[0]  = msg.a
        self.da[0] = msg.da
        
        # Evante based timing
        self.main_callback()
        
    ###################################
    def feedback_callback_a1(self, msg ):
        """ """
        
        self.a[1]  = msg.a
        self.da[1] = msg.da
        
    ###################################
    def feedback_callback_a2(self, msg ):
        """ """
        
        self.a[2]  = msg.a
        self.da[2] = msg.da
        
        # Evante based timing
        if self.robot_type == 'pendulum':
            self.main_callback()
        
        
#########################################
if __name__ == '__main__':
    
    plt.ion()
    rospy.init_node('obs',anonymous=False)
    node = DSDM_OBS()
    #rospy.spin()
    
    rate = rospy.Rate(100) # 10hz
    i    = 0
    while not rospy.is_shutdown():
        
        # Plot at 10 Hz
        if node.plot & (i > 10):
            node.plot_update_callback( None )
            i = 0
        else:
            i = i + 1
        rate.sleep()
