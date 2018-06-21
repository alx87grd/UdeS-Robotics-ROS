#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from dsdm_msgs.msg import dsdm_actuator_control_inputs

#########################################
class nav(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = False
        
        
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        
        self.pub_a0u      = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_a1u      = rospy.Publisher("a1/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_a2u      = rospy.Publisher("a2/u", dsdm_actuator_control_inputs , queue_size=1  )
        
        # Init DSDM msgs
        self.f = np.array([0.,0.,0.])
        self.k = np.array([1,1,1]) # High force mode
        self.n = np.array([0.,0.,0.])
        
    #######################################   
    def joy_callback( self, msg ):
        """ """
        
        # Pick ctrl_mode with button state
        if ( msg.axes[2] < 0 ):
            enable = True
        else:
            enable = False
        
        ######################
        if enable:
            # Pick set_point with joysticks gain
        
            """ Ball screw DoF """
            #self.f  =    msg.axes[1] * 0.2
            self.f[0] = msg.buttons[4] * 0.2 + msg.buttons[5] * -0.2 
                
            # Pick mode with trigger
            if ( msg.axes[5] < 0):
                self.k[0] = 0
            else:
                self.k[0] = 1
                
            """ Shoulder """
            
            self.f[1] = msg.axes[1] * 0.2
            self.k[1] = not( msg.buttons[0] )
            self.n[1] = msg.axes[0] * 0.5
            
            """ Elbow """
            
            self.f[2] = msg.axes[4] * 0.2
            self.k[2] = not( msg.buttons[1] )
            self.n[2] = msg.axes[3] * 0.5
            
                
        else:
            self.f = np.array([0.,0.,0.])
            self.k = np.array([1,1,1])
            self.n = np.array([0.,0.,0.])
        ###########################
        
        
        self.pub_u_msg()
        
        if self.verbose:
            
            print( 'Ctrl mode: ', self.k , ' Set point: ', self.f )
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg0 = dsdm_actuator_control_inputs()
        msg1 = dsdm_actuator_control_inputs()
        msg2 = dsdm_actuator_control_inputs()
        
        msg0.f = self.f[0]
        msg0.k = self.k[0]
        
        msg1.f = self.f[1]
        msg1.k = self.k[1]
        msg1.n = self.n[1]
        
        msg2.f = self.f[2]
        msg2.k = self.k[2]
        msg2.n = self.n[2]
        
        self.pub_a0u.publish( msg0 )
        self.pub_a1u.publish( msg1 )
        self.pub_a2u.publish( msg2 )
        

            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('nav',anonymous=False)
    node = nav()
    rospy.spin()
