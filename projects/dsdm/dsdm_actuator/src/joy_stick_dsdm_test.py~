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
        
        self.verbose = True
        
        self.pub_u        = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        
        # Init DSDM msg
        self.f = 0
        self.k = 1 # High force mode
        
    #######################################   
    def joy_callback( self, msg ):
        """ """
        
        # Pick ctrl_mode with button state
        if ( msg.buttons[0] == 1 ):
            enable = True
        else:
            enable = False
        
        ######################
        if enable:
            # Pick set_point with joysticks gain
            self.f  =    int( msg.axes[1] * 1000 )
                
            # Pick mode with trigger
            if ( msg.axes[5] < 0):
                self.k = 0
                self.f = self.f * 0.5
            else:
                self.k = 1
                
        else:
            self.f = 0
            self.k = 1
        ###########################
        
        
        self.pub_u_msg()
        
        if self.verbose:
            
            print( 'Ctrl mode: ', self.k , ' Set point: ', self.f )
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg = dsdm_actuator_control_inputs()
        
        msg.f = self.f
        msg.k = self.k
        
        self.pub_u.publish( msg )
        

            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('nav',anonymous=False)
    node = nav()
    rospy.spin()
