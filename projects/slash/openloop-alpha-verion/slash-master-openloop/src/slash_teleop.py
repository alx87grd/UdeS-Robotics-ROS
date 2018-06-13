#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy


#########################################
class teleop(object):
    """
    teleoperation
    """
    def __init__(self):
        
        self.verbose = False
        
        
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        
        #self.pub_a0u      = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1  )
        
        
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
	    pass
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        pass
        

            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('teleop',anonymous=False)
    node = teleop()
    rospy.spin()
