#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


#########################################
class teleop(object):
    """
    teleoperation
    """
    def __init__(self):
        
        self.verbose = False
        
        
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        
        self.pub_cmd      = rospy.Publisher("cmd_vel", Twist , queue_size=1  )
        
        
    #######################################   
    def joy_callback( self, joy_msg ):
        """ """
        
        # init cmd msg
        cmd_msg = Twist()
        
        # if trigger is active
        if ( joy_msg.axes[2] < 0 ):
            
            enable = True
            
            #mapping button --> cmd
            cmd_msg.linear.x = 0.5 + joy_msg.axes[1] 
            #cmd_msg.linear.y = 0
            #cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            #cmd_msg.angular.y = 0
            cmd_msg.angular.z = 0.5 + joy_msg.axes[4]
            
        else:
            
            enable = False
            
            cmd_msg.linear.x = 0.5 
            #cmd_msg.linear.y = 0
            #cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            #cmd_msg.angular.y = 0
            cmd_msg.angular.z = 0.5 
                   
        
        # Publish cmd msg
        self.pub_cmd.publish( cmd_msg )
            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('teleop',anonymous=False)
    node = teleop()
    rospy.spin()
