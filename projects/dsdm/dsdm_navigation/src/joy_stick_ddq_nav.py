#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg    import Bool
from dsdm_msgs.msg   import joint_position

#########################################
class nav(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = False
        self.dof     = 3
        
        # Publishers
        self.pub_enable   = rospy.Publisher("enable", Bool  , queue_size=1                   )
        self.pub_u        = rospy.Publisher("setpoint", joint_position  , queue_size=1       )
        
        # Suscribers
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1   )
        
        # Init
        self.mode         = 0                     # 0: acc 1: spd  2: position 3: traj        
        self.ddq_d        = np.zeros( self.dof )
        self.dd_d         = np.zeros( self.dof )
        self.q_d          = np.zeros( self.dof )
        self.setpoints    = np.zeros( self.dof )
        
    #######################################   
    def joy_callback( self, msg ):
        """ """
        
        # Enable button
        if ( msg.buttons[0] == 1 ):
            enable = True
        else:
            enable = False
        
        # Pick ctrl_mode with button state
        ######################
        if   ( msg.buttons[1] == 1 ):
            self.mode = 1
        elif ( msg.buttons[2] == 1 ):
            self.mode = 2
        elif ( msg.buttons[3] == 1 ):
            self.mode = 3
        else:
            self.mode = 0
                
        # Set points
        ######################
        if enable:
            # Pick set_point with joysticks gain
            self.setpoints[0]  =  msg.axes[1] * np.pi
            self.setpoints[1]  =  msg.axes[2] * np.pi
            self.setpoints[2]  =  msg.axes[3] * np.pi
            
        else:
            self.setpoints     =  np.zeros( self.dof )
        
        # From setpoint to deisred traj
        if   (self.mode == 0 ):
            self.q_d    = 0
            self.dq_d   = 0
            self.ddq_d  = self.setpoints * 50
            
        elif (self.mode == 1 ):
            self.q_d    = 0
            self.dq_d   = self.setpoints
            self.ddq_d  = 0
            
        elif (self.mode == 2 ):
            self.q_d    = self.setpoints
            self.dq_d   = 0
            self.ddq_d  = 0
            
        else:
            self.q_d    = 0
            self.dq_d   = 0
            self.ddq_d  = 0
            
        ###########################
        self.pub_e( enable )
        self.pub_u_msg()
        
        if self.verbose:
            
            print( 'setpoints: ', self.setpoints )
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg     = joint_position()

        msg.q   = self.q_d        
        msg.dq  = self.dq_d
        msg.ddq = self.ddq_d
        
        self.pub_u.publish( msg )
        
        
    #######################################   
    def pub_e( self , enable ):
        """ pub actuator cmd """
        
        msg     = Bool()
        
        msg.data = enable
        
        self.pub_enable.publish( msg )
        

            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('nav',anonymous=False)
    node = nav()
    rospy.spin()
