#!/usr/bin/env python
import rospy
import numpy as np

# Ros stuff
from std_msgs.msg    import Float64MultiArray, Bool, Int32
from dsdm_msgs.msg   import joint_position, ctl_error
from dsdm_msgs.msg   import dsdm_actuator_sensor_feedback, dsdm_actuator_control_inputs
from std_msgs.msg    import Float64MultiArray
from dsdm_msgs.msg   import ctl_error


from flexsea_execute.msg import inputs
from flexsea_execute.msg import outputs




#########################################
class logger(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        # Sub 
        self.sub_state_feedback  = rospy.Subscriber("x_hat"       , Float64MultiArray             , self.update_state         ,  queue_size=1 )
        self.sub_u1              = rospy.Subscriber("a1/u"        , dsdm_actuator_control_inputs  , self.update_u1             , queue_size=1 )
        self.sub_u2              = rospy.Subscriber("a2/u"        , dsdm_actuator_control_inputs  , self.update_u2             , queue_size=1 )
        
        # Timer
        self.timer              = rospy.Timer( rospy.Duration.from_sec(1.0),    self.save )
        
        
        # INIT
        self.DATA = np.zeros((1,9))
        
        
        name      = '2dof_test4'
        
        self.name = '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_sensing/data/' + name + '.npy'
        
    #######################################   
    def save( self , event ):
        """ SAVE DATA at each sec  """ 
        
        np.save( self.name , self.DATA )
        
        print "DATA saved"
        
        
    #######################################   
    def logging( self  ):
        """  """
         
        t     = rospy.get_rostime().to_sec()
        
        data = np.array([[ self.q1 , self.dq1 , self.q2 , self.dq2 , self.f1 , self.k1, self.f2 , self.k2 , t ]])
        
        self.DATA = np.concatenate( ( self.DATA , data )  )
        
    
    #######################################
    def update_u1( self, msg ):
        
        self.f1 = msg.f
        self.k1 = msg.k
        
    #######################################
    def update_u2( self, msg ):
        
        self.f2 = msg.f
        self.k2 = msg.k
        
        
    #######################################   
    def update_state( self, msg ):
        """ state feedback """
        
        x = msg.data
        
        self.q1  = x[1]
        self.dq1 = x[4] 
        self.q2  = x[2]
        self.dq2 = x[5] 
        
        self.logging()
            
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('logger',anonymous=False)
    node = logger()
    rospy.spin()
