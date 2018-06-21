#!/usr/bin/env python
import rospy
import numpy as np
from dsdm_msgs.msg import joint_position
from sensor_msgs.msg import Joy

#########################################
class nav(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        self.pub_q_goal   = rospy.Publisher("q_goal", joint_position , queue_size=1      )
        
        # Timer
        self.pub_timer    = rospy.Timer( rospy.Duration.from_sec(15.0),    self.callback  )
        
        # Joystick
        #self.sub_joy   = rospy.Subscriber("joy", Joy , self.callback , queue_size=1      )
        
        self.n_DOF   = 2 ;
        self.q_max   = np.array([ 4, 4])
        self.q_min   = np.array([-4,-4])
        self.q_range = self.q_max - self.q_min
        
        
    #######################################   
    def callback( self, msg ):
        """ Publish a random goal """
        
        # Generate a random goal
        q_goal = self.random_goal()
        
        """
        if msg.buttons[0] == 1 :
                
            q_goal   = np.array([ 1, 1])
            
            # Publish the goal
            self.q2msg( q_goal )
            
        elif msg.buttons[1] == 1 :
                
            q_goal   = np.array([ -1, -1])
        """
            
        # Publish the goal
        self.q2msg( q_goal )
        
        
    #######################################   
    def random_goal( self ):
        """ Generate a random goal """
        
        q_rand = np.random.random( self.n_DOF ) * self.q_range + self.q_min 
        
        return q_rand   
        
        
    ############################
    def q2msg( self, q_goal = np.array( [ 0 , 0 ,] ) ):
        """ Convert numpay array to joint_position ROS message """
        
        msg = joint_position()
        
        msg.header.stamp = rospy.Time.now() # mark time
        msg.n = self.n_DOF 
        
        for i in range( msg.n ):
            msg.q.append( q_goal[i] )
            
        self.pub_q_goal.publish( msg )
        
        if self.verbose:
            rospy.loginfo("Navigation: Target " + str(q_goal) +" Published ")
            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('nav',anonymous=False)
    node = nav()
    rospy.spin()
