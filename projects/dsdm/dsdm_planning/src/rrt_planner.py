#!/usr/bin/env python
import rospy
import numpy as np
from dsdm_msgs.msg import joint_position
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from AlexRobotics.planning import RandomTree    as RPRT
from AlexRobotics.dynamic  import Manipulator   as M



#########################################
class planner(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        self.sub_q_goal   = rospy.Subscriber("q_goal", joint_position , self.callback , queue_size=1      )
        self.pub_traj_sol = rospy.Publisher("traj", JointTrajectory , queue_size=1      )
        
        self.n_DOF   = 2 ;
        
        self.setup_RRT_algo()
        
        # Init
        self.q_goal = np.array( [0,0] )
        self.x_goal = np.array( [0,0,0,0] )
        
    
    #######################################   
    def update_goal( self, msg ):
        """ Read Goal  """
        
        self.q_goal[0] = msg.q[0]
        self.q_goal[1] = msg.q[1]
        
        self.x_goal[0] = msg.q[0]
        self.x_goal[1] = msg.q[1]
        
        if self.verbose:
            rospy.loginfo("Planner: New Goal Received")
            
            
    #######################################   
    def compute_solution( self):
        """ Set goal  """
        
        self.RRT.find_path_to_goal( self.x_goal )
        
        
    #######################################   
    def publish_solution( self ):
        
        traj_msg = JointTrajectory()
        
        # Build message
        n_pts = self.RRT.solution[0][0].size
        
        # For all times
        for i in range( n_pts ):
            
            traj_pt = JointTrajectoryPoint()
            
            t = self.RRT.solution[2][i] # Time
            traj_pt.time_from_start.secs  = int(t)
            traj_pt.time_from_start.nsecs = ( t - int(t) ) * 1000
            
            # For all DOF
            for j in range( self.n_DOF ):
                traj_pt.positions.append(     self.RRT.solution[0][j][i] )
                traj_pt.velocities.append(    self.RRT.solution[0][j+self.n_DOF][i] )
                traj_pt.accelerations.append( self.RRT.solution[3][j+self.n_DOF][i] )
                
            # For all actuators
            for j in range( self.RRT.DS.m ):
                traj_pt.effort.append(  self.RRT.solution[1][j][i] )
                
            traj_msg.points.append( traj_pt )
            
        traj_msg.joint_names.append('1')
        traj_msg.joint_names.append('2')
        
        self.pub_traj_sol.publish( traj_msg )
        
        if self.verbose:
            rospy.loginfo("Planner: Trajectory Solution Published ")
            
        self.RRT.plot_2D_Tree()
        
        
    #######################################   
    def callback( self, msg ):
        """ Set goal  """
        
        self.update_goal( msg )
        
        self.compute_solution()
        
        self.publish_solution()
        
        #self.RRT.plot_open_loop_solution()
        
        
        
        
    #######################################   
    def setup_RRT_algo( self ):
        """ Init algo with right params """
        
        # Robot model
        self.R  =  M.TwoLinkManipulator()
        
        #RRT algo
        self.RRT = RPRT.RRT( self.R , x_start = np.array([3,0,0,0]) )
        
        T = 12 # torque
        
        self.RRT.U = np.array([[T,0],[0,0],[-T,0],[0,T],[0,-T],[T,T],[-T,-T],[-T,T],[T,-T]])
        #RRT.U = np.array([[0,T],[0,-T],[0,0]]) # Acrobot problem
        
        self.RRT.dt                    = 0.1
        self.RRT.goal_radius           = 1.0
        self.RRT.max_nodes             = 15000
        self.RRT.max_solution_time     = 10
        
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('plan',anonymous=False)
    node = planner()
    rospy.spin()
