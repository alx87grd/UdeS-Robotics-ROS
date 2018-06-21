#!/usr/bin/env python
import rospy
import numpy as np

# Ros stuff
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg    import Float64MultiArray, Bool, Int32
from dsdm_msgs.msg   import joint_position, ctl_error
from dsdm_msgs.msg   import dsdm_actuator_sensor_feedback, dsdm_actuator_control_inputs


from AlexRobotics.dynamic  import CustomManipulator    as CM
from AlexRobotics.control  import RminComputedTorque   as RminCTC
from AlexRobotics.planning import RandomTree           as RPRT


#########################################
class CTC_controller(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        # Sub 
        self.sub_set            = rospy.Subscriber("setpoint"    , joint_position , self.update_setpoint ,  queue_size=1 )
        self.sub_enable         = rospy.Subscriber("enable"      , Bool           , self.update_enable   ,  queue_size=1 )
        self.sub_mode           = rospy.Subscriber("ctl_mode"    , Int32          , self.update_mode     ,  queue_size=1 )
        self.sub_state_feedback = rospy.Subscriber("x_hat"       , Float64MultiArray , self.update_state ,  queue_size=1 )
        
        # Pub
        #self.pub_control        = rospy.Publisher("u", dsdm_robot_control_inputs , queue_size=1   )     
        self.pub_control        = rospy.Publisher("a2/u", dsdm_actuator_control_inputs , queue_size=1   )  # Temp for testing, only one actuator
        #self.pub_control        = rospy.Publisher("CTC_U", dsdm_actuator_control_inputs , queue_size=1   ) 
        self.pub_e              = rospy.Publisher("ctc_error", ctl_error               , queue_size=1  )
        
        # Timer
        #self.timer              = rospy.Timer( rospy.Duration.from_sec(0.01),    self.callback  )
        
        # Load ROS params
        self.load_params( None )
        
        # Assign controller
        if self.robot_type == 'BoeingArm':
            self.R       = CM.BoeingArm()
            self.CTC     = RminCTC.RminComputedTorqueController( self.R )
            
        elif self.robot_type == 'pendulum':
            self.R       = CM.TestPendulum()
            self.CTC     = RminCTC.RminComputedTorqueController( self.R )
            
            # Temp to load traj
            self.RRT = RPRT.RRT( self.R , np.array( [0,0,0,0,0,0] ) )
            self.RRT.load_solution( '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_control/data/pendulum_traj_test0.npy'  )
        
        else:
            print('Error loading robot type')
        
        # Load params
        self.CTC.w0            = 8.0
        self.CTC.zeta          = 0.7
        self.CTC.n_gears       = 2              ##########
        
        self.CTC.hysteresis    = True
        self.CTC.hys_level     = 0.001
        self.CTC.min_delay     = 0.2
        
        self.CTC.R.dq_max_HF   = 1.3   # [rad/sec]
        
        # Gear params
        R_HS  = self.CTC.R.R[0]
        R_HF  = self.CTC.R.R[1]
        
        #self.CTC.last_gear_i = 0
        #self.CTC.R.R = [ R_HS ] # only HF
        
        self.R_HS = R_HS
        self.R_HF = R_HF
        
        # INIT
        self.t_zero   =  rospy.get_rostime()
        self.x        =  np.array([ 0 , 0 , 0 , 0 , 0 , 0])
        self.ddq_d    =  np.array([ 0 , 0 , 0 ])
        self.dq_d     =  np.array([ 0 , 0 , 0 ])
        self.q_d      =  np.array([ 0 , 0 , 0 ])
        self.enable   =  False
        self.ctl_mode = 0
        self.setpoint = 0
        self.actual   = 0
        self.e        = 0
        self.de       = 0
        
        self.traj_started = False
        
        # Save data
        # self.data = np.zeros( ( self.R.n + self.R.m + 1 , )
        
        
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.robot_type     = rospy.get_param("robot_type",  'pendulum'  )

        
    #######################################   
    def callback( self, event ):
        """ Timed controller response """
        
        # State feedback
        x = self.x
        
        # Get time
        t_ros = rospy.get_rostime() - self.t_zero
        t     = t_ros.to_sec()
        
        # Compute u
        
#        # test hack
#        if t > 3:
#            self.CTC.R.R = [ self.R_HS , self.R_HF ]
#            self.CTC.n_gears       = 2 
        
        # ACC setpoint
        if ( self.ctl_mode == 0 ) :
            
            self.CTC.ddq_manual_setpoint = self.ddq_d
            
            u = self.CTC.manual_acc_ctl(  x , t )
            
            # Debug
            self.actual   = x[0]
            self.setpoint = self.ddq_d[0]
        
        # SPEED setpoint
        elif ( self.ctl_mode == 1 ) :
            
            # Update setpoint to actual position + desired speed
            [ q , dq ]    = self.R.x2q( self.x  )
            self.CTC.goal = self.R.q2x( q , self.dq_d )
            
            u = self.CTC.fixed_goal_ctl( x  , t )

            # Debug
            self.actual   = dq[0]
            self.setpoint = self.dq_d[0]
            self.e        = self.CTC.dq_e[0]
            
        # POS setpoint
        elif ( self.ctl_mode == 2 ) :
            
            [ q , dq ]    = self.R.x2q( self.x  )
            self.CTC.goal = self.R.q2x( self.q_d , self.dq_d )
            
            u = self.CTC.fixed_goal_ctl( x  , t )

            # Debug
            self.actual   = q[0]
            self.setpoint = self.q_d[0]
            self.e        = self.CTC.q_e[0]
            
        # Traj mode
        elif ( self.ctl_mode == 3 ) :
            
            if not( self.traj_started ) :
                self.traj_init()
                
            u = self.CTC.ctl( x , t )
                
            #u = self.R.ubar
                
            # Debug
            self.actual         = x[0]
            ddq_d , dq_d , q_d  = self.CTC.get_traj(t)
            self.setpoint       = q_d[0]
            self.e              = self.CTC.q_e[0]
        
#        # always high-force
#        if t > 3:
#            pass
#        else:
#            u[ self.R.dof ] = 0
        
        #u[ self.R.dof ] = 0
        
        # Publish u
        self.pub_u( u )
        
        
        ##################
        if self.verbose:
            print(u)
            #rospy.loginfo("Controller: e = " + str(self.CTC.q_e) + "de = " + str(self.CTC.dq_e) + "ddr =" + str(self.CTC.ddq_r) + " U = " + str(u) )
        
        #xt  = np.append( x  , t )
        #xtu = np.append( xt , u )
    
    #######################################   
    def traj_init( self ):
        """ init traj following mode """
        
        self.traj_started = True
        
        self.CTC.load_trajectory( self.RRT.solution )
        self.CTC.goal  = np.array([0,0,0,0,0,0])
        self.t_zero    =  rospy.get_rostime()
        
    
    #######################################   
    def update_state( self, msg ):
        """ state feedback """
        
        self.x = msg.data
        
        # Asynchrone control
        self.callback( None )
        
        
    #######################################   
    def update_enable( self, msg ):
        """ enable  """
        
        self.enable = msg.data
        
        
    #######################################   
    def update_mode( self, msg ):
        """ mode """
        
        self.ctl_mode = msg.data
        
        
        
    #######################################   
    def update_setpoint( self, msg ):
        """ Load Ref setpoint """
        
        self.ddq_d = np.array( msg.ddq )
        self.dq_d  = np.array( msg.dq  )
        self.q_d   = np.array( msg.q   )        
            
            
    #######################################   
    def pub_u( self, u ):
        """ pub control inputs """
        
        #msg = dsdm_robot_control_inputs()

        #self.pub_control.publish( msg )
        
        # Testing 1-DOF 1-DSDM
        msg = dsdm_actuator_control_inputs()
        
        if self.enable:
        
            msg.f  = u[0] 
            msg.k  = u[3]  # mode
            
        else:
            
            msg.f  = 0
            msg.k  = 1
        
        
        self.pub_control.publish( msg )
        
        # Publish error data
        msg              = ctl_error()
        msg.header.stamp = rospy.Time.now()
        msg.set_point    = self.setpoint
        msg.actual       = self.actual
        msg.e            = self.e  #self.CTC.q_e[0]
        msg.de           = self.de #self.CTC.dq_e[0]
        
        self.pub_e.publish( msg )
        
        if self.verbose:
            #print('Error:', self.e )
            pass
        
            
    
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('Master_Controller',anonymous=False)
    node = CTC_controller()
    rospy.spin()
