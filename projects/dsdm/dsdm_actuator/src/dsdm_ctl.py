#!/usr/bin/env python
import rospy
import numpy as np
from dsdm_msgs.msg import dsdm_actuator_control_inputs
from dsdm_msgs.msg import dsdm_actuator_sensor_feedback
from flexsea_execute.msg import inputs
from flexsea_execute.msg import outputs

#########################################
class DSDM_CTL(object):
    """
    Controller for DSDM :
    
    received DSDM control inputs
    send msg to two FlexSEA Driver Node
    """
    
    #################################
    def __init__(self):
        
        self.verbose = False
        
        # Message outgoing
        self.pub_cmd_m1     = rospy.Publisher("M1/u", inputs , queue_size=1      )
        self.pub_cmd_m2     = rospy.Publisher("M2/u", inputs , queue_size=1      )
        self.pub_y          = rospy.Publisher("y", dsdm_actuator_sensor_feedback , queue_size=1      )
        
        # Messages Inputs        
        self.sub_u          = rospy.Subscriber("u", dsdm_actuator_control_inputs , self.setpoint_callback , queue_size=1 )
        self.sub_y_m1       = rospy.Subscriber("M1/y", outputs , self.feedback_callback_m1 , queue_size=1 )
        self.sub_y_m2       = rospy.Subscriber("M2/y", outputs , self.feedback_callback_m2 , queue_size=1 )
        
        # Timers
        self.param_timer    = rospy.Timer( rospy.Duration.from_sec(1.0),    self.load_params  )
        
        #########################
        # Params
        
        self.load_params( None )

        ##########################
        # Variables initialization
        self.INIT          = True
        self.sync_err_i    = 0

        # Set point
        self.f = 0
        self.k = 1
        self.n = 0
        
        # Feedback from sensors
        self.y_raw  = np.zeros(3)  # M1 = [1], M2 = [1]
        self.w_raw  = np.zeros(3)  # Velocity
        self.y      = np.zeros(3)  # M1 = [1], M2 = [1]
        self.w      = np.zeros(3)  # Velocity
        self.a      = 0
        self.da     = 0
        self.k_real = 1
        
        # Memory for speed filter
        self.y_past = np.zeros(3)  # t-1 value
        self.w_x    = np.zeros(3)  # state of the filter        
        
        # Init flexsea input msg
        
        # Shared data
        self.ctrl_gains      = [10,0,0,0,0,0]
        self.trap_mode       = False
        trap_spd             = 50000
        trap_acc             = 50000
        self.trap_values     = [ 0 , 0 , 0 , trap_spd , trap_acc , 0 ]
        
        # Idividual data
        self.setpoints      = [0,0]
        self.ctrl_modes     = [0,0]
        self.brake_state    = 0
        
        # Time
        self.t_last = rospy.get_rostime()
        
        # Brake state
        self.time_brake_opened = 0
        
        
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.filter_rc       = rospy.get_param("filter_rc",        0.05  )
        self.filter_active   = rospy.get_param("filter_active",    True  )
        tpt                  = rospy.get_param("ticks_per_turns",   2000 )
        out_corr             = rospy.get_param("output_units_corr",    1 )
        r1                   = rospy.get_param("r1",                   1 )
        r2                   = rospy.get_param("r2",                   1 )
        rout                 = rospy.get_param("rout",                 1 )
        m1_sign              = rospy.get_param("m1_sign",              1 )
        m2_sign              = rospy.get_param("m2_sign",              1 )
        out_sign             = rospy.get_param("out_sign",             1 )
        m1_torque_sign       = rospy.get_param("m1_torque_sign",       1 )
        m2_torque_sign       = rospy.get_param("m2_torque_sign",       1 )
        kp                   = rospy.get_param("kp",                  10 )
        ki                   = rospy.get_param("ki",                   0 )
        kd                   = rospy.get_param("kd",                   0 )
        self.mbrake          = rospy.get_param("mbrake",               2 )
        self.ctl_mode        = rospy.get_param("ctl_mode",             1 )
        m1_torque_gain       = rospy.get_param("m1_torque_gain",       1 )
        m2_torque_gain       = rospy.get_param("m2_torque_gain",       1 )
        self.mA2units        = rospy.get_param("mA2units",             1 )
        m1_max_current       = rospy.get_param("m1_max_current",  1000   )
        m2_max_current       = rospy.get_param("m2_max_current",  1000   )
        
        #Motor signs
        self.signs        = np.array( [ out_sign, m1_sign, m2_sign] )
        self.torque_gains = np.array( [ 0 ,  m1_torque_sign * m1_torque_gain, m2_torque_sign * m2_torque_gain] )
        
        # Gear ratios [output,M1,M2] (ticks to output units)
        self.corr  = np.array([ out_corr , ( 2. * np.pi ) / tpt  , ( 2. * np.pi ) / tpt ])
        self.g     = np.array([  rout    , 1./r1  , 1./r2  ])  
        
        # Gain
        self.ctrl_gains      = [kp,ki,kd,0,0,0]
        
        # sync controller
        self.g_sync_kp    = rospy.get_param("g_sync_kp",       1 )
        self.g_sync_kp_HS = rospy.get_param("g_sync_kp_HS",    1 )
        self.g_sync_ki    = rospy.get_param("g_sync_ki",       0 )
        self.g_sync_ki_HS = rospy.get_param("g_sync_ki_HS",    0 )
        self.g_1          = rospy.get_param("g_1",            -1 )
        self.g_2          = rospy.get_param("g_2",            20 )
        self.w_eps        = rospy.get_param("w_eps",          10 )
        
        # Max current
        self.max_current = np.array([ m1_max_current , m2_max_current ])

        
    ###########################################
    def controller( self ):
        """ """
        
        
        if ( self.k == 0 ):
            
            #####################
            # High speed mode
            ###################
            
            # Main loop
            self.setpoints        = [ self.f2setpoint( self.f , 1 ) , 0  ] # Direct force feedtrough in M1
            
            # Nullspace Loop
            self.delta_setpoints  = np.array([ self.g_1 , self.g_2 ]) * ( self.g_sync_kp_HS * (  self.w[1] - self.n * 1000 ) + self.g_sync_ki_HS * self.sync_err_i ) # Minimize M1 speed
            
            # Fusion of controllers
            self.setpoints        = self.setpoints + self.delta_setpoints.astype( int )
            
            # Operating modes
            self.ctrl_modes       = [  self.ctl_mode , self.ctl_mode ] # PWM mode for M!
            #self.brake_state      = 255 # Brake open
            self.brake_open()
            
            # Reset sync ctl
            if (self.g_sync_ki_HS == 0):
                self.sync_err_i       = 0 # intergral effect reset
            else:
                self.sync_err_i       = self.sync_err_i + (  self.w[1] - self.n * 1000 ) # intergral effect
            
            # Feedback data
            self.k_real           = 0 # High speed mode
            
            
            
        elif ( self.k ==1 ):
            
            # velocity smaller than epsilon
            sync_is_done = ( np.abs( self.w[1] ) < self.w_eps )
            
            #sync_is_done = True  # debug
            
            if sync_is_done:
                
                #########################
                # High force mode
                ########################
                
                # Main Loop
                self.setpoints        = [ 0 , self.f2setpoint( self.f , 2 )  ] # Direct M1 PWM
                
                # Operating modes
                self.ctrl_modes       = [ 0 , self.ctl_mode                  ] # M1 is oFF
                #self.brake_state      = 0 # Brake close
                self.brake_close()
                
                # Reset syn ctl
                self.sync_err_i       = 0 # intergral effect
                
                # Feedback data
                self.k_real           = 1 # High Force mode
                
                
            else:
                
                ########################
                # Sync controller
                #######################
                
                # Main Loop
                self.setpoints_HS     = [ self.f2setpoint( self.f , 1 ) , 0              ] # Direct M1 PWM
                
                # Nullspace sync controller : PI on w1 speed
                self.delta_setpoints  = np.array([ self.g_1 , self.g_2 ]) * ( self.g_sync_kp *  self.w[1] + self.g_sync_ki * self.sync_err_i )
                self.sync_err_i       = self.sync_err_i + self.w[1] # intergral effect
                
                # Fusion of controllers
                self.setpoints        = self.setpoints_HS + self.delta_setpoints.astype( int )
                
                # Operating modes
                self.ctrl_modes       = [      self.ctl_mode                  , self.ctl_mode  ] 
                #self.brake_state      = 255 # Brake open
                self.brake_open()
                
                # Feedback data
                self.k_real           = 0 # High speed mode
                
        
        # Satuation
        self.setpoints_comm = self.setpoints_saturation( self.setpoints )
        
        # Publish Motor cmd
        self.pub_cmd()
        
        
    ###########################################
    def brake_open( self ):
        """ set brake state on """
        
        if self.time_brake_opened < 1.0:
            self.brake_state        = 255 # Brake open
            self.time_brake_opened  = self.time_brake_opened + self.dt
            
        else:
            # Brake behaving weird when reducing pwm...
            # Not used
            self.brake_state      = 255  # Brake open
    
    ###########################################
    def brake_close( self ):
        """ set brake state off """
    
        self.brake_state       = 0 # Brake close
        self.time_brake_opened = 0
        
        
    ###########################################
    def f2setpoint( self , f = 0 , motor_id = 1 ):
        
        setpoint = int( self.f * self.torque_gains[ motor_id ] )
            
        return setpoint
        
        
    ###########################################
    def setpoints_saturation( self , setpoints ):
        
        if self.ctl_mode == 3:
            
            # Current mode
            
            setpoints_comm = [0,0]
        
            for i in range( len(setpoints) ):
                # Satuarations
                if setpoints[i] > self.max_current[i]:
                    setpoints[i]  = self.max_current[i]
                    
                elif (setpoints[i]  < -self.max_current[i]):
                    setpoints[i]  = -self.max_current[i]
                    
                else:
                    setpoints[i]  = setpoints[i] 
                
                # Units
                setpoints_comm[i] =  int( setpoints[i] * self.mA2units )
                    
            return setpoints_comm
            
        else:
            
            return setpoints
                
    
    
    ############################################
    def setpoint_callback( self , msg ):
        
        """ Record new setpoint """
        self.f = msg.f  # effort
        self.k = msg.k  # mode
        self.n = msg.n  # nullspace setpoint
        
        
    ############################################
    def feedback_callback_m1( self , msg ):
        """ """
        
        self.update_feedback( msg , 1 )
        
    
    ############################################
    def feedback_callback_m2( self , msg ):
        """ """
        
        self.update_feedback( msg , 2 )
        
        # For asynchrone ctl
        self.decode_feedback()
        self.controller()
        self.pub_feedback( msg )
        
        
    ############################################
    def update_feedback( self , msg , motor_ID ):
        """ """

        self.y_raw[ motor_ID ]      = msg.encoder * self.signs[ motor_ID ]
        
        
    ############################################
    def decode_feedback( self ):
        """ """
        
        # Time period
        t      = rospy.get_rostime()
        dt_ros = t - self.t_last
        dt     = dt_ros.to_sec()
        
        # Update digital filter values
        self.alpha = dt / ( self.filter_rc + dt )
        
        # Init position to first read encoders values
        if self.INIT:
            self.INIT   = False
            self.y_past = self.y_raw 
        
        # Compute filtered speed with raw large values
        self.w_raw       = self.y_raw - self.y_past                      # ticks per period
        w                = ( self.w_raw + 0.0 ) / dt                         # ticks per seconds
        
        if self.filter_active:
            w_filtered_ticks = self.alpha * w + ( 1 - self.alpha ) * self.w_x    # filter
        else:
            w_filtered_ticks = w
        
        ################
        # Kinematic
        ###############
        
        # Position 
        self.y    = self.y_raw  * self.corr                         # for m1 & m2
        self.y[0] = self.g[1] * self.y[1] + self.g[2] * self.y[2]   # output rev
        self.a    = self.signs[0] * self.corr[0] * self.g[0] * self.y[0]
        # Speed [rad/sec] 
        self.w    = w_filtered_ticks * self.corr                    # M1 & M2
        self.w[0] = self.g[1] * self.w[1] + self.g[2] * self.w[2]   # output shaft
        self.da   = self.signs[0] * self.corr[0] * self.g[0] * self.w[0]
        
        
        #Memory
        self.w_x    = w_filtered_ticks
        self.y_past = self.y_raw.copy()
        self.t_last = t
        self.dt     = dt
        
        
    ###########################################
    def pub_cmd( self ):
        """ """
        
        msg_m1 = inputs()
        msg_m2 = inputs()
        
        msg_m1.header.stamp    = rospy.Time.now()        
        msg_m1.ctrl_mode       = self.ctrl_modes[0]
        msg_m1.ctrl_gains      = self.ctrl_gains 
        msg_m1.ctrl_setpoint   = int( self.setpoints_comm[0] )
        msg_m1.trap_mode       = self.trap_mode
        msg_m1.trap_values     = self.trap_values
        
        msg_m2.header.stamp    = rospy.Time.now() 
        msg_m2.ctrl_mode       = self.ctrl_modes[1]
        msg_m2.ctrl_gains      = self.ctrl_gains 
        msg_m2.ctrl_setpoint   = int( self.setpoints_comm[1] )
        msg_m2.trap_mode       = self.trap_mode
        msg_m2.trap_values     = self.trap_values
        
        # Brake is wired to board ID#2
        if self.mbrake == 2:
            msg_m1.brake_pwm       = 0
            msg_m2.brake_pwm       = self.brake_state
        
        # Brake is wired to board ID#1
        elif self.mbrake == 1:
            msg_m1.brake_pwm       = self.brake_state
            msg_m2.brake_pwm       = 0
        
        self.pub_cmd_m1.publish( msg_m1 )
        self.pub_cmd_m2.publish( msg_m2 )
        
        
    ###########################################
    def pub_feedback( self , msg ):
        """ """
        
        msg_y = dsdm_actuator_sensor_feedback()
        
        # Copy Header
        msg_y.header.stamp     = msg.header.stamp 
        msg_y.header.frame_id  = msg.header.frame_id
        
        # Feedback info
        msg_y.theta  = self.y
        msg_y.w      = self.w
        msg_y.y_raw  = self.y_raw
        msg_y.w_raw  = self.w_raw
        msg_y.da     = self.da
        msg_y.a      = self.a
        msg_y.w1     = self.w[1]
        msg_y.w2     = self.w[2]
        msg_y.id1    = self.setpoints[0]
        msg_y.id2    = self.setpoints[1]
        msg_y.k_real = self.k_real
        
        self.pub_y.publish( msg_y )
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('ctl',anonymous=False)
    node = DSDM_CTL()
    rospy.spin()
