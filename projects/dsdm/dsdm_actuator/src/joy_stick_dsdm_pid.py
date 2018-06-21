#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg    import Float64MultiArray
from sensor_msgs.msg import Joy
from dsdm_msgs.msg   import dsdm_actuator_control_inputs, dsdm_actuator_sensor_feedback, ctl_error

#########################################
class dsdm_pid(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = False
        
        # Publishers
        self.pub_u              = rospy.Publisher("a2/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_e              = rospy.Publisher("pid_error", ctl_error               , queue_size=1  )
        
        # Suscribers
        self.sub_joy            = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        #self.sub_state_feedback = rospy.Subscriber("x_hat", Float64MultiArray , self.update_state ,  queue_size=1      )
        self.sub_a2         = rospy.Subscriber("a2/y", dsdm_actuator_sensor_feedback , self.feedback_callback , queue_size=1 )
        
        
        # Init DSDM msg
        self.f  = 0
        self.k  = 1 # High force mode
        
        # Init sensor feedback
        self.x      = np.zeros(2)
        self.a      = self.x[0]
        self.da     = self.x[1]
        self.k_real = 1
        
        # Init joy feedback
        self.joy_msg         = Joy()
        self.joy_msg.axes    = [ 0 , 0 , 0 , 0 , 0 ]
        self.joy_msg.buttons = [ 0 , 0 , 0 , 0 , 0 ]
        
        # Init ctl_error msg
        self.e               = 0
        self.de              = 0
        self.set_point       = 0
        self.cmd             = 0
        self.actual          = 0
        self.n               = 0
        self.n_d             = 0  # Nullspace setpoint
        
        # PID
        self.enable    = False
        self.mode      = 'PWM'
        self.autoshift = True
        self.impact    = False
        #self.mode      = 'PID_position'
        #self.mode      = 'PID_speed'
        self.gain_HS_P = np.array([ 0.6 , 0.6  , 0.05 ])
        self.gain_HF_P = np.array([ 0.4 , 0.05 , 0.0 ])
        self.gain_HS_S = np.array([ 0.1 , 0.1  , 0.0 ])
        self.gain_HF_S = np.array([ 0.2 , 0.1  , 0.0 ])
        self.e_sum     = 0
        self.e_sat     = 3
        self.e_sum_r   = 3    # ratio of equivalence of integral action between modes
        self.dt        = 0.02  # assuming 500 HZ
        
        # Init hysteresis
        self.last_mode = self.mode
        self.last_k    = 1
        
        self.dda       = 0
        self.dda_state = 0
        self.da_last   = 0
        
    
    #######################################   
    def callback( self, msg ):
        """ """
        
        if self.enable:
            
            ##########################################################
            # Equivalence of integral action between modes
            # Gear changed
            if not( self.last_k == self.k_real ):
                # HF --> HS
                if (self.k_real == 0 ):
                    self.e_sum = self.e_sum * self.e_sum_r + 0.2
                    
                else:
                    self.e_sum = self.e_sum * ( 1.0 / self.e_sum_r ) 
                    
            self.last_k    = self.k_real
            
            # Saturation
            if (self.e_sum < -self.e_sat) :
                self.e_sum = -self.e_sat
            elif (self.e_sum > self.e_sat) :
                self.e_sum = self.e_sat
            
            ###############
            if self.autoshift:
                
                # postion-based
                """
                if ( self.a > 1.0 ):
                    self.k = 1
                else:
                    self.k = 0
                """
                
                #print(self.dda)
                
                if (self.dda > 15 ):
                    self.impact = True
                    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    
                if self.impact:
                    self.k = 1
                else:
                    self.k = 0
                    
            else:
                #reset memory
                self.impact = False
                    
            
            ######################################################
            
            # OPENLOOP
            if self.mode == 'PWM' :
                
                # Pick set_point with joysticks gain
                self.f     =    self.joy_msg.axes[1] * 0.2

                #FOr debug
                self.cmd = self.f
            
                        
            
            # PID Position
            elif self.mode == 'PID_position' :
                
                 #  Pick gains
                if ( self.k_real == 0):
                    kp = self.gain_HS_P[0]
                    ki = self.gain_HS_P[1]
                    kd = self.gain_HS_P[2]
                else:
                    kp = self.gain_HF_P[0]
                    ki = self.gain_HF_P[1]
                    kd = self.gain_HF_P[2]
                    
                # Torque
                
                e  = self.set_point - self.a
                de = 0              - self.da
                
                self.e_sum = self.e_sum + e * self.dt
                
                cmd = kp * e + kd * de + ki * self.e_sum
                
                self.f = cmd * 0.2
                
                # For debug
                self.actual = self.a
                self.e      = e
                self.de     = de
                self.cmd    = self.f
                
                
                    
            # PID Position
            elif self.mode == 'PID_speed' :
                
                #  Pick gains
                if ( self.k_real == 0 ):
                    kp = self.gain_HS_S[0]
                    ki = self.gain_HS_S[1]
                    kd = self.gain_HS_S[2]
                else:
                    kp = self.gain_HF_S[0]
                    ki = self.gain_HF_S[1]
                    kd = self.gain_HF_S[2]
                    
                # Torque
                
                e  = self.set_point - self.da
                de = 0
                
                self.e_sum = self.e_sum + e * self.dt
                
                cmd = kp * e + kd * de + ki * self.e_sum
                
                self.f = cmd * 0.2
                
                # For debug
                self.actual = self.da
                self.e      = e
                self.de     = de
                self.cmd    = self.f
                
                    
            # Nullspace Target
            self.n = self.n_d

        
        else:
            self.f   = 0
            self.k   = 1
            self.n_d = 0
        
        
        self.pub_u_msg()
        
        
    #######################################   
    def joy_callback( self, msg ):
        """ Log joy msg """
        
        self.joy_msg = msg
        
        self.set_point           = msg.axes[1] * np.pi
        
        self.n_d                 = msg.axes[4] 
        
        # Pick mode with trigger
        if ( self.joy_msg.axes[5] < 0):
            self.k = 0
        else:
            self.k = 1
        
        # Pick ctrl_mode with button state
        if ( msg.buttons[0] == 1 ):
            self.enable = True
        else:
            self.enable = False
            self.e_sum  = 0 # Reset integral error
            
        #Automatic gear-shift
        if ( msg.buttons[5] == 1 ):
            self.autoshift = True
        else:
            self.autoshift = False
            
        # Pick ctrl_mode with button state
        self.mode = 'PWM' #    Default
            
        if ( msg.buttons[1] == 1 ):
            self.mode   = 'PID_position'
            
            if ( msg.buttons[3] == 1 ):
                # Auto setpoint for posiiton
                self.set_point = 3.14
            
            
        elif ( msg.buttons[2] == 1 ):
            self.mode   = 'PID_speed'
            
            if ( msg.buttons[3] == 1 ):
                # Auto setpoint for speed
                self.set_point   = 1.0
            
        else:
            self.mode = 'PWM'
            self.e_sum  = 0 # Reset integral error
            
        # Reset Integral Error if mode changed
        if not( self.last_mode == self.mode ):
            
            print('Control Mode Updated to: ' + self.mode)
            self.e_sum = 0
            
        self.last_mode = self.mode
        
            
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg = dsdm_actuator_control_inputs()
        
        msg.f = self.f
        msg.k = self.k
        msg.n = self.n
        
        self.pub_u.publish( msg )
        
        # Publish error data
        msg2              = ctl_error()
        msg2.header.stamp = rospy.Time.now()
        msg2.set_point    = self.set_point
        msg2.actual       = self.actual
        msg2.e            = self.e
        msg2.de           = self.de
        msg2.e_int        = self.e_sum
        msg2.cmd          = self.cmd
        
        
        # temp hack
        msg2.cmd          = self.dda
        
        self.pub_e.publish( msg2 )
        
        if self.verbose:
            print('Error:', self.e )
            
            
            
    #######################################
    def feedback_callback( self , msg ):
        """ actuator feedback """
        
        self.a       = msg.a
        self.da      = msg.da
        self.k_real  = msg.k_real
        
        ## Acc estimation
        
        #Params
        dt           = 0.02
        a            = 0.02
        b            = ( 1 - a)
        
        # New measurement
        new_point    = ( self.da - self.da_last + 0.0 ) / dt
        
        #Filter
        self.dda     = a * new_point + b * self.dda_state
        
        # Memory
        self.dda_state = self.dda
        self.da_last   = self.da
        
        
        
        self.callback( None )
        


#########################################
if __name__ == '__main__':
    
    rospy.init_node('dsdm_pid',anonymous=False)
    node = dsdm_pid()
    rospy.spin()
