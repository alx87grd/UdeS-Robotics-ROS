#!/usr/bin/env python

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


# Embed font type in PDF
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype']  = 42



#data = np.array([[ self.q , self.dq , self.q_d , self.f , self.k , t, self.w1 , self.w2 , self.i1 , self.i2 , self.brake ]])

#########################################
class plotter(object):
    
    def __init__(self):
        
        self.save = False
        self.t0   = 0
        self.q0   = 0
    

    def loaddata( self, name , i , j , zero_position = True ):
    
        fullname      = '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_sensing/data/' + name + '.npy'
        #fullname      = name + '.npy'
        
        DATA = np.load( fullname )
        
        ## DEOCODING
        if zero_position:
            self.q  = DATA[i:j,0] - DATA[i,0]
            self.q0 = DATA[i,0]
        else:
            self.q  = DATA[i:j,0] - self.q0
            
        self.dq = DATA[i:j,1]
        self.qd = DATA[i:j,2]
        self.f  = DATA[i:j,3]
        self.k  = DATA[i:j,4]
        self.t  = DATA[i:j,5] - DATA[i,5] 
        self.w1 = DATA[i:j,6] 
        self.w2 = DATA[i:j,7]  
        self.i1 = DATA[i:j,8] 
        self.i2 = DATA[i:j,9]
        self.b  = DATA[i:j,10] 
        self.id1= DATA[i:j,11] 
        self.id2= DATA[i:j,12]
        self.wr1= DATA[i:j,13] 
        self.wr2= DATA[i:j,14]
        
        
    ## Figures
    
    def plot_position_tracking( self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        plot[0].plot( self.t ,  self.qd , 'r--', label = 'Ref.')
        plot[0].plot( self.t ,  self.q , 'b' ,  label = 'Actual')
        plot[0].set_yticks([0, 3.14])
        plot[0].set_ylim(-2,5)
        legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  self.dq , 'b')
        plot[1].set_yticks([-20,-10,0,10,20])
        plot[1].set_ylim(-25,25)
        plot[1].grid(True)
        
        plot[2].plot( self.t ,  self.f , 'r')
        plot[2].set_yticks([-0.2,0,0.2])
        plot[2].set_ylim(-0.3,0.3)
        plot[2].grid(True)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'Ref.')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'Actual')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[3].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Angle \n [rad]' , fontsize=fontsize )
        plot[1].set_ylabel('Speed \n [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Torque \n [Nm]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        #plot[-1].set_xlim(0,4)
        #plot[-1].set_xticks([0,1,2,3,4])
    
    
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
        
    def plot_all_OL(self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        #plot[0].plot( t ,  qd , 'r--', label = 'Ref. trajectory')
        plot[0].plot( self.t ,  self.q , 'b' ,  label = 'Actual')
        plot[0].set_yticks([0,100])
        plot[0].set_ylim(-30,170)
        plot[0].grid(True)
        #legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        #legend.get_frame().set_alpha(0.4)
        
        plot[1].plot( self.t ,  self.dq , 'b')
        plot[1].set_yticks([0,10,20])
        plot[1].set_ylim(-5,25)
        plot[1].grid(True)
        
        plot[2].plot( self.t ,  self.f , 'b')
        plot[2].set_yticks([-0.2,0,0.2])
        plot[2].set_ylim(-0.3,0.3)
        plot[2].grid(True)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'Ref.')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'Actual')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[3].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Angle \n [rad]' , fontsize=fontsize )
        plot[1].set_ylabel('Speed \n [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Torque \n [Nm]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        #plot[-1].set_xlim(0,4)
        #plot[-1].set_xticks([0,1,2,3,4])
    
    
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
    
    def plot_motors(self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(1, sharex=True,figsize=(3, 2),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        plot.plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot.plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot.grid(True)
        legend = plot.legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot.set_ylabel('Velocity [rad/sec]' , fontsize=fontsize )
        
        plot.set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
        
    def plot_shift(self , filename , LS = False ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(3, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        if LS:
            plot[0].set_yticks([0,1.0])
            plot[0].set_ylim(-0.6,1.6)
        
        plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        legend = plot[1].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[2].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'Ref.')
        plot[2].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'Actual')
        plot[2].set_yticks([23,474])
        plot[2].set_ylim(-100,600)
        plot[2].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Output [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('Motors [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
        
    def plot_shift_zoom(self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual')
        plot[0].set_yticks([0,1.0])
        plot[0].set_ylim(-0.6,1.6)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.wr1 , 'b' , label = 'M1')
        #plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        plot[1].set_yticks([-5,0,5])
        plot[1].set_ylim(-6,6)
        
        #plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[2].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[2].grid(True)
        plot[2].set_yticks([0,500])
        plot[2].set_ylim(-50,550)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'Ref.')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'Actual')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Output \n [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('M1 \n [ticks/dt]' , fontsize=fontsize )
        plot[2].set_ylabel('M2 \n [rad/sec]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
        
        
    def plot_speed_tracking(self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(3, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual')
        plot[0].plot( self.t ,  self.qd , 'r--', label = 'Ref.')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        legend = plot[1].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[2].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'Ref.')
        plot[2].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'Actual')
        plot[2].set_yticks([23,474])
        plot[2].set_ylim(-100,600)
        plot[2].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Outputs [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('Motors [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
        
    def plot_nullspace(self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(3, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual')
        plot[0].plot( self.t ,  self.qd , 'r--', label = 'Ref.')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        #plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        
        #plot[2].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[2].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[2].grid(True)
        
        plot[0].set_ylabel('Output [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('M1 [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('M2 [rad/sec]' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        
        
        
    def plot_contact(self , filename ):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title( filename )
        
        #plot[0].plot( t ,  qd , 'r--', label = 'Ref. trajectory')
        plot[0].plot( self.t ,  self.q , 'b' ,  label = 'Actual')
        #plot[0].set_yticks([0,100])
        #plot[0].set_ylim(-30,170)
        plot[0].grid(True)
        #legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        #legend.get_frame().set_alpha(0.4)
        
        plot[1].plot( self.t ,  self.dq , 'b')
        #plot[1].set_yticks([0,10,20])
        #plot[1].set_ylim(-5,25)
        plot[1].grid(True)
        
        plot[2].plot( self.t ,  -self.wr1 , 'b' , label = 'M1')
        plot[2].plot( self.t ,  -self.wr2 , 'r' , label = 'M2')
        plot[2].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'Ref.')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'Actual')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[3].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Angle \n [rad]' , fontsize=fontsize )
        plot[1].set_ylabel('Output \n [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Motors \n [rad/sec]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        #plot[-1].set_xlim(0,4)
        #plot[-1].set_xticks([0,1,2,3,4])
    
    
        simfig.tight_layout()
        
        if self.save:
            
            simfig.savefig( filename + '.pdf' )
            simfig.savefig( filename + '.png' )
        
        simfig.show()
        


# Main

pltr = plotter()

pltr.save = False
    
##############################################
###      Manual Experiment ###################
##############################################
    
#name      = 'dsdm_a2_manual_no1'
#i = 1 + 500 * 30
##j = i + 500 * 
#j = 1 + 500 * 42
#
#pltr.loaddata( name , i , j)
#pltr.plot_all_OL()
#pltr.plot_motors()

# nice global data
name      = 'dsdm_a2_manual_no2'
i = 1 + 500 * 1
#j = i + 500 * 
j = 1 + 500 * 40

pltr.loaddata( name , i , j)
pltr.plot_all_OL( 'dsdm_manual_all'    )
pltr.plot_motors( 'dsdm_manual_motors' )

#ZOOM on shifts
name      = 'dsdm_a2_manual_no2'
i = 1 + 500 * 23
#j = i + 500 * 
j = 1 + 500 * 32

pltr.loaddata( name , i , j)
pltr.plot_shift(  'dsdm_manual_shift' )

##############################################
###   Speed Tracking Experiment ##############
##############################################

name      = 'dsdm_a2_speed_no1'
i = 1 + 500 * 5
#j = i + 500 * 
j = 1 + 500 * 21

pltr.loaddata( name , i , j)
pltr.plot_speed_tracking( 'dsdm_speed_tracking' )

##############################################
###  Position Tracking Experiment ############
##############################################

name      = 'dsdm_a2_position_no3'

# HF

i = 1 + 500 * 40
#j = i + 500 * 
j = 1 + 500 * 80

pltr.loaddata( name , i , j)
pltr.plot_position_tracking(  'dsdm_position_tracking' )

## HS
#
#i = 1 + 500 * 67
##j = i + 500 * 
#j = 1 + 500 * 71
#
#pltr.loaddata( name , i , j)
#pltr.plot_position_tracking()

##############################################
###  Nullspace  ############
##############################################

name      = 'dsdm_a2_nullspace_no1'

i = 1 + 500 * 1
#j = i + 500 * 
j = 1 + 500 * 60

pltr.loaddata( name , i , j)
pltr.plot_nullspace( 'dsdm_nullspace' )


##############################################
###  Cst speed shifts  ############
##############################################

# Wihtout preparation

name      = 'dsdm_a2_cstspd_shifts_no1'

i = 1 + 500 * 4.5
#j = i + 500 * 
j = 1 + 500 * 14.5

pltr.loaddata( name , i , j)
pltr.plot_shift( 'dsdm_cstspd_shifts' , True )


# With preparation


name      = 'dsdm_a2_cstspd_shifts_no2'

i = 1 + 500 * 1
#j = i + 500 * 
j = 1 + 500 * 11

pltr.loaddata( name , i , j)
pltr.plot_shift( 'dsdm_cstspd_shifts_withprep' , True )


# With preparation zoom

i = 1 + 500 * 4.6
#j = i + 500 *4
j = 1 + 500 * 4.9

pltr.loaddata( name , i , j)
pltr.plot_shift_zoom( 'dsdm_cstspd_shifts_withprep_zoom' )


##############################################
###  Compliant Contact            ############
##############################################


#name      = 'dsdm_a2_compliant_no3'
#
#i = 1 + 500 * 1.5
##j = i + 500 * 
#j = 1 + 500 * 2.5
#
#pltr.loaddata( name , i , j )
#pltr.plot_contact( 'dsdm_compliant_HS' )
#
#
#i = 1 + 500 * 15.5
##j = i + 500 * 
#j = 1 + 500 * 16.5
#
#pltr.loaddata( name , i , j, False )
#pltr.plot_contact( 'dsdm_compliant_down_shift' )


name      = 'dsdm_a2_compliant_no5'


#i = 1 + 500 * 1
##j = i + 500 * 
#j = 1 + 500 * 60
#
#pltr.loaddata( name , i , j )
#pltr.plot_contact( 'dsdm_compliant' )


i = 1 + 500 * 7.1
#j = i + 500 * 
j = 1 + 500 * 7.6

pltr.loaddata( name , i , j  )
pltr.plot_contact( 'dsdm_compliant_HS' )


i = 1 + 500 * 11
#j = i + 500 * 
j = 1 + 500 * 13

pltr.loaddata( name , i , j , False )
pltr.plot_contact( 'dsdm_compliant_HF' )

i = 1 + 500 * 24
#j = i + 500 * 
j = 1 + 500 * 24.5

pltr.loaddata( name , i , j , False )
pltr.plot_contact( 'dsdm_compliant_down' )



##############################################
###  Stiff Contact                ############
##############################################

#name      = 'dsdm_a2_stiff_no1'

#i = 1 + 500 * 1
##j = i + 500 * 
#j = 1 + 500 * 20
#
#pltr.loaddata( name , i , j )
#pltr.plot_contact( 'dsdm_stiff_all' )


#i = 1 + 500 * 18
##j = i + 500 * 
#j = 1 + 500 * 19
#
#pltr.loaddata( name , i , j  )
#pltr.plot_contact( 'dsdm_stiff_down' )


# TO kept Nice

name      = 'dsdm_a2_stiff_no5'

i = 1 + 500 * 1.5
#j = i + 500 * 
j = 1 + 500 * 2

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_stiff_down_latency' )


name      = 'dsdm_a2_stiff_no6'

i = 1 + 500 * 31
#j = i + 500 * 
j = 1 + 500 * 31.5

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_stiff_down_nice' )


i = 1 + 500 * 21
#j = i + 500 * 
j = 1 + 500 * 22

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_stiff_down_some_negative' )



name      = 'dsdm_a2_stiff_no7'

i = 1 + 500 * 45.4
#j = i + 500 * 
j = 1 + 500 * 46

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_stiff_auto' )


name      = 'dsdm_a2_stiff_no8'

i = 1 + 500 * 14.5
#j = i + 500 * 
j = 1 + 500 * 15

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_stiff_auto_new' )


name      = 'dsdm_a2_stiff_no9'

i = 1 + 500 * 1.6
#j = i + 500 * 
j = 1 + 500 * 2.1

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_stiff_auto_nullspace_wrong' )


##############################################
###  Ballon                ############
##############################################



name      = 'dsdm_a2_ballon_no4'


i = 1 + 500 * 51
#j = i + 500 * 
j = 1 + 500 * 55

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_ballon_static' )


#i = 1 + 500 * 29
##j = i + 500 * 
#j = 1 + 500 * 31
#
#pltr.loaddata( name , i , j )
#pltr.plot_contact( 'dsdm_ballon_HS' )


i = 1 + 500 * 68.6
#j = i + 500 * 
j = 1 + 500 * 68.9

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_ballon_down_shift_nice' )


name      = 'dsdm_a2_ballon_no6'

i = 1 + 500 * 27.7
#j = i + 500 * 
j = 1 + 500 * 28.7

pltr.loaddata( name , i , j )
pltr.plot_contact( 'dsdm_ballon_down_shift_nice2' )










