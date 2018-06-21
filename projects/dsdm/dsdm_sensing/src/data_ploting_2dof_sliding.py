#!/usr/bin/env python

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


# Embed font type in PDF
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype']  = 42



#data = np.array([[ self.q , self.dq , self.q_d , self.f , self.k , t, self.w1 , self.w2 , self.i1 , self.i2 , self.brake ]])


## LOADING
path      = '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_sensing/data/' 

#name      = '2dof_test2'
#    
#i = 1 + 500 * 9
#j = i + 500 * 15


name      = '2dof_test3'
    
i = 1 + 500 * 0  
j = i + 500 * 7 #7

#i = 1 + 500 * 33
#j = i + 500 * 5
#
#name      = '2dof_test4'
#    
#i = 1 + 500 * 0  
#j = i + 500 * 17 #7


yaxis = True

#i = 1 + 500 * 3.95
#j = i + 500 * 4.8
DATA = np.load( path + name + '.npy' )

## DEOCODING
q1   = DATA[i:j,0] 
dq1  = DATA[i:j,1] 
q2   = DATA[i:j,2]
dq2  = DATA[i:j,3]
f1   = DATA[i:j,4]
k1   = DATA[i:j,5] 
f2   = DATA[i:j,6] 
k2   = DATA[i:j,7]  
t    = DATA[i:j,8]  - DATA[i,8]

q1d   = DATA[i:j,0] * 0.0 -3.3
q2d   = DATA[i:j,2] * 0.0  -2.0

save = False


## Figures

def plot_states( save ):

    fontsize = 7
    
    matplotlib.rc('xtick', labelsize=fontsize )
    matplotlib.rc('ytick', labelsize=fontsize )
    
    
    simfig , plot = plt.subplots(4, sharex=True,figsize=( 4 , 3),dpi=600, frameon=True)
    
    simfig.canvas.set_window_title('Closed loop trajectory')
    
    plot[0].plot( t ,  q1d , 'r--', label = 'Ref. trajectory')
    plot[0].plot( t ,  q1 , 'b' ,  label = 'Actual position')
    plot[0].set_yticks([-3.5,-3,-2.5])
    plot[0].set_ylim(-3.6,-2.4)
    plot[0].grid(True)
    #plot[0].set_ylim(-6,1)
    #legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
    #legend.get_frame().set_alpha(0.4)
    
    plot[1].plot( t ,  q2d , 'r--', label = 'Ref. trajectory')
    plot[1].plot( t ,  q2 , 'b')
    plot[1].set_yticks([-2,0,2])
    plot[1].set_ylim(-2.5,2.5)
    plot[1].grid(True)
    
    plot[2].plot( t ,  dq1 , 'b')
    plot[2].set_yticks([-1,0])
    plot[2].set_ylim(-2,0.5)
    plot[2].grid(True)
    
    plot[3].plot( t , dq2 , 'b')
    plot[3].set_yticks([-10,-5,0])
    plot[3].set_ylim(-11,1)
    plot[3].grid(True)
    
    a=2

    
    plot[0].set_ylabel('$q_1$  \n $[rad]$' , fontsize=(fontsize+a) )
    plot[2].set_ylabel('$\dot{q}_1$ \n $[rad/s]$' , fontsize=(fontsize+a) )
    plot[1].set_ylabel('$q_2$ \n $[rad]$' , fontsize=(fontsize+a) )
    plot[3].set_ylabel('$\dot{q}_2$ \n $[rad/s]$' , fontsize=(fontsize+a) )
    
    plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
    #plot[-1].set_xlim(0,5)
    #plot[-1].set_xticks([0,2,4])
    
    #plot[-1].set_xticks([0.65,1.16,1.39,2.63])
    #plot[-1].set_xticks([0,1,2,3,4])

    simfig.tight_layout()
    
    simfig.show()
    
    if save:
        simfig.savefig( path + name + '_x' +'.pdf' )
        simfig.savefig( path + name + '_x' + '.png' )
        
        
        
        
def plot_cmd( save ):

    fontsize = 7
    
    matplotlib.rc('xtick', labelsize=fontsize )
    matplotlib.rc('ytick', labelsize=fontsize )
    
    
    simfig , plot = plt.subplots(4, sharex=True,figsize=( 4 , 3),dpi=600, frameon=True)
    
    simfig.canvas.set_window_title('Closed loop cmd')
    
    plot[0].plot( t ,  f1 , 'b' ,  label = 'Actual position')
    plot[0].set_yticks([-0.08,0,0.08])
    plot[0].set_ylim(-0.12,0.12)
    plot[0].grid(True)
    #legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
    #legend.get_frame().set_alpha(0.4)
    
    plot[1].plot( t ,  f2 , 'b')
    plot[1].set_yticks([-0.08,0,0.08])
    plot[1].set_ylim(-0.12,0.12)
    plot[1].grid(True)
    
    plot[2].plot( t ,  k1 * 1153 + 72 , 'r')
    plot[2].set_yticks([72,1225])
    plot[2].set_ylim(-100,1500)
    plot[2].grid(True)

    plot[3].plot( t ,  k2 * 451 + 23 , 'b')
    plot[3].set_yticks([23,474])
    plot[3].set_ylim(-100,600)
    plot[3].grid(True)
    
    a=2
    
    plot[0].set_ylabel('$\\tau_1$ \n $[Nm]$' , fontsize=(fontsize+a) )
    plot[1].set_ylabel('$\\tau_2$ \n $[Nm]$' , fontsize=(fontsize+a) )
    plot[2].set_ylabel('$R_{11}$' , fontsize=(fontsize+a) )
    plot[3].set_ylabel('$R_{22}$' , fontsize=(fontsize+a) )
    
    plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
    #plot[-1].set_xlim(0,5)
    #plot[-1].set_xticks([0,2,4])
    
    #plot[-1].set_xticks([0.65,1.16,1.39,2.63])
    #plot[-1].set_xticks([0,1,2,3,4])

    simfig.tight_layout()
    
    simfig.show()
    
    if save:
        simfig.savefig( path + name + '_u' + '.pdf' )
        simfig.savefig( path + name + '_u' + '.png' )
        


    
plot_states( True)
plot_cmd( True )


