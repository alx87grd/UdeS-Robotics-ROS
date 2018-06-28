#!/usr/bin/env python


# ROS interface
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Python libs
import cv2
import numpy as np
import time


###########################################
class camreader(object):
    """ """
    def __init__(self):    
        """ """
        
        # ROS Stuff
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        # Publishers
        self.pub_img_raw = rospy.Publisher("cam_img_raw", Image, queue_size=1)
        
        # Timers
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.load_params )       
        
        # Objects
        self.bridge = CvBridge()
        
        # Memory
        self.camera_IMG     = None
        self.camera_msg    = None
        self.processed_img =None
        
        # First run
        self.load_params( None )
        
        # Open Camera
        self.cap = cv2.VideoCapture(0)
        
        
        #######################
        # Call back mode
        #######################
        
        max_rate = False
        
        
        if max_rate:
            while True:
                # Read Cam at maximum rate
                self.process_frame()
            
        else:
            # Set period for cam reading
            self.cam_timer   = rospy.Timer(rospy.Duration.from_sec( self.cam_period ), self.read_frame ) 
        

    #############################################
    def load_params(self, event):
        """ """
        
        # Timer period
        self.cam_period     = rospy.get_param("~cam_period", 0.04 )
        
        # Cropping Factors
        self.h_crop   = rospy.get_param("~h_crop", 0)
        self.v_crop   = rospy.get_param("~v_crop", 0)
        self.v_off    = rospy.get_param("~v_off",  0)
        self.h_crop   = rospy.get_param("~h_crop", 0)
        self.v_crop   = rospy.get_param("~v_crop", 0)
        self.v_off    = rospy.get_param("~v_off",  0)
        
        # Downsampling factor
        self.x_down   = rospy.get_param("~x_down", 1)
        
        # Output to terminal
        rospy.loginfo("[%s] Parameters Loaded " %(self.node_name))
        


    #############################################
    def read_frame(self, event ):
        """ """
        
        
        ret, frame       = self.cap.read()
        self.camera_IMG  = frame
        
        self.process_frame()
        self.publish_frame()
        
        
    ###############################################            
    def process_frame(self):
        """ Pre-pros image and publish """
    
        if not self.camera_IMG == None:
        
            # Crop
            h,w  = self.camera_IMG.shape[:2]
            print h,w
            a = self.v_crop # up/down edge crop
            c = self.v_off # horizontal offset
            b = self.h_crop  # Side crop
            
            crop_img = self.camera_IMG[0+a+c:h-a+c, 0+b:w-b]
            
            # Downsample
            if self.x_down == 1:
                """ No down sampling """
                processed_img = crop_img
                
            else:
                h,w           = crop_img.shape[:2]
                dstsize       = ( int( w / self.fast_x_down ) , int( h 
                / self.fast_x_down ) )
                processed_img = cv2.pyrDown( crop_img , dstsize )
                
            self.processed_img = processed_img
                
            rospy.loginfo("[%s] : Frame processed " %(self.node_name))
            
        else:
            
            rospy.loginfo("[%s] : No frame to process " %(self.node_name))
                

    #############################################
    def publish_frame(self):
        """ """
        
        if not self.processed_img == None:
            
             # Publish Message
            img_msg = self.bridge.cv2_to_imgmsg( self.processed_img , "bgr8")
            #img_msg.header.stamp = self.camera_msg.header.stamp
            #img_msg.header.frame_id = self.camera_msg.header.frame_id
            
            
            self.pub_img_raw.publish(img_msg)
            
            rospy.loginfo("[%s] Camera Frame Published " %(self.node_name))
        
        else:
            
            rospy.loginfo("[%s] : No frame to publish " %(self.node_name))






############################################################
if __name__ == '__main__': 
    rospy.init_node('camreader',anonymous=False)
    node = camreader()
    rospy.spin()
