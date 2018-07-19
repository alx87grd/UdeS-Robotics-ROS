#!/usr/bin/env python

import math
import sys
from image_setup.msg import CamState
from std_msgs.msg import Int32MultiArray
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

QUEUE_LENGTH=10

class LaneDetector:

  ##########################################################

  def __init__(self):
    
    self.left_lines  = deque(maxlen=QUEUE_LENGTH)
    self.right_lines = deque(maxlen=QUEUE_LENGTH)
    
    self.image_pub = rospy.Publisher("ycam_msg", CamState, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("cam_img_raw",Image,self.callback)
  

  ##########################################################

  
  def callback(self,data):
    try:
      width = 0.07
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      #white_yellow = self.select_white_yellow(image)
      smooth_gray  = self.apply_smoothing(image)
      edges        = self.detect_edges(smooth_gray)
      regions      = self.select_region(edges)
      regions_ori  = self.select_region(image)
      lines, img_line       = self.hough_lines(regions)
      left_line, right_line = self.lane_lines(image, lines)
      left_mean_line        = self.mean_line(left_line,  self.left_lines)
      right_mean_line       = self.mean_line(right_line, self.right_lines)
      state_y      = self.lateral_deviation((left_mean_line, right_mean_line),image ,width)
      final_img    = self.draw_lines(image, (left_mean_line, right_mean_line), state_y)
      
      msg = CamState()
      msg.header.stamp = rospy.Time.now()
      msg.state_y = state_y
      self.image_pub.publish(msg)
      
              
    except CvBridgeError as e:
      print(e)
      
    #cv2.imshow("Filtered",white_yellow)
    cv2.imshow("GRAY", smooth_gray)
    cv2.imshow("Canny", edges)
    cv2.imshow("ROI", regions)
    cv2.imshow("ROI ori", regions_ori)
    cv2.imshow("Lines", img_line)
    cv2.imshow("Final", final_img)
  

    cv2.waitKey(3)
      
  ##########################################################

  def select_white_yellow(self, image):
    
    converted = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    # white color mask
    lower = np.uint8([  0, 235,   0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    # yellow color mask
    lower = np.uint8([ 10,   0, 100])
    upper = np.uint8([ 40, 255, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    return cv2.bitwise_and(image, image, mask = mask)


  ##########################################################

  def apply_smoothing(self, image, kernel_size=15):

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    """
    kernel_size must be postivie and odd
    """
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

  ##########################################################
  
  def detect_edges(self, image, low_threshold=50, high_threshold=150):
    return cv2.Canny(image, low_threshold, high_threshold)

 ##########################################################

  def filter_region(self, image, vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
    return cv2.bitwise_and(image, mask)

  ##########################################################

  def select_region(self, image):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
    """
    # first, define the polygon by vertices
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.00, rows*0.9]
    top_left     = [cols*0.2, rows*0.4]
    bottom_right = [cols*1.00, rows*0.9]
    top_right    = [cols*0.8, rows*0.4] 
    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return self.filter_region(image, vertices)

  ##########################################################
  
  def hough_lines(self, image):
    """
    `image` should be the output of a Canny transform.
    
    Returns hough lines (not the image with lines)
    """
    img_line = np.copy(image)
    clr_img_line = cv2.cvtColor(img_line, cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv2.line(clr_img_line, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)
            
    return lines, clr_img_line

 
  ##########################################################

  def average_slope_intercept(self, lines):
    left_lines    = [] # (slope, intercept)
    left_weights  = [] # (length,)
    right_lines   = [] # (slope, intercept)
    right_weights = [] # (length,)

    if lines is None:
      return None
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2==x1:
                continue # ignore a vertical line
            slope = (y2-y1)/(x2-x1)
            intercept = y1 - slope*x1
            length = np.sqrt((y2-y1)**2+(x2-x1)**2)
            if slope < 0: # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    
    # add more weight to longer lines    
    left_lane  = np.dot(left_weights,  left_lines) /np.sum(left_weights)  if len(left_weights) >0 else None
    right_lane = np.dot(right_weights, right_lines)/np.sum(right_weights) if len(right_weights)>0 else None
    
    return left_lane, right_lane

  ##########################################################

  def make_line_points(self, y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None
    
    slope, intercept = line
    
    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    
    return ((x1, y1), (x2, y2))


  ##########################################################
  
  def lane_lines(self, image, lines):
    
    if lines is None:
        return None
    else:
      
     left_lane, right_lane = self.average_slope_intercept(lines)
    
     y1 = image.shape[0] # bottom of the image
     y2 = y1*0.6         # slightly lower than the middle

     left_line  = self.make_line_points(y1, y2, left_lane)
     right_line = self.make_line_points(y1, y2, right_lane)
    
     return left_line, right_line


  ##########################################################

  def mean_line(self, line, lines):
     if line is not None:
        lines.append(line)

     if len(lines)>0:
        line = np.mean(lines, axis=0, dtype=np.int32)
        line = tuple(map(tuple, line)) # make sure it's tuples not numpy array for cv2.line to work
     return line

  ##########################################################

  def draw_lines(self, image, lines, state_y, color=[0, 0, 255], thickness=20):
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image,line[0], line[1],  color, thickness)
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (180,400)
            fontScale              = 1
            fontColor              = (0,0,255)
            lineType               = 2

            cv2.putText(line_image,"State y = %.5f" %state_y, bottomLeftCornerOfText,font,fontScale,fontColor,lineType)
        
            
    final_img = cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)

    return final_img
  
  ##########################################################

  def lateral_deviation(self, lines, image, width):
    
    if lines is None:
      return None
    
    else:
    
     lslope = (float(lines[0][1][1])-float(lines[0][0][1]))/(float(lines[0][1][0])-float(lines[0][0][0]))
     rslope = (float(lines[1][1][1])-float(lines[1][0][1]))/(float(lines[1][1][0])-float(lines[1][0][0]))
     lintercept = float(lines[0][1][1]) - lslope*float(lines[0][1][0])
     rintercept = float(lines[1][1][1]) - rslope*float(lines[1][1][0])
     x_inter = (rintercept-lintercept)/(lslope-rslope)
     y_inter = lslope*x_inter+lintercept
     dist_l = x_inter-lines[0][0][0]
     dist_r = lines[1][0][0]-x_inter
     state_y = ((width*dist_l)/(dist_l+dist_r))-width/2
  
     return state_y

  ##########################################################

def main(args):
  ic = LaneDetector()
  rospy.init_node('LINE_DETECTION', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
