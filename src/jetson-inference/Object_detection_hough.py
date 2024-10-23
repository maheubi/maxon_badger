#!/usr/bin/env python3
# Import ROS libraries and messages 
import rospy
import sys
import os 
import torch # type: ignore
import cProfile
import timeit
import datetime
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO  # type: ignore
import tensorrt as trt # type: ignore
import numpy as np
from geometry_msgs.msg import PointStamped
import message_filters
import math
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


model = YOLO('yolov8n-seg.pt')

class ObjectDetector:
  def __init__(self, model):    
    self.bridge = CvBridge()
    self.model = model

   # Subscribe to the image and depth topics
    self.image_sub = rospy.Subscriber("/zed/zed_nodelet/left/image_rect_gray"\
                                      ,Image, self.image_callback, queue_size=1, buff_size=2**24,tcp_nodelay=False) 
    self.depth_sub = rospy.Subscriber("/zed/zed_nodelet/depth/depth_registered"\
                                      ,Image, self.depth_callback, queue_size=1, buff_size=2**24,tcp_nodelay=False)
    self.info_sub = rospy.Subscriber("/zed/zed_nodelet/left/camera_info"\
                                      ,CameraInfo, self.info_callback, queue_size=1)
    # Publisher for the plotted image
    self.image_pub = rospy.Publisher("image_plotted", Image, queue_size=1)
    self.depth_pub = rospy.Publisher("depth_plotted", Image, queue_size=1)
    self.min_distance_pub = rospy.Publisher("min_distance", PointStamped, queue_size=1)
    self.marker_pub = rospy.Publisher("line_marker", Marker, queue_size=5)

  def image_callback(self,data):  
    start_time = time.time()

    try:

      #convert image to numpy
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = cv2.resize(cv_image, (640,384))
      #perform model inference
      results = self.model(cv_image, device ='0' , imgsz=640, conf=0.5, half=True)
       

      if results[0].masks  is not None:
        #plot images with detections
        image_plotted = results[0].plot()

      
        #Perform depth calculation for depth_image
        self.calculate_depth(image_plotted,cv_image,results)
      #else: 
        #image_plotted=cv_image

      #ros_image = self.bridge.cv2_to_imgmsg(image_plotted, "bgr8")
      #execution_time = time.time() - start_time
      #remaining_time = max(0, 0.145 - execution_time)
      #time.sleep(remaining_time)
      #self.image_pub.publish(ros_image)
      return cv_image
    except CvBridgeError as e:
      print(e)
    
    end_time = time.time()
    loop_time = end_time - start_time
    print("One loop of hough took {:.2f} seconds.".format(loop_time))
  
  def info_callback(self,data):
    
    self.P = data.P


  def depth_callback(self, data):

    start_time = time.time()
    try:
      self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")  
    except CvBridgeError as e:
        print(e)

    end_time = time.time()
    elapsed_time = end_time - start_time
    # print("Depth callback took {:.2f} seconds.".format(elapsed_time))

  def mean_depth(self, image, cv_image, x1, y1, x2, y2):
    h, w = image.shape
    results = []
    
    # List to handle multiple points
    points = [(x1, y1), (x2, y2)]

    for (x, y) in points:
        if x > 8 and x < w-8 and y > 8 and y < h-9:
            # Define top-left and bottom-right coordinates for the rectangle
            top_left = (x - 8, y - 8)
            bottom_right = (x + 9, y + 9)

            # Draw the rectangle on the depth image and the visual image
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
            cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)

            # Extract the 11x11 patch around (x, y)
            patch = image[y-8:y+9, x-8:x+9]
            print("Patch around ({}, {}):".format(x, y), patch)

            # Exclude 0, infinite, and NaN values
            valid_depths = patch[np.where((patch > 0) & (np.isfinite(patch)))]
            
            # Calculate the mean depth if there are any valid depths remaining
            if valid_depths.size > 0:
                mean_val = np.mean(valid_depths)
                results.append(mean_val)
            else:
                results.append(np.nan)
        else:
            results.append(np.nan)

    # Display the images with patches
    cv2.imshow("Depth Image with Patches", image)
    cv2.imshow("CV Image with Patches", cv_image)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()

    return results

  
  def calculate_depth(self, image_plotted,cv_image, results):
    start_time = time.time()
     
    #convert image to numpy
    masks = results[0].masks.data
    boxes = results[0].boxes.data
    clss = boxes[:,5]
    people_indices = torch.where(clss == 0)
    #scale for visualizing results
    people_masks = masks[people_indices]
    
    people_mask = torch.any(people_masks, dim=0).int() * 255
    people_mask_uint8 = people_mask.cpu().numpy().astype(np.uint8)
    kernel_size = 40  # Determines how much you want to erode the mask
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

# Erode the mask to shrink it
    people_mask_uint8 = cv2.erode(people_mask_uint8, kernel, iterations=1)

    min_depth = float('inf') 
    f_x = self.P[0]
    f_y = self.P[5]
    c_x = self.P[2]
    c_y = self.P[6]
    T_x = self.P[7]
    T_y = self.P[11]
    print("Matrix", self.P)

  
    #cv2.imshow("masks", people_mask_uint8)
    #cv2.waitKey(1)
    #cv2.destroyAllWindows()
    if people_mask is None:
      #end_time = time.time()
      #elapsed_time = end_time - start_time
      print("Calculate depth function took {:.2f} seconds.".format(elapsed_time))
      return 0
      
    if people_mask is not None:
# Combine segmentation masks into a single mask


      # Apply binary mask to the original image
      print("type people_mask",type(people_mask_uint8))
      print("cv_image", cv_image.shape)
      print("mask shape" , people_mask_uint8.shape)
      masked_image = cv2.bitwise_and(cv_image, cv_image, mask=people_mask_uint8)

      # Apply Canny edge detection
      gray_masked_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
      edges = cv2.Canny(gray_masked_image, 1, 20)

      # Hough Line Transformation
      lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, minLineLength=10, maxLineGap=200)

      if lines is not None:
          # Find the longest line
          max_length = 0
          longest_line = None
          for line in lines:
              x1, y1, x2, y2 = line[0]
              length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
              if length > max_length:
                  max_length = length
                  longest_line = (x1, y1, x2, y2)

          # Draw the longest line on the image
          x1, y1, x2, y2 = longest_line
          image = cv2.resize(self.depth_image, (640,384))

          #Z1 = image[y1,x1]
          #Z2 = image[y2,x2]
          # Calculate the mean depths around each point
          results = self.mean_depth(image,cv_image, x1, y1,x2,y2)
          Z1, Z2 = results

          print("Mean depth around (x1, y1):", Z1)
          print("Mean depth around (x2, y2):", Z2)
          
          if np.isfinite(Z1) and ~np.isnan(Z1) and np.isfinite(Z2) and ~np.isnan(Z2):
            print("Z1 equals", Z1, "Z2 equals", Z2)
            X1 = ((x1-c_x)*Z1)/f_x 
            Y1 = ((y1-c_y)*Z1)/f_y
            X2 = ((x2-c_x)*Z2)/f_x 
            Y2 = ((y2-c_y)*Z2)/f_y
            print("wolrd coodrinates = ", X1,Y1)
            self.publish_line_marker(X1,Y1,X2,Y2,Z1,Z2)
          else:
             print("Z1 is NAN")
          
          cv2.line(image_plotted, (x1, y1), (x2, y2), (255, 0, 0), 2)

          cv2.circle(image_plotted, (x1, y1), radius=5, color=(0, 255, 0), thickness=-1)
          cv2.circle(image_plotted, (x2, y2), radius=5, color=(0, 255, 0), thickness=-1)

          # Display the image with the longest line
          #cv2.imshow("Image with Longest Line and Start Point",  image_plotted)
          #cv2.waitKey(100)
          #cv2.destroyAllWindows()
          image = np.copy(image_plotted)  
          ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
          self.image_pub.publish(ros_image)

      else:
          rospy.logwarn("No lines found in the Hough transformation.")
                        # Display the image with the longest line
       
      #else:
      #  print("No contours found in the mask")
    end_time = time.time()
    elapsed_time = end_time - start_time
    
    #print("Calculate depth function took {:.2f} seconds.".format(elapsed_time))
     
  def publish_line_marker(self, X1, Y1, X2, Y2, Z1, Z2):
    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "zed_base_link"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.05  # Line width

    # Set the color (RGBA) of the line
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Create two points for the line
    point1 = Point()
    point1.x = Z1
    point1.y = -X1
    point1.z = 0  # Depth value

    point2 = Point()
    point2.x = Z2
    point2.y = -X2
    point2.z = 0  # Depth value

    # Add the points to the marker
    marker.points.append(point1)
    marker.points.append(point2)

    # Publish the marker
    self.marker_pub.publish(marker)
    



def main():
  model = YOLO('/home/user/catkin_ws/src/object_detection/src/models/best.pt','detect')
  #model = YOLO('/home/user/catkin_ws/src/object_detection/src/yolov8n-seg.engine','detect')
  #model = YOLO('yolov8n-seg.pt')
  #labels = open("labels.txt" , "w")
  rospy.init_node('Object_Detector', anonymous=True)
  ic = ObjectDetector(model)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
  main()  
 