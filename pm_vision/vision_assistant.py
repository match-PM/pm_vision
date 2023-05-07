# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import json
from pm_vision.vision_utils import match_vision_function
import numpy as np



def Conv_Pixel_Top_Left_TO_Center(img_width, img_height, x, y):
     x_center=int(x-img_width/2)
     y_center=int(y-img_height/2)
     return x_center, y_center

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
    self.file_path= '/home/niklas/ros2_ws/src/pm_vision/vision_processes/test_function.json'
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
      
    # We will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
    self.img_width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    self.img_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    self.magnification = 3
    self.pixelsize = 2    # in um
    self.pixelPROum=self.magnification/self.pixelsize
    self.umPROpixel=self.pixelsize/self.magnification
    self.FOV_width=self.umPROpixel*self.img_width 
    self.FOV_height=self.umPROpixel*self.img_height

    print("FOV width is " + str(self.FOV_width) + "um")
    print("FOV hight is " + str(self.FOV_height) + "um")

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    

  
     
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
    frame_processed = frame
    display_frame=frame_processed
    self.VisionOK = True
    try:
      f = open(self.file_path)
      FileData = json.load(f)
      pipeline_list=FileData['vision_pipeline']
      for list_item in pipeline_list:
      # Iterating through the json
        for key, function_parameter in list_item.items():
          match key:
            case "threshold":
                active = function_parameter['active']
                thresh = function_parameter['thresh']
                maxval = function_parameter['maxval']
                type = function_parameter['type'] 
                if active == 'True':
                    _Command = "cv2." + type
                    _,frame_processed = cv2.threshold(frame_processed,thresh,maxval,exec(_Command))
                    print("Theshold executed")
                    display_frame=frame_processed
            case "adaptiveThreshold":
                active = function_parameter['active']
                maxValue = function_parameter['maxValue']
                adaptiveMethod = function_parameter['adaptiveMethod']
                thresholdType = function_parameter['thresholdType']
                blockSize = function_parameter['blockSize'] 
                C_Value = function_parameter['C']
                if active == 'True':
                    _Command_adaptiveMethod = "cv2." + adaptiveMethod
                    _Command_thresholdType = "cv2." + thresholdType
                    frame_processed = cv2.adaptiveThreshold(frame_processed,maxValue,exec(_Command_adaptiveMethod),exec(_Command_thresholdType),blockSize,C_Value)
                    print("Adaptive Theshold executed")
                    display_frame=frame_processed
            case "bitwise_not":
                active = function_parameter['active']
                if active == 'True':
                    frame_processed = cv2.bitwise_not(frame_processed)
                    print("bitwise_not executed")
                    display_frame=frame_processed
            case "BGR2GRAY":
                active = function_parameter['active']
                if active == 'True':
                    frame_processed = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2GRAY)
                    print("BGR2GRAY executed")
                    display_frame=frame_processed
            case "Canny":
                active = function_parameter['active']
                threshold1 = function_parameter['threshold1']
                threshold2 = function_parameter['threshold2']
                aperatureSize = function_parameter['aperatureSize']
                L2gradient = function_parameter['L2gradient'] 
                if active == 'True':
                    frame_processed = edges = cv2.Canny(frame_processed,threshold1,threshold2,aperatureSize)
                    print("Canny executed")
                    display_frame=frame_processed
            case "findContours":
                active = function_parameter['active']
                draw_contours = function_parameter['draw_contours']
                mode = function_parameter['mode']
                method = function_parameter['method']
                fill = function_parameter['fill']
                if active == 'True':
                    _Command_mode = "cv2." + mode
                    _Command_method = "cv2." + method
                    #contours, hierarchy  = cv2.findContours(frame_processed, exec(_Command_mode), exec(_Command_method))  # Keine Ahnung warum das nicht funktioniert!!!
                    #print(_Command_method)
                    contours, hierarchy  = cv2.findContours(frame_processed, exec(_Command_mode), cv2.CHAIN_APPROX_SIMPLE)
                    if fill =="True":
                      cv2.fillPoly(frame_processed,pts=contours,color=(255,255,255))
                      display_frame=frame_processed
                    if draw_contours == 'True':
                      display_frame = cv2.cvtColor(frame_processed,cv2.COLOR_GRAY2RGB)
                      display_frame = cv2.drawContours(display_frame, contours, -1, (0,255,75), 2)
                    print("findContours executed")
            case "select_Area":
                active = function_parameter['active']
                mode = function_parameter['mode']
                method = function_parameter['method']
                max_area = function_parameter['max_area']
                min_area = function_parameter['min_area']
                if active == 'True':
                    _Command_mode = "cv2." + mode
                    _Command_method = "cv2." + method
                    #contours, hierarchy  = cv2.findContours(frame_processed, exec(_Command_mode), exec(_Command_method))  # Keine Ahnung warum das nicht funktioniert!!!
                    contours, hierarchy  = cv2.findContours(frame_processed, exec(_Command_mode), cv2.CHAIN_APPROX_SIMPLE)
                    all_areas= []
                    for cnt in contours:
                        area= cv2.contourArea(cnt)
                        all_areas.append(area)
                    contour_frame = np.zeros((frame_processed.shape[0], frame_processed.shape[1], 1), dtype = np.uint8)
                    self.VisionOK = False
                    for index, area_item in enumerate(all_areas):
                        
                        if area_item<max_area and area_item>min_area:
                          contour_frame = cv2.drawContours(contour_frame, contours, index, color=(255,255,255), thickness=cv2.FILLED)
                          self.VisionOK = True
                                      
                    if not self.VisionOK:
                      print("No matching Area")
                    
                    frame_processed = contour_frame
                    display_frame = frame_processed
                    print("select_Area executed")
            case "Morphology_Ex_Opening":
                active = function_parameter['active']
                kernelsize = function_parameter['kernelsize']
    
                if active == 'True':
                    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelsize, kernelsize))
                    frame_processed = cv2.morphologyEx(frame_processed, cv2.MORPH_OPEN, kernel)
                    display_frame=frame_processed
                    print("Morphology_Ex_Opening executed")
            case "Draw_Grid":
                active = function_parameter['active']
                grid_spacing = function_parameter['grid_spacing']
                
                if active == 'True':
                    display_frame=cv2.line(display_frame, (int(display_frame.shape[1]/2), 0),(int(display_frame.shape[1]/2), display_frame.shape[1]), (255, 0, 0), 1, 1)
                    display_frame=cv2.line(display_frame, (0,int(display_frame.shape[0]/2)),(int(display_frame.shape[1]), int(display_frame.shape[0]/2)), (255, 0, 0), 1, 1)
                    print("Grid executed")

            case "HoughCircles":
                active = function_parameter['active']
                draw_circles = function_parameter['draw_circles']
                method = function_parameter['method']
                dp = function_parameter['dp']
                minDist = function_parameter['minDist']
                param1 = function_parameter['param1']
                param2 = function_parameter['param2']
                minRadius = int(function_parameter['minRadius']* self.pixelPROum) # conv in Pixel
                maxRadius = int(function_parameter['maxRadius']* self.pixelPROum) # conv in Pixel
                if active == 'True':
                  _Command_method = "cv2." + method
                  #print(_Command_method)
                  try:
                    detected_circles = cv2.HoughCircles(frame_processed,cv2.HOUGH_GRADIENT, dp, minDist, param1 = param1,param2 = param2, minRadius = minRadius, maxRadius = maxRadius)
                    if detected_circles is not None:
                      print("Cirlces Detected")
                      # Convert the circle parameters a, b and r to integers.
                      detected_circles = np.uint16(np.around(detected_circles))
                      if draw_circles == 'True':
                        for pt in detected_circles[0, :]:
                          x, y, r = pt[0], pt[1], pt[2]                          
                          x_center_pix, y_center_pix =Conv_Pixel_Top_Left_TO_Center(frame_processed.shape[1], frame_processed.shape[0],x,y)
                          x_center_um=x_center_pix*self.umPROpixel
                          y_center_um=y_center_pix*self.umPROpixel
                          radius_um=r*self.umPROpixel
                          print(x_center_um)
                          print(y_center_um)
                          print(radius_um)
                          # Conv to RGB to add Circles to display
                          display_frame = cv2.cvtColor(frame_processed,cv2.COLOR_GRAY2RGB)
                          # Draw the circumference of the circle.
                          cv2.circle(display_frame, (x, y), r, (0, 255, 0), 2)
                          # Draw a small circle (of radius 1) to show the center.
                          cv2.circle(display_frame, (x, y), 1, (0, 0, 255), 2)
                    else:
                      self.VisionOK=False
                    print("Hough Circles executed")
                  except:
                    print("Circle detection failed! Image may not be grayscale!")

      # Closing file
      f.close()
    except:
        #print("Json Loading Error")
        print("Error in Vision Function")
    #print(type(thresh))

    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      #self.publisher_.publish(self.br.cv2_to_imgmsg(frame_processed))
      self.publisher_.publish(self.br.cv2_to_imgmsg(display_frame))
    # Display the message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
