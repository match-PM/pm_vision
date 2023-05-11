# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import json
from ament_index_python.packages import get_package_share_directory
from pm_vision.vision_utils import match_vision_function
import numpy as np
import os
from os import listdir
from datetime import datetime
import yaml
import fnmatch
from yaml.loader import SafeLoader
import subprocess
from ament_index_python.packages import get_package_share_directory


def get_screen_resolution():
   output = subprocess.Popen('xrandr | grep "\*" | cut -d " " -f4', shell=True, stdout=subprocess.PIPE).communicate()[0]
   resolution = output.split()[0].split(b'x')
   return {'width':resolution[0],'height':resolution[1]}

def image_resize(image, width=None, height = None, inter = cv2.INTER_AREA):
  dim = None
  (h,w)=image.shape[:2]

  if width is None and height is None:
    return image
  if width is None:
    r=height/float (h)
    dim = (int(w*r),height)
  else:
    r=width/float(w)
    dim=(width,int(h*r)) 
  resized = cv2.resize(image, dim ,interpolation=inter)
  return resized

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
    super().__init__('vision_assistant')
    self.declare_parameter('launch_as_assistant', True)      # 'execute_process'
    self.declare_parameter('process_filename','process_demo.json')
    self.declare_parameter('camera_config_filename','webcam_config.yaml')
    self.declare_parameter('db_cross_val_only', False)
    
    self.vision_filename = self.get_parameter('process_filename').value
    #ament_index_python.get_resource('package', 'rclcpp')

    # may raise PackageNotFoundError
    package_share_directory = get_package_share_directory('pm_vision')
    print(package_share_directory)
    self.config_file_path = get_package_share_directory('pm_vision') + '/vision_assistant_config.yaml'

    #self.config_file_path= '/home/niklas/ros2_ws/src/pm_vision/config/vision_assistant_config.yaml'

    #self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
    # Create image subscriber
    if self.get_parameter('launch_as_assistant').value:
      self.get_logger().info('Starting node in assistant mode!')
    else:
      self.get_logger().info('Starting node in processing mode!')

    self.get_logger().info('Current vision process: ' + self.get_parameter('process_filename').value)

    timestamp = datetime.now()
    self.screen_resolution=get_screen_resolution()
    self.screen_height=int(self.screen_resolution["height"].decode('UTF-8'))
    self.screen_width=int(self.screen_resolution["width"].decode('UTF-8'))
    self.get_logger().info('Screen resolution: '+ str(self.screen_width)+'x'+str(self.screen_height))

    self.process_start_time=timestamp.strftime("%d_%m_%Y_%H_%M_%S")
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.counter_error_cross_val=0
    self.VisionOK_cross_val=True

    self.load_assistant_config()
    self.process_file_path = self.process_library_path + self.vision_filename
    self.load_process_file_metadata()
    self.load_camera_config()

    if self.get_parameter('db_cross_val_only').value:
      self.cycle_though_db()
    else:
      self.subscription = self.create_subscription(
        Image, 
        'video_frames', 
        self.Vision_callback, 
        10)
      self.subscription # prevent unused variable warning  
    

  def load_assistant_config(self):
    try:
      f = open(self.config_file_path)
      FileData = yaml.load(f,Loader=SafeLoader)
      config=FileData["vision_assistant_config"]
      self.cross_validation=config["cross_validation"]
      self.show_image_on_error=config["show_image_on_error"]
      self.step_though_images=config["step_though_images"]
      self.process_library_path=config["process_library_path"]
      self.vision_database_path=config["vision_database_path"]
      self.image_display_time_in_execution_mode=config["image_display_time_in_execution_mode"]*1000
      self.camera_config_path=config["camera_config_path"]
      self.show_input_and_output_image=config["show_input_and_output_image"]

      self.process_file_path = self.process_library_path + self.vision_filename
      f.close()
      self.get_logger().info('Vision assistant config loaded!')
    except:
      self.get_logger().error('Error opening vision assistant configuration: ' + str(self.config_file_path)+ "!")

  def load_camera_config(self):
    try:
      f = open(self.camera_config_path+self.get_parameter('camera_config_filename').value)
      FileData = yaml.load(f,Loader=SafeLoader)
      config=FileData["camera_params"]
      self.pixelsize=config["pixelsize"]
      self.magnification=config["magnification"]
      self.camera_axis_1=config["camera_axis_1"]
      self.camera_axis_2=config["camera_axis_2"]
      # Calculate Camera parameter
      self.pixelPROum=self.magnification/self.pixelsize
      self.umPROpixel=self.pixelsize/self.magnification
      f.close()
      self.get_logger().info('Camera config loaded!')
    except:
      self.get_logger().error('Error opening camera config file: ' + str(self.camera_config_path+self.get_parameter('camera_config_filename').value)+ "!")

  def load_process_file_metadata(self):
    try:
      f = open(self.process_file_path)
      FileData = json.load(f)
      self.vision_process_name=FileData['vision_process_name']
      self.vision_process_id=FileData['id_process']
      self.process_db_path=self.vision_database_path+self.vision_process_name
      f.close()
      self.get_logger().info('Process meta data loaded!')
    except:
      self.get_logger().error('Error opening process file: ' + str(self.process_file_path)+ "!")

  def create_vision_element_overlay(self,displ_frame,vis_elem_frame):
    #Add visual elements to the display frame
    if len(displ_frame.shape)<3:
      displ_frame = cv2.cvtColor(displ_frame,cv2.COLOR_GRAY2BGR)
    # Now create a mask of logo and create its inverse mask also
    mask_frame = cv2.cvtColor(vis_elem_frame,cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(mask_frame, 10, 255, cv2.THRESH_BINARY)
    mask_inv = cv2.bitwise_not(mask)
    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(displ_frame,displ_frame,mask = mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(vis_elem_frame,vis_elem_frame,mask = mask)
    # Put logo in ROI and modify the main image
    displ_frame = cv2.add(img1_bg,img2_fg)
    return displ_frame
  
  def process_image(self,received_frame):

    self.img_width  = received_frame.shape[1]
    self.img_height = received_frame.shape[0]
    self.FOV_width=self.umPROpixel*self.img_width 
    self.FOV_height=self.umPROpixel*self.img_height

    print("FOV width is " + str(self.FOV_width) + "um")
    print("FOV hight is " + str(self.FOV_height) + "um")    
    frame_buffer=[]
    frame_processed = received_frame
    frame_visual_elements = np.zeros((received_frame.shape[0], received_frame.shape[1], 3), dtype = np.uint8)
    display_frame=received_frame
    frame_buffer.append(received_frame)
    self.VisionOK = True
    try:
      f = open(self.process_file_path)
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
                    frame_buffer.append(frame_processed)
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
                    frame_buffer.append(frame_processed)
            case "bitwise_not":
                active = function_parameter['active']
                if active == 'True':
                    frame_processed = cv2.bitwise_not(frame_processed)
                    print("bitwise_not executed")
                    display_frame=frame_processed
                    frame_buffer.append(frame_processed)

            case "BGR2GRAY":
                active = function_parameter['active']
                if active == 'True':
                  if len(frame_processed.shape)==3:
                    frame_processed = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2GRAY)
                    display_frame=frame_processed
                    frame_buffer.append(frame_processed)
                    print("BGR2GRAY executed")
                  else:
                    print("BGR2GRAY ignored! Image is already Grayscale!")
                       

            case "Canny":
                active = function_parameter['active']
                threshold1 = function_parameter['threshold1']
                threshold2 = function_parameter['threshold2']
                aperatureSize = function_parameter['aperatureSize']
                L2gradient = function_parameter['L2gradient'] 
                if active == 'True':
                    frame_processed = cv2.Canny(frame_processed,threshold1,threshold2,aperatureSize)
                    print("Canny executed")
                    display_frame=frame_processed
                    frame_buffer.append(frame_processed)

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
                      frame_buffer.append(frame_processed)
                    if draw_contours == 'True':
                      cv2.drawContours(frame_visual_elements, contours, -1, (0,255,75), 2)
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
                    contour_frame = np.zeros((frame_processed.shape[0], frame_processed.shape[1]), dtype = np.uint8)
                    self.VisionOK = False
                    for index, area_item in enumerate(all_areas):
                        
                        if area_item<max_area and area_item>min_area:
                          frame_processed = cv2.drawContours(contour_frame, contours, index, color=(255,255,255), thickness=cv2.FILLED)
                          self.VisionOK = True
                                      
                    if not self.VisionOK:
                      self.VisionOK = False
                      self.counter_error_cross_val += 1
                      print("No matching Area")
                    display_frame = frame_processed
                    frame_buffer.append(frame_processed)
                    print("select_Area executed")

            case "Morphology_Ex_Opening":
                active = function_parameter['active']
                kernelsize = function_parameter['kernelsize']
    
                if active == 'True':
                    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelsize, kernelsize))
                    frame_processed = cv2.morphologyEx(frame_processed, cv2.MORPH_OPEN, kernel)
                    display_frame=frame_processed
                    frame_buffer.append(frame_processed)
                    print("Morphology_Ex_Opening executed")

            case "save_image":
                active = function_parameter['active']
                prefix = function_parameter['prefix']
                with_vision_elements = function_parameter['with_vision_elements']
                if active == 'True':
                    
                    if not os.path.exists(self.process_db_path):
                      os.makedirs(self.process_db_path)
                      print("Process DB folder created!")
                    
                    image_name=self.process_db_path+"/"+self.vision_process_id+"_"+self.process_start_time+prefix+".png"
                    if not os.path.isfile(image_name):
                      cv2.imwrite(image_name,frame_processed)
                      print("Image saved!")
                    print("save_image executed")
                    
            case "Draw_Grid":
                active = function_parameter['active']
                grid_spacing = function_parameter['grid_spacing']
                
                if active == 'True':
                    numb_horizontal = int((self.FOV_height/2)/grid_spacing)+1
                    numb_vertical = int((self.FOV_width/2)/grid_spacing)+1
                    cv2.putText(img=frame_visual_elements,text="Grid: "+ str(grid_spacing) + "um", org=(5,30), fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1,color=(255,0,0), thickness=1)
                    cv2.line(frame_visual_elements, (int(display_frame.shape[1]/2), 0),(int(display_frame.shape[1]/2), display_frame.shape[1]), (255, 0, 0), 1, 1)
                    cv2.line(frame_visual_elements, (0,int(display_frame.shape[0]/2)),(int(display_frame.shape[1]), int(display_frame.shape[0]/2)), (255, 0, 0), 1, 1)
                    #Draw horizontal lines
                    for numb_h in range(numb_horizontal):
                      pixel_delta=int(numb_h*grid_spacing*self.pixelPROum)
                      cv2.line(frame_visual_elements, (0,int(display_frame.shape[0]/2)+pixel_delta),(int(display_frame.shape[1]), int(display_frame.shape[0]/2)+pixel_delta), (255, 0, 0), 1, 1)
                      cv2.line(frame_visual_elements, (0,int(display_frame.shape[0]/2)-pixel_delta),(int(display_frame.shape[1]), int(display_frame.shape[0]/2)-pixel_delta), (255, 0, 0), 1, 1)
                    # draw vertical lines
                    for numb_v in range(numb_vertical):
                      pixel_delta=int(numb_v*grid_spacing*self.pixelPROum)
                      cv2.line(frame_visual_elements, (int(display_frame.shape[1]/2)+pixel_delta, 0),(int(display_frame.shape[1]/2)+pixel_delta, display_frame.shape[1]), (255, 0, 0), 1, 1)
                      cv2.line(frame_visual_elements, (int(display_frame.shape[1]/2)-pixel_delta, 0),(int(display_frame.shape[1]/2)-pixel_delta, display_frame.shape[1]), (255, 0, 0), 1, 1)
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
                          print('X-Coordinate: '+ str(x_center_um))
                          print('X-Coordinate: '+ str(y_center_um))
                          print('Radius: '+ str(radius_um))
                          # Conv to RGB to add Circles to display
                          # Draw the circumference of the circle.
                          cv2.circle(frame_visual_elements, (x, y), r, (0, 255, 0), 2)
                          # Draw a small circle (of radius 1) to show the center.
                          cv2.circle(frame_visual_elements, (x, y), 1, (0, 0, 255), 2)
                    else:
                      self.VisionOK=False
                      self.counter_error_cross_val += 1
                    print("Hough Circles executed")
                  except:
                    print("Circle detection failed! Image may not be grayscale!")
      if self.VisionOK:
        print("Vision executed cleanly!")
        #print(len(frame_buffer))
      else:
        print("Vision execuded with Error!")
      # Closing file
      f.close()
    except:
        #print("Json Loading Error")
        self.get_logger().error("Vision Pipline executed with Error!")
    
    if not self.VisionOK:
      cv2.rectangle(frame_visual_elements,(0,0),(frame_visual_elements.shape[1],frame_visual_elements.shape[0]),(0,0,255),3)
    else:
      cv2.rectangle(frame_visual_elements,(0,0),(frame_visual_elements.shape[1],frame_visual_elements.shape[0]),(0,255,0),3)
    
    display_frame=self.create_vision_element_overlay(display_frame,frame_visual_elements)

    if len(received_frame.shape)<3:
      received_frame = cv2.cvtColor(received_frame,cv2.COLOR_GRAY2BGR)

    if  self.show_input_and_output_image:
      display_frame = cv2.vconcat([received_frame,display_frame])

    display_frame=image_resize(display_frame, height = (self.screen_height-100))

    return display_frame
  
  def cycle_though_db(self):
    try:
      while(True):
        k=cv2.waitKey(1) & 0xFF
        self.run_crossvalidation()
    except KeyboardInterrupt:
      pass
    
  def run_crossvalidation(self):
    # Starting cross validation with images in folder
    if self.cross_validation:
      self.counter_error_cross_val=0
      self.VisionOK_cross_val=True
      print("----------------------------")
      print("Starting Cross Validation...")
      # Get number of images to be processed
      numb_images_cross_val=len(fnmatch.filter(os.listdir(self.process_db_path),'*.png'))
      print(str(numb_images_cross_val)+" images for crossvalidation")
      for image_in_folder in os.listdir(self.process_db_path):
        if (image_in_folder.endswith(".png")):
          image=cv2.imread(self.process_db_path+"/"+image_in_folder)
          print("----------------------------")
          print("Processing image: " + image_in_folder)
          # Calculate VisionOK on image
          display_image = self.process_image(image)

          if not self.VisionOK:
            self.VisionOK_cross_val=False

          #if image_in_folder=="001_08_05_2023_20_09_28_3.png":
          #   self.VisionOK=False
          if (self.get_parameter('launch_as_assistant').value):
            if (not self.VisionOK and self.show_image_on_error) or self.step_though_images: 
              while(True):
                display_image = self.process_image(image)
                cv2.imshow("PM Vision Assistant", display_image)
                k = cv2.waitKey(1)& 0xFF
                if k == ord('q'):
                  break
      if self.counter_error_cross_val==0:
        self.get_logger().info("Crossvalidation exited with no Error!")

      else:
        print("Crossvalidation error!")
        self.get_logger().warning('Crossvalidation had errors! ' + str(self.counter_error_cross_val)+"/"+str(numb_images_cross_val)+" images had errors!")
  
  def Vision_callback(self,data):

    self.load_assistant_config()
    self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image
    received_frame = self.br.imgmsg_to_cv2(data)

    display_image = self.process_image(received_frame)
    # Show image
    cv2.imshow("PM Vision Assistant", display_image)
    
    if (self.get_parameter('launch_as_assistant').value):
      cv2.waitKey(1)
    else:
      cv2.waitKey(self.image_display_time_in_execution_mode)
      cv2.destroyAllWindows()

    self.run_crossvalidation()
       
    if not (self.get_parameter('launch_as_assistant').value):
      self.get_logger().info('Vision Process Ended!')
      self.destroy_node()

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
