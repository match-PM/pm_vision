from pathlib import Path
import os
import sys

#Import from absolute path
#sys.path.append('/home/niklas/ros2_ws/src/pm_vision/pm_vision/pm_vision_API')

#Import from relative path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from  vision_assistant_API import *
from  geometry_utils import *

if __name__ == "__main__":
    my_vision_assistant = vision_assistant_API()
    VisionOK, _ = my_vision_assistant.run_vision( process_file = "process_demo.json", camera_config_file = "webcam_config.yaml")
    circle_list = my_vision_assistant.get_circles_from_vision_results()
    if circle_list is not False:
        for c in circle_list:
            c.print_circle_information()
    
    if VisionOK:
        #print(my_vision_assistant.vision_results_list)
        print("Vision Assistant API Test sucessfull!")
    else:
        print("Vision Assistant API Test failed!")