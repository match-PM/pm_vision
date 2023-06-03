import os
from subprocess import call
import subprocess
import json
from pathlib import Path
import uuid
import yaml
from yaml.loader import SafeLoader

class vision_assistant_API:
    def __init__(self, vision_assistant_path_config_path):
        self.vision_process_libary_path="PATH_NOT_GIVEN"
        self.vision_database_path="PATH_NOT_GIVEN"
        self.camera_libary_path="PATH_NOT_GIVEN"
        self.vision_assistant_config_file="PATH_NOT_GIVEN"
        self.load_path_config(vision_assistant_path_config_path)
        self.default_processfile = "process_demo.json"
        self.default_camera_config = "webcam_config.yaml"

    def load_path_config(self,vision_assistant_path_config_path):
        PathNotReadSuccessfuly=True

        while(PathNotReadSuccessfuly):
            try:
                f = open(vision_assistant_path_config_path)
                FileData = yaml.load(f,Loader=SafeLoader)
                f.close()
                config=FileData["vision_assistant_path_config"]
                self.vision_process_libary_path=config["process_library_path"]
                self.vision_database_path=config["vision_database_path"]
                self.camera_libary_path=config["camera_config_path"]
                self.vision_assistant_config_file=config["vision_assistant_config"]
                print('Vision assistant path config loaded!')
                PathNotReadSuccessfuly=False
            except:
                print('Error opening vision assistant path configuration: ' + str(vision_assistant_path_config_path)+ "!")
                print("Give path to vision_assistant_path_config.yaml or enter 'exit'")
                vision_assistant_path_config_path = input()
                if vision_assistant_path_config_path == 'exit':
                    break
                    
    def process_vision_result_file(self):
        self.current_vision_result_file = self.vision_process_libary_path + '/' + Path(self.current_process_file).stem + '_results.json' 
        try:
            f = open(self.current_vision_result_file)
            FileData = json.load(f)
            self.vision_process_name=FileData['vision_process_name']
            self.VisionOK=FileData['vision_OK']
            self.VisionOK_cross_val=FileData['VisionOK_cross_val']
            self.vision_process_id=FileData['process_UID']
            self.vision_results_list=FileData['vision_results']
            print("Process Name: " + self.vision_process_name)
            print("Vision OK?: " + str(self.VisionOK))
            print("Vision Crossvalidation OK?: " + str(self.VisionOK_cross_val))
            print("Process UUID: " + self.vision_process_id)
            f.close()
            print("Results loaded successfully!")
        except Exception as e:
            print("Error opening resutls file!")
            print(e)
                        
    def run_vision(self, process_file = None, camera_config_file = None, show_node_output = False):
        self.VisionOK=False
        self.VisionOK_cross_val=False

        if process_file == None:
            self.current_process_file = self.default_processfile
        else:
            self.current_process_file = process_file

        if camera_config_file == None:
            self.current_camera_config_file = self.default_camera_config
        else:
            self.current_camera_config_file = camera_config_file

        current_process_id = uuid.uuid4()

        Vision_exec_Comand=('ros2 run pm_vision vision_assistant --ros-args ' + 
                ' -p ' + 'launch_as_assistant:=False' + 
                ' -p ' + 'process_UID:=' + str(current_process_id) +
                ' -p ' + 'process_filename:=' + self.current_process_file + 
                ' -p ' + 'camera_config_filename:=' + self.current_camera_config_file )
        
        node_terminal_output = subprocess.getstatusoutput(Vision_exec_Comand)

        if show_node_output:
            print(node_terminal_output[1])

        self.process_vision_result_file()
        return self.VisionOK , self.VisionOK_cross_val

    
    def start_vision_assistant(self, process_file = None, camera_config_file = None):
        if process_file == None:
            self.current_process_file = self.default_processfile
        else:
            self.current_process_file = process_file

        if camera_config_file == None:
            self.current_camera_config_file = self.default_camera_config
        else:
            self.current_camera_config_file = camera_config_file

        Vision_exec_Comand=('ros2 run pm_vision vision_assistant --ros-args ' + 
        ' -p ' + 'process_filename:=' + process_file + 
        ' -p ' + 'camera_config_filename:=' + camera_config_file )

        s = subprocess.getstatusoutput(Vision_exec_Comand)
        self.process_vision_result_file()
        return self.VisionOK , self.VisionOK_cross_val

if __name__ == "__main__":
    my_vision_assistant = vision_assistant_API('/home/niklas/ros2_ws/src/pm_vision/config/vision_assistant_path_config2.yaml')
    VisionOK, _ = my_vision_assistant.run_vision()
    #VisionOK, _ = my_vision_assistant.run_vision( process_file = "process_demo.json", camera_config_file = "webcam_config.yaml")
    if VisionOK:
        #print(my_vision_assistant.vision_results_list)
        print("Vision Assistant API Test sucessfull!")
    else:
        print("Vision Assistant API Test failed!")