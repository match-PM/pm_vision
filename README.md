# PM Vision Assistant
ROS2 Vision-Assistant to intuitivly build reusable vision-pipelines based on openCV functions.
## 1. Description
The pm vision assistant is designed to intuitivly build reusable vision-pipelines based on openCV functions. It's basic idea is that the vision assistant node runs different vision functions (mostly based on openCV) successively to extract a specific feature from an image. Running the node in assistant mode will show you the processed (and the original) image you subscribed to while you can modify the vision pipeline in the processfile.json. It also has an feature  to crossvalidate images from a data base with the loaded vision_process.json. 

There are some important config files to operate the vision assistant:
* `vision_assistant_config.yaml`: This config file configues how the vision assistant will operate (parameters can be changed while the node is runing).
* `camera_config.yaml`: This file specifies the parameters for the connected camera (e.a. webcam). Some of the input and (all) output parameter of the vision functions are specified in microns (calculated by the definitions in camera_config). 

Currently the assistant is designed for camera setups with telecentric lenses only! It is currently intended to be used in precision assembly tasks for fast vision pipeline deployment.

### Overview
* `config`: Contains the vision_assistant_config.yaml
* `pm_vision`: Contain the node file for the vision assistant, the python classes for the assistant and an webcam image publisher
* `vision_functions`: Description of all the vision functions supported by the vision assistant - do be done!
* `vision_db`: Default database for demonstration  
* `vision_processes`: Default folder for processes with a process_demo.json

## 2. Installation 
* To run the vision assistant the installation of ROS2 (tested on Humble) is mandatory!
* Start by changing directory to your workspace!
* Clone package
```
git clone https://github.com/match-PM/match_vision.git
```
* Go into pm_vision/config/vision_assistant_config.yaml and change:
```
process_library_path
```
```
vision_database_path
```
```
camera_config_path
```
The pathes you insert in the config.yaml specify where the vision assitants looks for processes and camera_configs and where to save images. You can specify any path here. 

## 3. Getting Started
If you leave (modify accordingly) the default file pathes in the vision_assistant_config.yaml, the vision assistant will launch with the process_demo.json. Images will be saved in vision_db/process_demo. By default it will load the camera parameter from config/webcam_config.yaml. By default it will subscribe to the topic 'video_frames'.

### To start the node
```
ros2 run pm_vision vision_assistant
```
### Start the webcam publisher (a webcam needs to be connected)
```
ros2 run pm_vision vision_webcam_publisher
```
### Remap the node to start with a different process file
```
ros2 run pm_vision vision_assistant --ros-args -p process_filename:=PROCESS_FILENAME.json

```
### Remap the node to subscribe to a different image topic
```
ros2 run pm_vision vision_assistant --ros-args -r video_frames:=/Cam1/image_raw

```
### Remap the node to start in process mode (by default starts in assistant mode)
```
ros2 run pm_vision vision_assistant --ros-args -r launch_as_assistant:=False

```
### Remap the node to start with a different camera config file
```
ros2 run pm_vision vision_assistant --ros-args -p camera_config_filename:=CAMERA_config.yaml

```
### Remap the node to start assistant only with data base images
```
ros2 run pm_vision vision_assistant --ros-args -p db_cross_val_only:=True

```
## 4. Working with the vision assistant
Tutorials for working with the assistant will be given here...
### Build a new vision pipeline for a new task
...
### Build a vision pipeline for images in a data base
...

## 5. To Do's
* Conditions
* Function yaml descriptions
* cleaner code
* Returns
* camera coordinate system

* GUI to build vision pipelines in json and to configure the vision asssistant.

## 5. External documentation
[ROS 2 - Humble - Documentation and Tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


