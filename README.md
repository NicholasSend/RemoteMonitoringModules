# High-Definition Virtual Reality (VR) Platform for Remote Monitoring and Collaboration
This repository contains code related to a project aimed at developing a customizable VR platform for remote monitoring 
and collaboration in various fields such as industrial, medical, public safety, and entertainment. The project will 
focus on developing a basic VR system that can provide a virtual tour of the 5G Ericsson ARISE wireless lab at Carleton 
University and support remote collaboration between users.

## Background
Recent advances in VR technologies have led to the availability of off-the-shelf VR products for consumers. However, 
these products are mainly geared towards gaming and pre-made media consumption and lack customization for industrial 
use. The aim of this project is to develop an end-to-end VR solution that can be customized for remote monitoring and 
collaboration.

## Purpose
The purpose of this project is to provide the 5G Ericsson ARISE wireless lab with a state-of-the-art collaboration tool 
that can be rolled out to other facilities. The project will involve studying, evaluating, and recommending the best 
components for an end-to-end VR solution and developing the necessary software to interface them seamlessly with the 
user(s).

## Built With

Python - The language used for 

C++ - The language used for 

OpenCV - The Computer Vision Library used

OpenVR - The VR library used

PyAV - The Python library used

vidstream - The Python library used

inputs - The Python library used

## Scope and Deliverables
The project will be carried out in two phases. The first phase will involve the development of a high-definition 
360-degree VR camera and VR goggle system. The second phase will involve the development of a user-controlled 360-degree 
roaming high-definition VR camera solution with obstacle avoidance and strict route implementation.

### Phase 1
- [x] Conduct a literature survey on capabilities and requirements of various VR goggles and 360-degree cameras.
- [x] Select, purchase, and evaluate the most promising VR goggles and 360-degree cameras with respect to usability, performance specs, and reliability reviews.
- [x] Establish an end-to-end high-definition VR link between the camera and VR hardware.
- [x] Use the VR goggle head tracking to sweep the video feed from the camera.
- [x] Create a custom application to establish a secure remote VR video session.
- [x] Investigate the feasibility to integrate the solution with MS Teams and other conferencing software.
- [x] Investigate the system capacity and guarantee a smooth video feed with minimal latency and delay variation.
- [x] Find operational regimes to stress the system and assess its performance limits.

### Phase 2
- [x] Develop a small remotely controlled and semi-autonomous vehicle to transport the 360-degree camera in a remote location or lab.
- [x] [UNDER DEVELOPMENT] Develop obstacle avoidance and strict route implementation with either floor markings or image processing techniques.
- [x] Integrate standard controllers/feedback devices that accompany the VR goggles to control the VR camera transport contraption around the remote location/lab with minimal latency.
- [x] Build on the infrastructure developed in Phase 1 to make the VR camera mobile that allows a user to remotely navigate the camera around the lab over a wireless connection and the internet.
- [x] Create a custom application to establish a secure remote VR video session and also allow the integration of VR controllers to navigate the camera in the remote location/lab.

## Directory Structure
```
|   .gitignore
|   Pipfile
|   README.md
|   requirements.txt
|
+---.idea
|   |   .gitignore
|   |   .name
|   |   misc.xml
|   |   modules.xml
|   |   SYSC4907_EricssonARISE.iml
|   |   workspace.xml
|   |
|   \---inspectionProfiles
|           profiles_settings.xml
|
+---.vscode
|       launch.json
|
+---AudioVideoTransmission
|   |   AudioTransmissionVisualReciever.py
|   |   AudioVisualTransmission.py
|   |
|   +---reference
|   |       AudioTransmission.py
|   |       InternetPhone.py
|   |
|   \---__pycache__
+---ControllerTransmission
|   |   HeadTrackingDebug.py
|   |   HeadTrackingTransmission.py
|   |   InputTransmission.py
|   |   InputTransmissionReceiveTest.py
|   |
|   +---controller_objects
|   |   |   GamepadController.py
|   |   |   KeyboardController.py
|   |   |   ReverbG2.py
|   |   |
|   |   \---__pycache__
|   |           GamepadController.cpython-310.pyc
|   |
|   \---reference
|           ReverbG2.py
|           ReverbG2_1.py
|
+---ObjectAvoidance
|   +---filters
|   |   +---point_cloud
|   |   |   |   3dmap_set.txt
|   |   |   |   calibration.mp4
|   |   |   |   Calibration.py
|   |   |   |   fine_tune_1.jpg
|   |   |   |   fine_tuning.py
|   |   |   |   ObstacleAvoidance.py
|   |   |   |   PointCloudVisualization.py
|   |   |   |   stop_sign.xml
|   |   |   |
|   |   |   \---calib_result
|   |   |           cam_mats_left.npy
|   |   |           cam_mats_right.npy
|   |   |           disp_to_depth_mat.npy
|   |   |           dist_coefs_left.npy
|   |   |           dist_coefs_right.npy
|   |   |           e_mat.npy
|   |   |           f_mat.npy
|   |   |           proj_mats_left.npy
|   |   |           proj_mats_right.npy
|   |   |           rectification_map_left.npy
|   |   |           rectification_map_right.npy
|   |   |           rect_trans_left.npy
|   |   |           rect_trans_right.npy
|   |   |           rot_mat.npy
|   |   |           trans_vec.npy
|   |   |           undistortion_map_left.npy
|   |   |           undistortion_map_right.npy
|   |   |           valid_boxes_left.npy
|   |   |           valid_boxes_right.npy
|   |   |
|   |   +---reference
|   |   |   |   access_test.py
|   |   |   |   ApplyEdgeDetection.py
|   |   |   |   calibration.mp4
|   |   |   |   calibration_1.jpg
|   |   |   |   calibration_10.jpg
|   |   |   |   calibration_12.jpg
|   |   |   |   calibration_2.jpg
|   |   |   |   calibration_3.jpg
|   |   |   |   calibration_4.jpg
|   |   |   |   calibration_5.jpg
|   |   |   |   calibration_6.jpg
|   |   |   |   calibration_7.jpg
|   |   |   |   calibration_8.jpg
|   |   |   |   calibration_9.jpg
|   |   |   |   calibration_via_video.py
|   |   |   |   EdgeDetection.py
|   |   |   |   img0.jpg
|   |   |   |   output.mp4
|   |   |   |   stereopi.mp4
|   |   |   |   stereo_calibration.py
|   |   |   |   StopSignDetect.py
|   |   |   |   stop_sign.xml
|   |   |   |   video.mp4
|   |   |   |
|   |   |   \---calib_result
|   |   |           cam_mats_left.npy
|   |   |           cam_mats_right.npy
|   |   |           disp_to_depth_mat.npy
|   |   |           dist_coefs_left.npy
|   |   |           dist_coefs_right.npy
|   |   |           e_mat.npy
|   |   |           f_mat.npy
|   |   |           proj_mats_left.npy
|   |   |           proj_mats_right.npy
|   |   |           rectification_map_left.npy
|   |   |           rectification_map_right.npy
|   |   |           rect_trans_left.npy
|   |   |           rect_trans_right.npy
|   |   |           rot_mat.npy
|   |   |           trans_vec.npy
|   |   |           undistortion_map_left.npy
|   |   |           undistortion_map_right.npy
|   |   |           valid_boxes_left.npy
|   |   |           valid_boxes_right.npy
|   |   |
|   |   \---__pycache__
|   \---res
|           object_avoidance_algorithm.drawio
|           object_avoidance_algorithm.png
|
+---ROSModelNodes
|   +---arduino
|   |       ttyACM0_ard.ino
|   |       ttyACM1_ard.ino
|   |       ttyACM2_ard_servo.ino
|   |
|   +---reference
|   |       arduino.cpp
|   |       ard_intf.cpp
|   |       audio_pass.py
|   |       udp_intf.py
|   |
|   \---ros
|           api.launch
|           ard_intf.cpp
|           udp_intf.py
|           udp_intf_2.py
|
+---scratch
|       fusion_gear.py
|       yolov4-tiny.cfg
|       yolov4-tiny.weights
|
\---__pycache__
```

## Feature Breakdown

### Audio-Video Transmission

#### Description

#### Usage

### Controller Transmission

#### Description

#### Usage

### Object Avoidance

#### Description

#### Usage

### ROS Model Nodes

#### Description

#### Usage

## Authors
Nicholas Sendyk - [NicholasSend](https://github.com/NicholasSend)

Brian McDonald - [bmac1613](https://github.com/bmac1613)

## Acknowledgments
![alt text][logo_1] ![alt text][logo_2]

[logo_1]: https://carleton.ca/brand/wp-content/uploads/brand-logo-800w-1.jpg "Carleton University"

[logo_2]: https://www.ericsson.com/cdn-cgi/image/format=auto,fit=scale-down,width=700/4aa9bc/assets/global/qbank/2020/11/02/econ-logo_1500-92386511d4f7159bb1454d52ed2aa73819e7d.jpg "Ericsson Canada"

Thank you to Ericsson and Carleton University for the support on this research.

Thank you to Arsh Salym and Nikita Volochay for your support on the hardware side of the project.
As well, a special thanks to the supervisors of the project Dr. Ioannis Lambadaris and Dr. Syed Naqvi.


