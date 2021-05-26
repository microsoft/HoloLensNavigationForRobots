# HoloLens ROS Navigation Package

## Overview

This is HoloLens bridge example for Robot navigation in ROS system. It contains three modules:

### HoloLensBridge
Universal Windows Platform (UWP) application for HoloLens. Communicates with HoloROSBridge.

### HoloROSBridge
ROS package of HoloLens brigde.
Module for using HoloLens in ROS system.

### HoloLens_Localization
ROS package including HoloLens Localization module, 
offline calibration between HoloLens and Robot's head, and online calibration between HoloLens and Robot's base.

## Prerequisites, installation and build

Follow instructions in the [setup instructions](Setup/README.md).

## Tips

General tips aiding in the deployment, handling Pepper, accessing HoloLens, etc can be found [here](/Setup/TIPS.md).


## How to use

### Advance Preparation
Follow instructions how to build and install the map [here](Setup/MAP.md).

Attach HoloLens to Robot's Head (instructions TODO)

### Running Process
#### HoloLens Stack
- (HoloLens) Boot HoloLens Bridge
    - Launch the HoloLensNavigation application from Device Portal (access the HoloLens ip from browser). Or use alternative methods.
- (ROS) Launch Pepper's stack
    - `$  roslaunch pepper_bringup pepper_full.launch nao_ip:=<pepper ip> network_interface:=<network interface>`
    - The local ROS computer's network interface name can be found using the terminal command "ifconfig" and looking for the name associated with the active IP address. Do not include the colon after the network interface.
    - Ideally start Pepper with life disabled. Use Choregraph or refer to the [tips](/Setup/TIPS.md) document for alternative options.
- (ROS) Launch HoloLens stack
    - `$ roslaunch navigation_launcher hololensstack.launch HoloLens_ip:=<hololens ip>`
    - Note that XTerm needs to be installed for this as the script uses it to interact with the calibration.

#### Navigation Stack
- (ROS) Launch Navigation program
    - ```roslaunch navigation_launcher navstack.launch```

#### Calibration
- in the calibration window:
    - move Pepper's head into inital/default pose. Use either Choregraph or connect to Pepper via SSH and set the pitch directly:
      - ```qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3```
      - ```qicli call ALMotion.setAngles "HeadYaw" 0.0 0.3```
    - press ```space``` to record the initial position.
    - move Pepper's head upward. Use either Choregraph or connect to Pepper via SSH and set the pitch directly:
      - ```qicli call ALMotion.setAngles "HeadPitch" -0.35 0.3```
    - press ```space``` again to record the new position.
    - reset Pepper's head pitch and then rotate to left. Use either Choregraph or connect to Pepper via SSH:
      - ```qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3```
      - ```qicli call ALMotion.setAngles "HeadYaw" 0.7 0.3```
    - press ```space``` to record the new position.
    - rotate Pepper's head to the right. Use either Choregraph or connect to Pepper via SSH:
      - ```qicli call ALMotion.setAngles "HeadYaw" -0.7 0.3```
    - press ```space``` to record the new position.
    - press ```c``` to calibrate.
    - reset Pepper's head pitch and rotation. Use either Choregraph or connect to Pepper via SSH:
      - ```qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3```
      - ```qicli call ALMotion.setAngles "HeadYaw" 0.0 0.3```

#### Navigation
- (ROS) Launch rviz
    - `$  rosrun rviz rviz`
    - add Map and Pepper RobotModel topics. Alternatively, load the [pepper.rviz](rviz/pepper.rviz) rviz configuration file.
- In rviz, select `2D Pose Estimate` and set Pepper's inital position and direction on the map. Try to be as precise as 
 possible. The script will calculate a pose estimate and localize the Pepper model.
- In rviz, select `2D Nav Goal` and select a destination goal and orientation on the map.
- Pepper navigation will start.



### Running Process (INDIVIDUAL NODES)
- Pepper ROS full stack
  - ```$ roslaunch pepper_bringup pepper_full.launch nao_ip:=<pepper ip> network_interface:=<network interface>```
  - example: ```roslaunch pepper_bringup pepper_full.launch nao_ip:=10.1.1.202 network_interface:=enp3s0```
- HoloLens ROS Bridge
  - ```$ rosrun hololens_ros_bridge hololens_ros_bridge_node <hololens_ip> 1234```
  - example: ```rosrun hololens_ros_bridge hololens_ros_bridge_node 10.1.1.206 1234```
- ROS map_server
  - ```$ rosrun map_server map_server src/navigation_launcher/params/map.yaml```
- HoloLens Anchor Localizer
  - ```$ rosrun hololens_localizer anchor_localizer```
- Localizer Calibration
  - ```$ rosrun hololens_localizer static_calibration <robot odom frame> <robot head frame> <robot base link> [calibrationFileName]```
  - example: ```rosrun hololens_localizer static_calibration odom Head base_footprint calibrationData.bin```
- Dynamic Adjuster
  - ```$ rosrun hololens_localizer dynamic_adjuster.py <robot foot frame>```
  - example: ```rosrun hololens_localizer dynamic_adjuster.py```
