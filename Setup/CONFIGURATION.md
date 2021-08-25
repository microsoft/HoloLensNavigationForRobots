## ![logo](../img/MARR_logo.png) [Microsoft Applied Robotics Research Library](https://special-giggle-b26bab5f.pages.github.io/)
### Open Source Samples for Service Robotics
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

# HoloLens Mounting and Pepper Configuration Instructions
After installation, the following steps are required to configure the system components in order to connect and function with each other.

## Mount HoloLens on Pepper
The HoloLens device can be conveniently secured to the Pepper robot's head using adhesive velcro strips.

Assembly Steps:
 - First, access the Pepper's joint-release key which is stored under the rubber covering near the Emergency Power Off button behind the robot's neck:
![HololensNavigation RVIZ Config File](../img/HololensNavigation_JointKeyStorage.jpg) 

 - Use the joint-release key to remove the access-cover on the back of the robot's head by inserting it into the two holes below the cover:
![HololensNavigation RVIZ Config File](../img/HololensNavigation_RemoveHeadPlate.jpg) 

 - Make a head back strap with adhesive velcro by cutting 10-inch strips of both the hook and loop sides and sticking their adhesive sides together.  Attach the strap by slipping it between the flexible fan housing and the head-cover support beam:
![HololensNavigation RVIZ Config File](../img/HololensNavigation_MountHeadBackStrap.jpg) 

 - Affix a 2.5 inch and a 3.5 inch adhesive velcro loop pad to the top of the robot's head in the positions shown:
![HololensNavigation RVIZ Config File](../img/HololensNavigation_MountHeadPads.jpg) 

 - Set the HoloLens device over the robot's head and open the head-band adjust band all the way by rotating the knob all the way to the left:
![HololensNavigation RVIZ Config File](../img/HololensNavigation_BandAdjust.jpg) 

 - Secure the back of the HoloLens device to the back of the robot's head by tightly creating a closed loop with the two-sided Velcro head strap.
![HololensNavigation RVIZ Config File](../img/HololensNavigation_HeadStrap.jpg) 

 - Make a second mount strap by creating a 2.5 inch two-sided Velcro strip and securing it over the loop surface of both the HoloLens headstrap and rear mounting pad.
![HololensNavigation RVIZ Config File](../img/HololensNavigation_HeadPadRear.jpg) 

 - Make a third mount strap by creating a 3.5 inch two-sided Velcro strip and securing it over the loop surface of both the HoloLens headstrap and front mounting pad and bringing over the hook side of the HoloLens headstrap to the loop side of the same strap:
![HololensNavigation RVIZ Config File](../img/HololensNavigation_HeadPadFront.jpg) 

## HoloLens Device Portal

Enable Device Portal on the HoloLens device in `Settings->For developers`

Navigate to the HoloLens IP using your web browser and set up a user/password.

## Certificate Installation
Navigate to the HoloLens IP using your web browser and download the HoloLens certificate. Convert and install it:

```
$ sudo apt-get install ca-certificates -y
$ openssl x509 -outform der -in certificate.pem -out certificate.crt
$ sudo cp certificate.crt /usr/local/share/ca-certificates
$ sudo update-ca-certificates
```

## Change or disable HoloLens sleep settings
Navigate to the HoloLens IP using your web browser and log in with the user/pwd set above. In System->Preferences set sleep settings. To disable, use the browser's inspect feature and add `0` as a list option, then select `0`. Note you will need to do this twice if you want to disable sleep for both `battery` and `plugged-in` settings.

# Pepper Configuration

Pepper [qicli](http://doc.aldebaran.com/2-5/dev/libqi/guide/qicli.html) commands can be issued via the Choregraphe application or directly using a secure shell (SSH) terminal.

## Start Pepper with autonomous life disabled

Disable Pepper autonomous life mode using Choregraph:

- Connect to Pepper using Choregraph.
- Click on blue heart icon in upper right corner.
- Wake Pepper up by clicking on sunshine icon in upper right corner
 
Disable Pepper autonomous life mode using `ssh`:

```
$ ssh nao@<pepper IP>
  > nao stop
  > naoqi-bin --disable-life
```

Connect to Pepper again and:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.wakeUp
```

To make Pepper go to sleep:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.rest
```

To shut down Pepper:
```
$ ssh nao@<pepper IP>
  > sudo shutdown -h now
```

To get the joint names for the body or a chain:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.getBodyNames "Body"
```

To view Pepper's current joint state:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.getSummary
```

To change Pepper's head pitch:
```
$ ssh nao@<pepper IP>
  > qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3
```

The `setAngles` call is a non-blocking call. Parameters:
- `names` – The name or names of joints, chains, “Body”, “JointActuators”, “Joints” or “Actuators”.
- `angles` – One or more angles in radians
- `fractionMaxSpeed` – The fraction of maximum speed to use

Valid Pepper joint names can be found [here](http://doc.aldebaran.com/2-5/family/romeo/joints_romeo.html?highlight=joint) and [here](https://developer.softbankrobotics.com/nao6/nao-documentation/nao-developer-guide/kinematics-data/effector-chain-definitions#nao-chains),
or call `getBodyNames` for a complete list.

## Pepper RViz configuration file

location: `/opt/ros/melodic/share/naoqi_driver/share/pepper.rviz`


# Miscellaneous
The folowing instructions are optional and provide alternative methods to perfom setup and configuration actions. They are intended to be helpful in system modification and/or troubleshooting.

## Enumerate network interfaces on Ubuntu:
The following IP commands can be used on the Navigation PC's console UI to expose values needed for configuration:
```
$ ip l show
$ ip a show eno1
```

## Naoqi Commands for Calibration Positions
As an alternative to the Dashboard UI, the console UI in the calibration window can be accessed via an SSH terminal to set the head angles directly. In these examples the first value indicates the joint motor, the second value indicates the angle to move to in radians and last value indicates the speed of movement in seconds.

- move Pepper's head into inital/default pose: 
```
$ qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3
```
```
$ qicli call ALMotion.setAngles "HeadYaw" 0.0 0.3
```
- press ***space bar*** to record the initial position
- move Pepper's head upward:
```
$ qicli call ALMotion.setAngles "HeadPitch" -0.35 0.3
```
- press ***space bar*** again to record the new position
- reset Pepper's head pitch and then rotate to left:
```
$ qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3
```
```
$ qicli call ALMotion.setAngles "HeadYaw" 0.7 0.3
```
- press ***space bar*** again to record the new position
- rotate Pepper's head to the right:
```
$ qicli call ALMotion.setAngles "HeadYaw" -0.7 0.3
```
- press ***space bar*** again to record the new position
- press ***"c" key*** to calibrate
- reset Pepper's head pitch and rotation:
```
$ qicli call ALMotion.setAngles "HeadPitch" 0.0 0.3
```
```
$ qicli call ALMotion.setAngles "HeadYaw" 0.0 0.3
```

## Running Individual Processes
The following terminal commands will launch the ROS software modules individually.
- Pepper ROS full stack
  ```
  $ roslaunch pepper_bringup pepper_full.launch nao_ip:=<pepper ip> network_interface:=<network interface>
  ```
  - example: 
  ```
  $ roslaunch pepper_bringup pepper_full.launch nao_ip:=10.1.1.202 network_interface:=enp3s0
  ```

- HoloLens ROS Bridge
  ```
  $ rosrun hololens_ros_bridge hololens_ros_bridge_node <hololens_ip> 1234
  ```
    - example: 
  ```
  $ rosrun hololens_ros_bridge hololens_ros_bridge_node 10.1.1.206 1234
  ```
- ROS map_server
  ```
  $ rosrun map_server map_server src/navigation_launcher/params/map.yaml
  ```
- HoloLens Anchor Localizer
  ```
  $ rosrun hololens_localizer anchor_localizer
  ```
- Localizer Calibration
  ```
  $ rosrun hololens_localizer static_calibration <robot odom frame> <robot head frame> <robot base link> [calibrationFileName]
  ```
    - example: 
  ```
  $ rosrun hololens_localizer static_calibration odom Head base_footprint calibrationData.bin
  ```
- Dynamic Adjuster
  ```
  $ rosrun hololens_localizer dynamic_adjuster.py <robot foot frame>
  ```
    - example: 
  ```
  $ rosrun hololens_localizer dynamic_adjuster.py
  ```

## RVIZ Configuration
- Map and Pepper RobotModel topics can be added manually:
  - Map

  - RobotModel 