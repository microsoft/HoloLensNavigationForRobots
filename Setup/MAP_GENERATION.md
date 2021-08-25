## ![logo](../img/MARR_logo.png) [Microsoft Applied Robotics Research Library](https://special-giggle-b26bab5f.pages.github.io/)
### Open Source Samples for Service Robotics
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) 

# Map Generation Instructions

## HoloLens Spatial Mapping

Use the HoloLens device to build a spatial mapping of your environment. Map the floor up to at least your eye level, 
and make sure to carefully trace along the floor edges to get accurate readings.

For visual feedback, compile and run the HoloLensNavigation application and complete the spatial mesh mapping. Alternatively, compile and run the lighter weight Microsoft's Holographic spatial mapping sample found 
[here](https://github.com/microsoft/Windows-universal-samples/tree/master/Samples/HolographicSpatialMapping).

## Create a floor plan image from HoloLens' Spatial Map in ROS

- launch the HoloLensNavigation app on your HoloLens device
    - there are different ways to launch the application. Easiest way is to use the Device Portal to launch 
application. Alternatively wear the headset and launch from GUI, or launch from VisualStudio.
- launch the ROS map_server
    - ```$ rosrun map_server map_server src/navigation_launcher/params/map.yaml```
- launch the HoloLens bridge in ROS
    - ```$ rosrun hololens_ros_bridge hololens_ros_bridge_node <hololens ip> 1234```
    - note that ```1234``` is the port number.
- launch the HoloLens localizer in ROS
    - ```$ rosrun hololens_localizer anchor_localizer```
- launch the image saver node in ROS
    - ```$ rosrun image_view image_saver image:=/hololens/image```
- launch rviz in ROS  
    - ```$ rviz```
    - select "2D Pose Estimate" and then click anywhere on the map view to set initial position in any location and 
  direction. This instructs image_saver to create a cross-section of the HoloLens' spatial map and save it as 
  ```left000.jpg``` in the same folder the image_saver was launched.
- open the file ```left0000.jpg```
    - open the file ```left0000.jpg``` with your favorite image editor.
    - using the depicted pointcloud outline, create a ROS compliant image map.
    - save the modified ```left0000.jpg``` file in the ```navigation_launcher/params/``` folder.
