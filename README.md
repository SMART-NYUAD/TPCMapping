# TPC Mapping
This package provdies pan-tilt and camera integration along with the thermal mapping capabilities for the SUMMIT-XL Robot. 

## PTU-Control
Files for controlling FLIR Pan Tilt Unit with the FLIR A65 Infrared Camera. Tested on Ubuntu 18.04 with ROS Melodic. 

### Installation
The following dependencies are needed for this node to work:


**For the PTU**
- [flir_ptu_ethernet](https://github.com/RobotnikAutomation/flir_ptu_ethernet)
    - [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs/)
    - [rcomponent](https://github.com/RobotnikAutomation/rcomponent/)
    - [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors/)
- [flir_ptu](https://github.com/ros-drivers/flir_ptu)

**For the Camera**
- [Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/)
- [FLIR_camera](https://github.com/SMART-NYUAD/FLIR_camera/tree/kinetic-devel)
- [pointgrey_camera_driver](https://github.com/ros-drivers/pointgrey_camera_driver.git) (only image_exposure_msgs, statistics_msgs, and wfov_camera_msgs are specifically needed from this. **Recommended** to delete pointgrey_camera_description and driver to prevent errors)

**IP Addresses**

Make sure that the network is setup with a subnet mask of **255.255.255.0** with a network IP of **192.168.50.0**

- **PTU: 192.168.0.21**
- **FLIR Camera: 192.168.0.22**

### Usage: 

To launch with both the pan tilt and camera nodes: 
```
roslaunch ptu_control ptu_control.launch
```

To launch with just the pan-tilt node:
```
roslaunch ptu_control pan_tilt.launch
```

To launch with just the camera node:
```
roslaunch ptu_control camera.launch
```

All above launch files will initiate the Graphical User Interface for operating the relevant nodes, as seen below:  
<p align="center">
    <img src="figures/GUI.png"/>
</p>

Commands for the pan tilt unit can be placed through the service caller via the **/pan_tilt** service. Alternatively, the service can be called directly through a command-line terminal via: 
```
rosservice call /pan_tilt "pan: 0.0
tilt: 0.0
max_pan_speed: 0.0
max_tilt_speed: 0.0" 
```
