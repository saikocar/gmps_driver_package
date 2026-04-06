## Overview
The **GMPS Driver** is a ROS2 Driver for [GMPS sensor](https://www.aichi-steel.co.jp/smart/mi/gmps/).
 
`GMPS sensor` <--(CAN)--> `USB-CAN I/F` <--(socketCAN)--> `ros2_socketcan` <--(can_msgs::Frame)--> `gmps_driver`

## Requirements
- Host device
  - Linux OS: Ubuntu 22.04
  - ROS2: humble
- Other packages
  - [gmps_msgs](https://github.com/aichisteel-gmps/gmps_msgs)
  - [can_msgs](https://github.com/ros-industrial/ros_canopen/tree/dashing)
  - [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan)
  
## Launch
```bash
# enable socketCAN  
$ sudo modprobe can_dev
$ sudo modprobe can 
$ sudo modprobe can_raw 
$ sudo ip link set can0 type can bitrate 500000
$ sudo ip link set up can0

# launch ros2_socketcan   
$ ros2 launch ros2_socketcan socket_can_bridge.launch.xml

# launch gmps_driver  
$ ros2 launch gmps_driver gmps_driver
```

 


## Nodes
### Subscribed Topics
- in_gmps_can_frame (can_msgs/Frame)  
 CAN messages from CAN bus.

- in_velocity (geometry_msgs/TwistWithCovarianceStamped)  
  Vehicle speed.

- in_soft_reset (std_msgs/Bool)  
  Reboot GMPS sensor.

### Published Topics
- out_gmps_can_frame (can_msgs/Frame)  
 CAN messages to CAN bus.

- out_gmps_detect (gmps_msgs/GmpsDetect)  
  GMPS sensor ouput, which means lateral deviation from detected magnetic marker.

- out_gmps_error (gmps_msgs/GmpsError)  
  GMPS sensor error.

## Parameter description

The parameters are set in `launch/gmps_driver.launch.xml` .

| Name                          | Type   | Description                                                   | 
| :---------------------------- | :----- | :------------------------------------------------------------ | 
| offset_search_mode | bool | Launch this node for sensor calibration. |
| delay_dist | double | Delay distance from the center of a magnetic marker [m] |
| watchdog_timeout| double | Threshold for watchdog timer [s] |

