# ROS driver for the RoMAA mobile robot

This package contains the driver node and launch files for the RoMAA robot.

## Driver node

Usage
```
rosrun romaa_ros romaa_driver
```

Example:
```
rosrun romaa_ros romaa_driver _port=/dev/ttyUSB1 _baud=57600
```

Paremeters:
 * `~port` (string, default: `/dev/ttyUSB0`): serial communication port device 
 * `~baud` (int, default: `115200`): serial communication port baudrate 

### Published topics
 * `~odom` (`nav_msgs/Odometry`): odometry messages 

### Subscribed topics
 * `~cmd_vel` (`geometry_msgs/Twist`): Command velocity messages 

### Services
 * `reset_odometry` (`std_srvs/Empty`): reset robot odometry
 * `set_odometry` (`romaa_ros/SetOdometry`): set robot odometry
 * `enable_motor` (`std_srvs(SetBool`): enable (`data: true`) or disable (`data: false`) motors


## Launch files

 * `romaa.launch`:
 * `joy_teleop.launch`:
 * `uvc_cam.launch`:

