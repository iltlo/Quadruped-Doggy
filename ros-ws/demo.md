# Guide to start a demo
## Wifi Connection
1. `./set_ros_master_uri.sh` 
    - remember the ROS_MASTER_URI
2. On the firmware program, modify the `firmware/include/WiFiConfig.h` accordingly
    - IPAddress server(<ROS_MASTER_URI>);
    - Connect ESP32 to the same network with ROS, by setting the `SSIDNAME` and `WIFIPASSWORD`
## Bluetooth Connection
1. `sudo rfcomm bind 1 <MAC address of your device>` (`rfcomm release 1` if needed)
## After connection config 
1. `catkin_make` if needed
2. Connect a joystick to the ROS System
3. `roslaunch robot_bringup bringup.launch`
    - Happy demo!