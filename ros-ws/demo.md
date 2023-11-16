# Guide to start a demo
1. `./set_ros_master_uri.sh` 
    - remember the ROS_MASTER_URI
2. On the firmware program, modify the `firmware/include/WiFiConfig.h` accordingly
    - IPAddress server(<ROS_MASTER_URI>);
    - Connect ESP32 to the same network with ROS, by setting the `SSIDNAME` and `WIFIPASSWORD`
3. `catkin_make` if needed
4. Connect a joystick to the ROS System
5. `roslaunch robot_bringup bringup.launch`
    - Happy demo!