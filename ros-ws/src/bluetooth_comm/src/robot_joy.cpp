#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

geometry_msgs::Twist twist;

bool update = false;
void joyStateCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (msg->axes[1] != twist.linear.x) {
    twist.linear.x = msg->axes[1];
    update = true;
  }
  if (msg->axes[0] != twist.linear.y) {
    twist.linear.y = msg->axes[0];
    update = true;
  }
  if (msg->axes[2] != twist.angular.z) {
    twist.angular.z = msg->axes[2];
    update = true;
  }
  //ROS_INFO("joyStateCallback() called %f %f", twist.linear.x,twist.angular.z);
}

void espChatterCallback(const std_msgs::String::ConstPtr& msg) { 
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_joy");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub =
      nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber joy_sub =
      nh.subscribe("/joy", 1000, joyStateCallback);
  ros::Subscriber esp_sub = 
      nh.subscribe("/ESPchatter", 1000, espChatterCallback);

  // ros::Rate loop_rate(5);
  // while (ros::ok()) {
  //   if (true) {
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    if (update) {
      ROS_INFO("publish twist");
      cmd_pub.publish(twist);
      update = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
