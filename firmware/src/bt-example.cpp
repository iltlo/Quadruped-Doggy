// // #include <M5Stack.h>
// #include <ros/node_handle.h>
// #include <BluetoothHardware.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Twist.h>

// ros::NodeHandle_<BluetoothHardware> nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("ESPchatter", &str_msg);


// float demandx = 0;
// float demandy = 0;
// float demandz = 0;
// volatile bool demandUpdate = false;
// void cmdVelCallback(const geometry_msgs::Twist &cmd_vel) {
//   float x = constrain(cmd_vel.linear.x, -1, 1) * 127;
//   float y = constrain(cmd_vel.linear.y, -1, 1) * 127;
//   float z = constrain(cmd_vel.angular.z, -1, 1) * 127;
//   if (demandx != x) {
//     demandUpdate = true;
//     demandx = x;
//   }
//   if (demandy != y) {
//     demandUpdate = true;
//     demandx = x;
//   }
//   if (demandz != z) {
//     demandUpdate = true;
//     demandz = z;
//   }
//   Serial.print("x: ");
//   Serial.print(x);
//   Serial.print(" y: ");
//   Serial.print(y);
//   Serial.print(" z: ");
//   Serial.println(z);
// }
// ros::Subscriber<geometry_msgs::Twist> cmdVelSubscriber("cmd_vel", cmdVelCallback);

// char hello[19] = "hello from esp32!!";

// void setup()
// {
//   Serial.begin(115200);
//   nh.initNode();
//   nh.subscribe(cmdVelSubscriber);
//   nh.advertise(chatter);

//   Serial.print("rosserial with bluetooth\n");

//   while (not nh.connected()) {
//       nh.spinOnce();
//       delay(100);
//   }
//   Serial.print("Bluetooth connected.");
// }

// void loop()
// {
//   str_msg.data = hello;
//   chatter.publish( &str_msg );
//   nh.spinOnce();
//   delay(100);
// }