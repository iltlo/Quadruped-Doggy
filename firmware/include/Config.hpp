#ifndef __CONFIG__
#define __CONFIG__

//
#define __DEBUG__
//
#define Console Serial
#define CommandConsole Serial2

// select the way of control
// #define __SERIAL__
// #define __GOBLE__
#define __ROS__
#define __USE_WIFI__
// #define __USE_BT__

#define BUZZER_GPIO_PIN 23
#define SERVO_CAL_GPIO_PIN 13
#define SERVO_POWER_OFF_GPIO_PIN 34

#define __IO_BOARD_HERKULES__
#endif //__CONFIG_
