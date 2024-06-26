/*
  Quadruped robot arduino sketch.
  This code is modification of https://github.com/maestrakos/quad by Alexandros
  Petkos Comment Description:

  /// comment

  //> used to explain the function of a line
  //: used to summurize the function of multiple lines

  === used for headers
  ::: used for sketch parts

  // ## used to explain the measurement unit of a variable
  // !! used for warnings
*/
#include <Arduino.h>

#include "Buzzer.hpp"
#include "Config.hpp"
#include "Hardware.hpp"
#include "Kinematics.hpp"

#include "SerialCommand.hpp"
#include <BluetoothHardware.h>  // for ros bluetooth connection

#include "WiFiConfig.h"         // for ros wifi connection password
#include <Wifi.h>

#include <ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
/*
  ==============================
  HARDWARE - control method
  ==============================
*/
//
#ifdef __GOBLE__
#include <SerialBLE.hpp>

#include "GoBLE.hpp"
SerialBLE serialBle;
_GoBLE<SerialBLE, HardwareSerial> Goble(serialBle, Console);
#endif  //__GOBLE__

#ifdef __SERIAL__
  SerialCommand<HardwareSerial, HardwareSerial> sCmd(CommandConsole, Console);
#endif  //__SERIAL__

Hardware hardware;
Kinematics kinematics(hardware);
float vo = kinematics.vrt_offset, ho = kinematics.hrz_offset;

Buzzer buzzer(BUZZER_GPIO_PIN);

//: those local variables control the step direction and period
datatypes::Vector2D _direction = {0, 0};
float turn = 0;    //> indicates the direction of rotation
float height = 0;  //> indicates the leg extension

int state = 0;         //> indicates the type of gait, (0) idle, (1) trot, (2) yaw, (3) pitch-roll
// float _period = 7.0;  //> indicates the number of steps every second
// float _period = 6.0;  //> indicates the number of steps every second
float _period = 4.0;  //> indicates the number of steps every second

datatypes::Rotator
    _sRotation;  //> this variable stores the relative rotation of the body

int8_t joystickLX = 0;
int8_t joystickLY = 0;
int8_t joystickRX = 0;
int8_t joystickRY = 0;

unsigned long duration;

//: handle input paramters
float stick_min = 5.f;
float lx, ly, rx, ry;

// IPAddress server(192,168,99,211); // if wifi
// const char*  ssid = "Robot-Operating-System-WiFi";
// const char*  password = "ros-workshop";
const char*  ssid = SSIDNAME;
const char*  password = WIFIPASSWORD;

#ifdef __ROS__
  //: ROS variables, and Publishers
  #ifdef __USE_WIFI__
    uint16_t serverPort = 11411;
    ros::NodeHandle nh;
  #endif //__USE_WIFI__
  #ifdef __USE_BT__
    ros::NodeHandle_<BluetoothHardware> nh;
  #endif //__USE_BT__
  
  std_msgs::String str_msg;
  ros::Publisher chatter("ESPchatter", &str_msg);

  uint16_t period = 10000;
  uint32_t last_time = 0;

  char hello[25] = "hello world from esp32!!";

  //: ROS callback functions, and Subscribers
  // float demandx = 0;
  // float demandy = 0;
  // float demandz = 0;
  volatile bool demandUpdate = false;
  void cmdVelCallback(const geometry_msgs::Twist &cmd_vel) {
    int joyMax = 127;
    // int joyMax = 80;
    float x = constrain(cmd_vel.linear.x, -1, 1) * joyMax;
    float y = constrain(cmd_vel.linear.y, -1, 1) * joyMax;
    float z = constrain(cmd_vel.angular.z, -1, 1) * joyMax;
    if (joystickLX != x) {
      demandUpdate = true;
      joystickLY = x;
    }
    if (joystickLY != y) {
      demandUpdate = true;
      joystickLX = y;
    }
    if (joystickRX != z) {
      demandUpdate = true;
      joystickRX = -z;
    }
    // Serial.print("joystick cmd_vel callback ");
    Serial.println((String)"x: " + x + " y: " + y + " z: " + z);
  }
  ros::Subscriber<geometry_msgs::Twist> cmdVelSubscriber("cmd_vel", cmdVelCallback);
#endif  //__ROS__


void aborted() {
  Console.println("{msg:program aborted!}");
  buzzer.beepError();
  while (1)
    ;
}

void handle_input() {
  lx = Kinematics::inter(lx, joystickLX / 4.f, 0.5f);  //> gets the interpolated x-position of the left  analog stick
  ly = Kinematics::inter(ly, joystickLY / 4.f, 0.5f);  //> gets the interpolated y-position of the left  analog stick
  rx = Kinematics::inter(rx, joystickRX / 4.f, 0.5f);  //> gets the interpolated x-position of the right analog stick
  ry = Kinematics::inter(ry, joystickRY / 4.f, 0.5f);  //> gets the interpolated y-position of the right analog stick
  if (abs(lx) > stick_min) {  //> checks whether the stick position is out of the deadzone
    float x0 = lx - stick_min * Kinematics::sign(lx);  //> subtracts the deadzone
    if (state == 1) {
      _direction.y = x0 / 10.f;
    } else if (state != 4) {
      _direction.y = x0 / 2;
    }
  } else
    _direction.y = 0;

  if (abs(ly) > stick_min) {  //> checks whether the stick position is out of the deadzone
    float y0 = ly - stick_min * Kinematics::sign(ly);  //> subtracts the deadzone
    if (state == 1) {
      _direction.x = y0 / 10.f;
      if (y0 > 0)
        kinematics.vrt_offset =
            Kinematics::inter(kinematics.vrt_offset, vo - 6.f, 2.f);
      else
        kinematics.vrt_offset =
            Kinematics::inter(kinematics.vrt_offset, vo + 3.f, 2.f);
    } else if (state != 4) {
      _direction.x = y0 / 2;
      kinematics.vrt_offset = vo;
    }
  } else {
    _direction.x = 0;
    kinematics.vrt_offset = vo;
  };

  if (abs(rx) > stick_min) {  //> checks whether the stick position is out of the deadzone
    float x1 = rx - stick_min * Kinematics::sign(rx);  //> subtracts the deadzone
    if (state == 1)
      turn = x1 / 16.f;
    else if (state != 4)
      turn = x1;
  } else
    turn = 0;

  if (abs(ry) > stick_min) {  //> checks whether the stick position is out of the deadzone
    float y1 = ry - stick_min * Kinematics::sign(ry);  //> subtracts the deadzone
    height = y1;
  } else
    height = 0;

  // #ifdef __PS2_GAMEPAD__
  //    if (ps2x.ButtonPressed(PSB_CIRCLE))
  //    {
  //      if (_tb == true)
  //      {
  //        _tb = false;
  //        state++;
  //        if (state > 4)
  //          state = 0;
  //        buzzer.beepShort();
  //        //Console.println("state: " + String(state));
  //      }
  //    }
  //    else
  //      _tb = true;
  // #endif
}

#ifdef __SERIAL__
  int handle_arg_int(char *args) {
    if (args == NULL) return -1;
    auto j = atoi(args);
    return j;
  }

  void handle_s_cmd(void) {
    bool validCmd = true;

    auto leg = handle_arg_int(sCmd.next());
    auto joint = handle_arg_int(sCmd.next());
    auto pw = handle_arg_int(sCmd.next());
    // CommandConsole.printf("%d %d %d\n", leg, joint, pw);
    if (leg < 0 || leg > 4 || joint < 0 || joint > 3 || pw == -1) {
      validCmd = false;
      goto __return;
    }
    hardware.set_max_pw_offset(leg, joint, pw);
  __return:
    if (validCmd)
      CommandConsole.println("ok");
    else
      CommandConsole.println("bad, usage: p leg joint pw");
  }

  void handle_p_cmd(void) {
    int leg, pw;

    for (auto leg = 1; leg <= 4; leg++) {
      for (auto joint = 1; joint <= 3; joint++) {
        pw = hardware.get_max_pw_offset(leg, joint);
        CommandConsole.printf("s %d %d %d\n", leg, joint, pw);
      }
    }
  }
  void handle_w_cmd(void) {}

  void handle_t_cmd(void) {
    state = state ? 0 : 1;
    joystickRX = 0;
    joystickLY = 70;
    CommandConsole.println("ok");
  }

  void unrecognized(const char *command) {
    CommandConsole.printf("Invalid command! [%s]", command);
  }
#endif  //__SERIAL__

void setupWiFi()  // if wifi
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.println((String)"SSID: " + WiFi.SSID() + " IP: " + WiFi.localIP());
}

void setup() {
  Console.begin(115200);
  CommandConsole.begin(38400);
  #ifdef __DEBUG__
    Console.println("{msg:in debugging mode}");
  #endif

  #ifdef __ROS__
    #ifdef __USE_WIFI__
      setupWiFi();  // if wifi
      nh.getHardware()->setConnection(server, serverPort);
      nh.initNode();
      // Another way to get IP
      Serial.println((String)"ROS IP = " + nh.getHardware()->getLocalIP());
    #endif //__USE_WIFI__
    #ifdef __USE_BT__
      nh.initNode();
    #endif //__USE_BT__

    nh.subscribe(cmdVelSubscriber);
    nh.advertise(chatter);
    
    Serial.println("rosserial with wifi/bluetooth");
  #endif  //__ROS__
    //
  #ifdef __GOBLE__  
    serialBle.begin();
    Goble.begin();
  #endif //__GOBLE__
  //

  hardware.init_hardware();
  //: servo calibration mode - while PIN 13 connects to 3.3V, all servos in 90°
  //: for servo arm adjustment °
  pinMode(SERVO_CAL_GPIO_PIN, INPUT_PULLDOWN);
  while (digitalRead(SERVO_CAL_GPIO_PIN)) {
    Console.println("{msg:calibrating}");
    delay(1000);
  }
  pinMode(SERVO_POWER_OFF_GPIO_PIN, INPUT_PULLUP);
  //

  #ifdef __SERIAL__
    sCmd.addCommand("s", handle_s_cmd);
    sCmd.addCommand("p", handle_p_cmd);
    sCmd.addCommand("w", handle_w_cmd);
    sCmd.addCommand("t", handle_t_cmd);
    sCmd.setDefaultHandler(unrecognized);
  #endif  //__SERIAL__

  buzzer.beepShort();
  Console.println("{msg:started}");
  CommandConsole.println("{msg:started}");
}

void test_freq() {
  // this code gets the frequency of the loop function
  static long sample_sum = 0, sample_index = 0;
  const long sample_num = 1000;
  duration = millis();
  sample_sum += 1000.0 / (millis() - duration);
  sample_index++;

  if (sample_index > sample_num) {
    long freq = sample_sum / sample_num;
    Console.println("{test:frequency=" + String(freq) + "}");
    sample_sum = 0;
    sample_index = 0;
  }
}

void loop() {
  // hardware.testGPIOservos();  return;
  // test_freq(); return;
  //
  duration = millis();
  // hardware.handle_hardware();
  kinematics.handle_kinematics(state, _direction, turn, height, _period);
  //
  //: test mode -  while PIN 13 connects to 3.3V again, will walk in trot gait
  static bool testMode = false;
  static long checkDuration = 0;
  if ((duration - checkDuration) > 1000) {  // check every 1 sec
    checkDuration = duration;
    if (digitalRead(SERVO_CAL_GPIO_PIN)) {
      CommandConsole.println("{msg:auto walking mode}");
      hardware.attachServos();
      // joystickLY = 127;
      joystickLY = 65;
      // joystickRX = 127;
      joystickRX = 0;
      Serial.println((String)"Auto Walk > joystickLY: " + joystickLY + " joystickRX: " + joystickRX);
      state = 1;
      testMode = true;
      goto __handle_input;
    } else if (testMode) {
      joystickLX = 0;
      joystickLY = 0;
      joystickRX = 0;
      joystickRY = 0;
      testMode = false;
      // goto __handle_input;
    }
  }

  #ifdef __ROS__
    state = 1;
    if(millis() - last_time >= period)  // check if ros is connected every <period> sec
    {
      last_time = millis();
      if (nh.connected())
      {
        Serial.println("Connected");
        // Say hello
        str_msg.data = hello;
        chatter.publish( &str_msg );
      } else {
        Serial.println("Not Connected");
      }
    }
    // nh.spinOnce();
    // delay(1);

    nh.spinOnce();
    // Serial.println((String)"LOOP > joystickLX: "+joystickLX+" joystickLY: "+joystickLY+" joystickRX: "+joystickRX);

    // delay(100);
    goto __handle_input;
  #endif  //__ROS__


  #ifdef __GOBLE__
    static long previousDuration = 0;
    static unsigned long timeout = 0;
    if (millis() > timeout && !digitalRead(SERVO_POWER_OFF_GPIO_PIN)) {
      // CommandConsole.println("{msg:servo power saving mode}");
      if ((duration - previousDuration) > 600000) {   // 10 min
        previousDuration = duration;
        hardware.detachServos();  // turn off servos while not moving for 1 min
        joystickLX = 0;
        joystickLY = 0;
        joystickRX = 0;
        joystickRY = 0;
        buzzer.beepShort();
        CommandConsole.println("{msg:stopped servos for power saving}");
        return;
      }
      timeout = millis() + 200;
    }

    if (Goble.available()) {
      previousDuration = duration;
      hardware.attachServos();  // turn on servos if they are off
      switch (state) {
        case 1:
          joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
          joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
          break;
        case 2:
          joystickRY = map(Goble.readJoystickY(), 255, 0, 127, -128);
          joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
          break;
        case 3:
          joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
          joystickLX = map(Goble.readJoystickX(), 0, 255, 127, -128);
          break;
        case 0:
        default:
          joystickLX = 0;
          joystickLY = 0;
          joystickRX = 0;
          joystickRY = 0;
      }

      if (Goble.readSwitchUp() == PRESSED) {
        state = 0;
        buzzer.beepShort();
      } else if (Goble.readSwitchDown() == PRESSED) {
        state = 2;
        buzzer.beepShort();
      } else if (Goble.readSwitchLeft() == PRESSED) {
        state = 3;
        buzzer.beepShort();
      } else if (Goble.readSwitchRight() == PRESSED) {
        state = 1;
        buzzer.beepShort();
      }

      if (Goble.readSwitchMid() == PRESSED) {
        buzzer.beepShort();
      } else if (Goble.readSwitchSelect() == PRESSED) {
        buzzer.beepShort();
      } else if (Goble.readSwitchAction() == PRESSED) {
        buzzer.beepShort();
      } else if (Goble.readSwitchStart() == PRESSED) {
        buzzer.beepShort();
        hardware.detachServos();
        joystickLX = 0;
        joystickLY = 0;
        joystickRX = 0;
        joystickRY = 0;
        previousDuration = duration;
      }
    }
  #endif  //__GOBLE__
  //
  __handle_input:
  handle_input();

  #ifdef __SERIAL__
    sCmd.readSerial();
  #endif  //__SERIAL__
}
