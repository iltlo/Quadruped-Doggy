
#include "GoBLE.hpp"
#include "SerialBLE.hpp"
template class _GoBLE<HardwareSerial, HardwareSerial>;
template class _GoBLE<SerialBLE, HardwareSerial>;
/*
#include <SoftwareSerial.h>
template class _GoBLE<HardwareSerial, SoftwareSerial>;
template class _GoBLE<SoftwareSerial, SoftwareSerial>;
template class _GoBLE<SoftwareSerial, HardwareSerial>;
*/

template<typename T, typename T2>
_GoBLE<T, T2>::_GoBLE(T& blueTooth, T2& console):
  Console(console), BlueTooth(blueTooth),
  _joystickX(127), _joystickY(127) {

  initRecvDataPack();
  for (int i = 0; i < MAXBUTTONID+1; i++) {
    _button[i] = RELEASED;
  }
  for (int i = 0; i < 20; i++) bleQueue.push(0x00);
  for (int i = 0; i < 20; i++) bleQueue.pop();
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::available() {
  /*
    function introduction:
     push the new valid data to the data buffer package
     throw away the invalid byte
     parse the data package when the command length is matching the protocol
  */

  if (BlueTooth.available())  bleDataReceiver();

  if (DEBUGDATARAW) {
    Console.println("GoBLE availalbe -> new data package!");
    for (int i = 0; i < rDataPack.commandLength; i++) {
      if (!bleQueue.isEmpty()) {
        Console.print(bleQueue.pop(), HEX);
      }
    }
    Console.println();
  }

  if (DEBUGPARSER) {
    Console.print("GoBLE availalbe -> bleQueue Counter: ");
    Console.print(bleQueue.count());
    Console.println();
  }

  if (rDataPack.commandFlag && bleQueue.count() == rDataPack.commandLength) {
    rDataPack.parseState = bleDataPackageParser();
    if (rDataPack.parseState == PARSESUCCESS) {
      updateJoystickVal();
      updateButtonState();
      return true;
    }
  }

  return false;
}

template<typename T, typename T2>
int _GoBLE<T, T2>::readJoystickX() {
  return  _joystickX;
}

template<typename T, typename T2>
int _GoBLE<T, T2>::readJoystickY() {
  return  _joystickY;
}

template<typename T, typename T2>
int _GoBLE<T, T2>::readJoystickX2() {
  return  _joystickX2;
}

template<typename T, typename T2>
int _GoBLE<T, T2>::readJoystickY2() {
  return  _joystickY2;
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchUp() {
  return _button[SWITCH_UP];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchDown() {
  return _button[SWITCH_DOWN];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchLeft() {
  return _button[SWITCH_LEFT];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchRight() {
  return _button[SWITCH_RIGHT];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchSelect() {
  return _button[SWITCH_SELECT];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchStart() {
  return _button[SWITCH_START];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchAction() {
  return _button[SWITCH_ACTION];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchMid() {
  return _button[SWITCH_MID];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchPanLf() {
  return _button[SWITCH_PAN_LF];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchPanRt() {
  return _button[SWITCH_PAN_RT];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchTiltUp() {
  return _button[SWITCH_TILT_UP];
}

template<typename T, typename T2>
boolean _GoBLE<T, T2>::readSwitchTiltDn() {
  return _button[SWITCH_TILT_DN];
}


// Private functions
template<typename T, typename T2>
int _GoBLE<T, T2>::bleDataPackageParser() {
  /*
    0x10  - Parse success
    0x11  - Wrong header charactors
    0x12  - Wrong button number
    0x13  - Check Sum Error
  */
  byte calculateSum = 0;

  rDataPack.header1 = bleQueue.pop(), calculateSum +=  rDataPack.header1;
  rDataPack.header2 = bleQueue.pop(), calculateSum +=  rDataPack.header2;

  if (rDataPack.header1 != DEFAULTHEADER1)     return 0x11;
  if (rDataPack.header2 != DEFAULTHEADER2)     return 0x11;

  rDataPack.address = bleQueue.pop(), calculateSum +=  rDataPack.address;

  rDataPack.latestDigitalButtonNumber = rDataPack.digitalButtonNumber;
  rDataPack.digitalButtonNumber = bleQueue.pop(), calculateSum +=  rDataPack.digitalButtonNumber;

  int digitalButtonLength = rDataPack.digitalButtonNumber;


  if (DEBUGCHECKSUM) {
    Console.print("Parser -> digitalButtonLength: ");
    Console.println(digitalButtonLength);
  }
  if (digitalButtonLength > MAXBUTTONID)   return 0x12;

  rDataPack.joystickPosition = bleQueue.pop(), calculateSum +=  rDataPack.joystickPosition;

  // read button data package - dynamic button payload length
  for (int buttonPayloadPointer = 0; buttonPayloadPointer < digitalButtonLength; buttonPayloadPointer++) {
    byte buttonID = bleQueue.pop();
    if (DEBUGCHECKSUM) {
      Console.print("Parser -> receive buttion ID: ");
      Console.println(buttonID);
    }
    rDataPack.buttonPayload[buttonPayloadPointer] = buttonID;
    calculateSum +=  rDataPack.buttonPayload[buttonPayloadPointer];
  }
  // read 4 byte joystick data package
  for (int i = 0; i < 4; i++)  {
    rDataPack.joystickPayload[i] = bleQueue.pop();
    calculateSum +=  rDataPack.joystickPayload[i];
  }

  rDataPack.checkSum = bleQueue.pop();

  if (DEBUGCHECKSUM) {
    Console.print("Parser -> sum calculation: ");
    Console.println(calculateSum);

    Console.print("Parser -> checkSum byte value: ");
    Console.println(rDataPack.checkSum);
  }

  // check sum and update the parse state value
  // if the checksum byte is not correct, return 0x12
  rDataPack.commandFlag = false;
  if (rDataPack.checkSum == calculateSum)
    return PARSESUCCESS;
  else
    return 0x13;
}

template<typename T, typename T2>
void  _GoBLE<T, T2>::bleDataReceiver() {

  byte inputByte = BlueTooth.read();

  if (DEBUGDATARECEIVER) {
    Console.print("bleDataReceiver -> new data:");
    Console.println(inputByte, HEX);
  }

  // throw the trash data and restore the useful data to the queue buffer
  if (inputByte == DEFAULTHEADER1 || rDataPack.commandFlag == true) {
    bleQueue.push(inputByte);
    rDataPack.commandFlag = true;

    // auto adjust the command length based on the button command value
    if (bleQueue.count() == PACKBUTTONSIGN) {
      if (inputByte > 0 && inputByte < MAXBUTTONID + 1) {
        // default command length + button number
        rDataPack.commandLength = DEFAULTPACKLENGTH + inputByte;
        if (DEBUGDATARECEIVER)  BlueTooth.print("bleDataReceiver -> Command Length:"), BlueTooth.println(rDataPack.commandLength);
      }
      else  rDataPack.commandLength = DEFAULTPACKLENGTH;
    }
  }

}


template<typename T, typename T2>
void  _GoBLE<T, T2>::initRecvDataPack() {
  rDataPack.commandFlag         = false;
  rDataPack.commandLength       = DEFAULTPACKLENGTH;
  rDataPack.parseState          = PARSESUCCESS;

  rDataPack.digitalButtonNumber = 0;
  rDataPack.latestDigitalButtonNumber = 0;
}

template<typename T, typename T2>
void _GoBLE<T, T2>::updateJoystickVal() {
  _joystickY = rDataPack.joystickPayload[0];
  _joystickX = rDataPack.joystickPayload[1];
  _joystickY2 = rDataPack.joystickPayload[2];
  _joystickX2 = rDataPack.joystickPayload[3];

}

template<typename T, typename T2>
void _GoBLE<T, T2>::updateButtonState() {

  if (rDataPack.digitalButtonNumber == 0 && rDataPack.latestDigitalButtonNumber != 0) {
    for (int i = 0; i <= MAXBUTTONID; i++) {
      if (_button[i] == PRESSED) {
        if (DEBUGUPDATEBUTTON) {
          Console.print("updateButtonState -> clear Pressed button number: ");
          Console.println(i);
        }
        _button[i] = RELEASED;
      }
    }
  }

  for (int i = 0; i < rDataPack.digitalButtonNumber; i++)   {
    byte buttonID = rDataPack.buttonPayload[i];
    if (DEBUGUPDATEBUTTON) {
      Console.print("updateButtonState -> set Pressed button number ");
      Console.println(buttonID);
    }
    _button[rDataPack.buttonPayload[i]] = PRESSED;
  }
}
