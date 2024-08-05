# Embedded System

### Rear Board Specifications
Board: ATmega328 (Arduino UNO)
Libraries required:
- MCP2515: https://github.com/autowp/arduino-mcp2515
- PWM: https://github.com/terryjmyers/PWM

### Front Board Specifications
Board: ATmega32 ([MightyCore](https://github.com/MCUdude/MightyCore))
Libraries required:
- MCP2515: https://github.com/autowp/arduino-mcp2515
- AccelStepper: https://github.com/waspinator/AccelStepper
- ROS (check guide below)

# Interfacing with rosserial
Our front board uses a lot of dynamic memory, due to that, we modified rosserial files to decrease the usage.

To accomplish the same task, the following must be done.

- Make sure you have installed rosserial
  ```bash
  sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
  sudo apt-get install ros-${ROS_DISTRO}-rosserial
  ```
- Run the following command at your Arduino directory (ex: ` ~/Arduino/libraries`): <br/>
  ```bash
  rosrun rosserial_arduino make_libraries.py
  ```
- Change directory to ROS library and modify the file `ros.h` to:
  ```c
  #ifndef _ROS_H_
  #define _ROS_H_

  #include "ros/node_handle.h"

  #if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
    #include "ArduinoTcpHardware.h"
  #else
    #include "ArduinoHardware.h"
  #endif

  namespace ros
  {
    typedef NodeHandle_<ArduinoHardware, 2, 2, 280, 280> NodeHandle;
  }

  #endif
  ```
- Change the `baud_` value at line `81` in `ArduinoHardware.h` to `115200`

After completing the above steps, interfacing with front board should be seamless. 

```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200
```

Dont forget to change port if the above isn't working!