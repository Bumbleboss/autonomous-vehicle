#ifndef FRONT_BOARD
#define FRONT_BOARD

/**********************************
 * DEFINE PINS
***********************************/

#define PEDAL_PIN A6
#define CAN_PIN 4

#define HORN_PIN 24
#define HORN_I2C 7

#define WARNING_I2C 5
#define WARNING_INTERVAL 600

#define LEFT_WARNING_PIN 22
#define LEFT_WARNING_I2C 4

#define RIGHT_WARNING_PIN 23
#define RIGHT_WARNING_I2C 3

#define HEADLIGHTS_PIN 21
#define HEADLIGHTS_I2C 6

#define ACC_LED 18
#define ACC_I2C 2

/**********************************
 * DEFINE DATA TYPES AND STRUCT
***********************************/

typedef unsigned int uint16;    // 65,535
typedef unsigned long uint32;   // 4,294,967,295

typedef enum {
  PEDAL_MODE,
  PID_MODE,
  AUTONOMOUS_MODE
} DRIVING_MODES;

struct can_frame can_msg_send;
struct can_frame can_msg_receive;

/**********************************
 * DEFINE FUNCTIONS
***********************************/

/**
 * pull-down switch functionality with debouncing
 * - SW_INPUT: switch input value
 * - SW_FLAG: previous state condition
 * - SW_VALUE: stored value that will be used for our computations 
*/
void pull_down_switch(bool *SW_INPUT, bool *SW_FLAG, bool *SW_VALUE);

/**
 * control the front and rear left/right leds in specific modes
*/
void led_controller();

#endif
