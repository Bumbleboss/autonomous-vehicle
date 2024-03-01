#ifndef FRONT_BOARD
#define FRONT_BOARD

/**********************************
 * DEFINE PINS
***********************************/

#define CAN_PIN 4
#define PEDAL_PIN A6
#define HORN_PIN 24

#define ACC_LED 18

#define HORN_I2C 7
#define WARNING_I2C 5
#define LEFT_WARNING_I2C 3
#define RIGHT_WARNING_I2C 4
#define HEADLIGHTS_I2C 6

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
 * debounce a switch using three different variables
 * - SW_INPUT: switch input value
 * - SW_FLAG: fallback condition flag
 * - SW_VALUE: the filtered value to check state of condition
*/
void debounce_switch(bool *SW_INPUT, bool *SW_FLAG, bool *SW_VALUE);

#endif
