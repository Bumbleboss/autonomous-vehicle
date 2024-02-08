#ifndef FRONT_BOARD
#define FRONT_BOARD

/**********************************
 * DEFINE PINS
***********************************/

// bulbs
#define LEFT_PIN 22
#define RIGHT_PIN 23

#define CAN_PIN 4
#define PEDAL_PIN A6

#define ACC_LED 18

/**********************************
 * DEFINE DATA TYPES AND STRUCT
***********************************/

typedef unsigned char uint8;    // 255
typedef unsigned long uint32;   // 4,294,967,295

typedef enum {
  PEDAL_MODE,
  PID_MODE,
  AUTONOMOUS_MODE
} CAR_MODES;

struct can_frame can_msg_send;
struct can_frame can_msg_receive;

/**********************************
 * DEFINE FUNCTIONS
***********************************/

#endif
