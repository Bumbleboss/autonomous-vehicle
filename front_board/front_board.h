#ifndef FRONT_BOARD
#define FRONT_BOARD

/**********************************
 * DEFINE PINS
***********************************/

#define CAN_PIN 4
#define PEDAL_PIN A6

/**********************************
 * DEFINE DATA TYPES AND STRUCT
***********************************/

typedef unsigned char uint8;
typedef unsigned int  uint16;
typedef unsigned long uint32;

struct can_frame can_msg_send;
struct can_frame can_msg_receive;

/**********************************
 * DEFINE FUNCTIONS
***********************************/

#endif