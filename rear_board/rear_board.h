#ifndef REAR_BOARD
#define REAR_BOARD

/**********************************
 * DEFINE PINS
***********************************/

// bulbs
#define LEFT_PIN 16
#define RIGHT_PIN 19
#define BRAKES_PIN 17

#define COUNTER_PIN 3
#define CAN_PIN 10
#define MOTOR_PIN 9 

/**********************************
 * DEFINE DATA TYPES AND STRUCT
***********************************/

typedef unsigned char uint8;
typedef unsigned long uint32;

struct can_frame can_msg_send;
struct can_frame can_msg_receive;

/**********************************
 * DEFINE FUNCTIONS
***********************************/

/**
 * interrupt service routine for calculating pulses
*/
void int0ISR();

/**
 * refactored function for sending pwm signal to motor
*/
void set_motor_value(uint8 value);

/**
 * increment or decrement to the desired value based on old throttle value
*/
void set_throttle(uint8 value);

#endif
