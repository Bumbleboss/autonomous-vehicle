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

// the x.0 is for division computation
#define TICKS_PER_METER 47.0

/**********************************
 * DEFINE CAN STRUCT
***********************************/

struct can_frame can_msg_send;
struct can_frame can_msg_receive;

/**********************************
 * DEFINE FUNCTIONS
***********************************/

/**
 * interrupt service routine for calculating speed using pulses
*/
void calculate_speed();

/**
 * refactored function for sending pwm signal to motor
*/
void set_motor_value(uint16_t value);

/**
 * increment or decrement to the desired value based on old throttle value
*/
void set_throttle(uint16_t value);

#endif
