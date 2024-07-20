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

// bottom-last-right
#define CALIB_LED 18
#define CALIB_I2C 2

// bottom-mid
#define AUTO_LED 19
#define AUTO_I2C 1

// bottom-last-left
#define CONS_LED 15
#define CONS_I2C 3


// stepper motor (steering) config
#define STEPPER_PIN 14
#define STEPPER_DIR_PIN 13
#define STEPPER_ENA_PIN 12
#define STEERING_LIMIT_PIN 11

/**********************************
 * DEFINE FIXED VALUES
***********************************/

#define STEPPER_SPEED -3500
#define STEPPER_REVOLUTION 1000 

#define STEERING_CENTER -10000
#define STEERING_MAX_STEPS 6900
#define STEERING_MAX_ANGLE 0.698132329

// max car speed in m/s
#define MAX_CAR_SPEED_MS 5

/**********************************
 * DEFINE TYPES AND STRUCTS
***********************************/

typedef enum {
  CALIBRATE_INTERRUPT = 1,
  CALIBRATE_RESET_POSITION,
  CALIBRATE_CENTER,
  CALIBRATE_END
} CALIBRATION_PHASES;

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
 * 
 * It also contains a custom snippet for deactivating other driving modes when a new one is set to be activated
*/
void pull_down_switch(bool *SW_INPUT, bool *SW_FLAG, bool *SW_VALUE);

/**
 * control the front and rear left/right leds in specific modes
*/
void warning_led_controller();

/**
 * calibrate steering motor to its center
 * 1. keep turning the stepper until an interrupt from limit switch happens
 * 2. stop the motor and wait 2 seconds
 * 3. reset the stepper position to the limit switch position
 * 4. reverse the steering direction to the center (the center is based on practical experiment)
 * 5. once the distance to the center is achieved, break from the calibration function entirely
*/
void steering_calibration();

/**
 * method called when limit switches for the steering is triggered
*/
void steering_limit_interrupt();

void ackerman_callback(const ackermann_msgs::AckermannDrive& ackermann_variable);
void horn_callback(const std_msgs::UInt8& horn_bool);

#endif