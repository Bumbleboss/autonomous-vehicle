#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/UInt8.h>
#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);

uint16_t pedal;
uint16_t speed;
uint32_t distance;

uint8_t I2C_B1, I2C_B2;

bool BRAKING_SW;
bool RIGHT_WARNING_SW, RIGHT_WARNING_FLAG, RIGHT_WARNING_VAL;
bool LEFT_WARNING_SW, LEFT_WARNING_FLAG, LEFT_WARNING_VAL;
bool WARNING_SW, WARNING_FLAG, WARNING_VAL;
bool HEADLIGHTS_SW, HEADLIGHTS_FLAG, HEADLIGHTS_VAL;

bool HORN_ROS = 0;
bool HORN_SW;

bool CALIB_SW, CALIB_FLAG, CALIBRATION_MODE;
bool AUTO_SW, AUTO_FLAG, AUTONOMOUS_MODE;
bool CONS_SW, CONS_FLAG, CONST_SPEED_MODE;

uint32_t current_millis;
uint32_t previous_millis = 0;

uint32_t ros_previous_millis = 0;

AccelStepper stepper_controller(1, STEPPER_PIN, STEPPER_DIR_PIN);

bool CALIBRATE_INTERRUPT_FLAG = LOW;
bool LIMIT_SWITCH_FLAG = LOW;
CALIBRATION_PHASES calibration_phase;

uint16_t throttle_value = 0;
int16_t angle_value = 0;

std_msgs::UInt8 driving_mode;
std_msgs::UInt8 horn;

ros::NodeHandle node_handle;
ros::Subscriber<ackermann_msgs::AckermannDrive> ackermann_subscriber("/ackermann_cmd", &ackerman_callback);
ros::Subscriber<std_msgs::UInt8> horn_subscriber("/horn", &horn_callback);
ros::Publisher driving_mode_publisher("/driving_mode", &driving_mode);

void setup() {
  can_msg_send.can_id = 0x00;
  can_msg_send.can_dlc = 3;
  can_msg_send.data[0] = 0x00; // throttle
  can_msg_send.data[1] = 0x00; // throttle
  can_msg_send.data[2] = 0x00; // bulbs + Braking_SW

  Wire.begin(8); // join I2C bus with address #8
  Wire.onReceive(I2C_read); // register event

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  pinMode(RIGHT_WARNING_PIN, OUTPUT);
  pinMode(LEFT_WARNING_PIN, OUTPUT);
  pinMode(HEADLIGHTS_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(STEPPER_ENA_PIN, OUTPUT);

  // limit swtich pin definition and interrupt setup
  pinMode(STEERING_LIMIT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEERING_LIMIT_PIN), steering_limit_interrupt, FALLING);

  // set max speed for stepper to move on any condition
  stepper_controller.setMaxSpeed(STEPPER_SPEED);

  // setup rosserial
  node_handle.initNode();
  node_handle.subscribe(ackermann_subscriber);
  node_handle.subscribe(horn_subscriber);
  node_handle.advertise(driving_mode_publisher);
}

void loop() {
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    distance = ((uint32_t) can_msg_receive.data[0] & 0xFF) | ((uint32_t) (can_msg_receive.data[1] & 0xFF) << 8) | ((uint32_t) (can_msg_receive.data[2] & 0xFF) << 16) | ((uint32_t) (can_msg_receive.data[3] & 0xFF) << 24);
    speed = ((uint16_t) can_msg_receive.data[4] & 0xFF) | ((uint16_t) (can_msg_receive.data[5] & 0xFF) << 8);
  }
  
  current_millis = millis();

  // specific switches use pull-down logic
  pull_down_switch(&RIGHT_WARNING_SW, &RIGHT_WARNING_FLAG, &RIGHT_WARNING_VAL);
  pull_down_switch(&LEFT_WARNING_SW, &LEFT_WARNING_FLAG, &LEFT_WARNING_VAL);
  pull_down_switch(&WARNING_SW, &WARNING_FLAG, &WARNING_VAL);
  pull_down_switch(&HEADLIGHTS_SW, &HEADLIGHTS_FLAG, &HEADLIGHTS_VAL);

  pull_down_switch(&CALIB_SW, &CALIB_FLAG, &CALIBRATION_MODE);
  pull_down_switch(&AUTO_SW, &AUTO_FLAG, &AUTONOMOUS_MODE);
  pull_down_switch(&CONS_SW, &CONS_FLAG, &CONST_SPEED_MODE);

  digitalWrite(HEADLIGHTS_PIN, HEADLIGHTS_VAL);
  digitalWrite(HORN_PIN, HORN_ROS || HORN_SW);

  // enable switch led for it's relative mode
  digitalWrite(CALIB_LED, CALIBRATION_MODE);
  digitalWrite(AUTO_LED, AUTONOMOUS_MODE);
  digitalWrite(CONS_LED, CONST_SPEED_MODE);

  // disable stepper's holding torque when its not in calibration nor autonomous mode
  if (CALIBRATION_MODE || AUTONOMOUS_MODE) {
    digitalWrite(STEPPER_ENA_PIN, LOW);
  } else {
    digitalWrite(STEPPER_ENA_PIN, HIGH);
  }

  // braking switch  
  bitWrite(can_msg_send.data[2], 2, BRAKING_SW);
  warning_led_controller();

  // driving modes: calibration, autonomous, const speed
  // calibration mode active
  if (CALIBRATION_MODE && !AUTONOMOUS_MODE && !CONST_SPEED_MODE) {
    steering_calibration();
  
  // autonomous mode active
  } else if (!CALIBRATION_MODE && AUTONOMOUS_MODE && !CONST_SPEED_MODE) {
    // stop steering if limits are somehow reached
    if (LIMIT_SWITCH_FLAG == HIGH) {
      stepper_controller.stop();
      angle_value = stepper_controller.currentPosition();
      LIMIT_SWITCH_FLAG = LOW;
    } else {
      stepper_controller.moveTo(angle_value);
      stepper_controller.setSpeed(STEPPER_SPEED);     
      stepper_controller.runSpeedToPosition();

      // indicate the steering direction through warning led
      if (angle_value > 0) {
        RIGHT_WARNING_VAL = 0;
        LEFT_WARNING_VAL = 1;

      } else if (angle_value < 0) {
        RIGHT_WARNING_VAL = 1;
        LEFT_WARNING_VAL = 0;
      }

      // disable warning led if there's no new steering movement
      if (stepper_controller.distanceToGo() == 0) {
        RIGHT_WARNING_VAL = 0;
        LEFT_WARNING_VAL = 0;
      }
    }

    driving_mode.data = 2;

  // constant speed mode active
  } else if (!CALIBRATION_MODE && !AUTONOMOUS_MODE && CONST_SPEED_MODE) {
    throttle_value = 260;
    driving_mode.data = 3;

  // manual mode active
  } else if (!CALIBRATION_MODE && !AUTONOMOUS_MODE && !CONST_SPEED_MODE) {
    throttle_value = analogRead(PEDAL_PIN);
    driving_mode.data = 0;
  }
 
  // this code is added because of the buffer size that was modified in ros.h
  // we create a non-blocking delay to get rid of the mismatch error present in rosserial
  unsigned long ros_current_millis = millis();
  if (ros_current_millis - ros_previous_millis >= ROS_INTERVAL) {
    ros_previous_millis = ros_current_millis;

    driving_mode_publisher.publish(&driving_mode);
    node_handle.spinOnce();
  }

  can_msg_send.data[0] = (throttle_value >> 0) & 0xFF;
  can_msg_send.data[1] = (throttle_value >> 8) & 0xFF;

  mcp2515.sendMessage(&can_msg_send);
}

void I2C_read(int num) {
  I2C_B1 = Wire.read();
  I2C_B2 = Wire.read();

  // left board switches
  BRAKING_SW        = bitRead(I2C_B1, BRAKING_I2C);
  RIGHT_WARNING_SW  = bitRead(I2C_B1, RIGHT_WARNING_I2C);
  LEFT_WARNING_SW   = bitRead(I2C_B1, LEFT_WARNING_I2C);
  WARNING_SW        = bitRead(I2C_B1, WARNING_I2C);
  HEADLIGHTS_SW     = bitRead(I2C_B1, HEADLIGHTS_I2C);
  HORN_SW           = bitRead(I2C_B1, HORN_I2C);

  // right board switches
  AUTO_SW           = bitRead(I2C_B2, AUTO_I2C);
  CALIB_SW          = bitRead(I2C_B2, CALIB_I2C);
  CONS_SW           = bitRead(I2C_B2, CONS_I2C);
}

void pull_down_switch(bool *SW_INPUT, bool *SW_FLAG, bool *SW_VALUE) {
  if (*SW_INPUT == HIGH && *SW_FLAG == HIGH) {
    delay(50);

    if (*SW_INPUT == HIGH) {
      *SW_VALUE = (!*SW_VALUE);
      *SW_FLAG = LOW;

      // check if the switch is related to driving modes
      // if it is, and its being activated, deactivate other modes
      if (SW_VALUE == &CALIBRATION_MODE && *SW_VALUE == HIGH) {
        AUTONOMOUS_MODE = LOW;
        CONST_SPEED_MODE = LOW;
      } else if (SW_VALUE == &AUTONOMOUS_MODE && *SW_VALUE == HIGH) {
        CALIBRATION_MODE = LOW;
        CONST_SPEED_MODE = LOW;
      } else if (SW_VALUE == &CONST_SPEED_MODE && *SW_VALUE == HIGH) {
        CALIBRATION_MODE = LOW;
        AUTONOMOUS_MODE = LOW;
      }
    }
  }

  if (*SW_INPUT == 0) {
    *SW_FLAG = HIGH;
  }
}

void warning_led_controller() {
  // both left and right leds will be off
  if (!WARNING_VAL && !LEFT_WARNING_VAL && !RIGHT_WARNING_VAL) {
    digitalWrite(LEFT_WARNING_PIN, LOW);
    bitWrite(can_msg_send.data[2], 0, LOW);

    digitalWrite(RIGHT_WARNING_PIN, LOW);
    bitWrite(can_msg_send.data[2], 1, LOW);

    return;
  }

  bool left_warn = !digitalRead(LEFT_WARNING_PIN);
  bool right_warn = !digitalRead(RIGHT_WARNING_PIN);

  if (current_millis - previous_millis > WARNING_INTERVAL) {
    previous_millis = current_millis;

    // both left and right leds will blink
    if (WARNING_VAL) {
      digitalWrite(LEFT_WARNING_PIN, left_warn);
      bitWrite(can_msg_send.data[2], 0, left_warn);

      digitalWrite(RIGHT_WARNING_PIN, right_warn);
      bitWrite(can_msg_send.data[2], 1, right_warn);

    // left leds will blink
    } else if (!WARNING_VAL && LEFT_WARNING_VAL && !RIGHT_WARNING_VAL) {
      digitalWrite(LEFT_WARNING_PIN, left_warn);
      bitWrite(can_msg_send.data[2], 0, left_warn);

      digitalWrite(RIGHT_WARNING_PIN, LOW);
      bitWrite(can_msg_send.data[2], 1, LOW);

    // right leds will blink
    } else if (!WARNING_VAL && !LEFT_WARNING_VAL && RIGHT_WARNING_VAL) {
      digitalWrite(LEFT_WARNING_PIN, LOW);
      bitWrite(can_msg_send.data[2], 0, LOW);

      digitalWrite(RIGHT_WARNING_PIN, right_warn);
      bitWrite(can_msg_send.data[2], 1, right_warn);
    }
  }
}

void steering_calibration() {
  if (calibration_phase == CALIBRATE_END) {
    driving_mode.data = 4;
    return;
  }

  driving_mode.data = 1;

  switch(calibration_phase) {
    case (CALIBRATE_INTERRUPT):
      CALIBRATE_INTERRUPT_FLAG = HIGH;

      stepper_controller.stop();
      delay(2000);

      calibration_phase = CALIBRATE_RESET_POSITION;
      break;
    case (CALIBRATE_RESET_POSITION):
      stepper_controller.setCurrentPosition(0);
      calibration_phase = CALIBRATE_CENTER;
      break;
    case (CALIBRATE_CENTER):
      stepper_controller.moveTo(STEERING_CENTER);
      stepper_controller.setSpeed(STEPPER_SPEED);
      stepper_controller.runSpeedToPosition();

      if (stepper_controller.distanceToGo() == 0) {
        stepper_controller.setCurrentPosition(0);
        calibration_phase = CALIBRATE_END;
      }

      break;
    default:
      stepper_controller.setSpeed(-STEPPER_SPEED);
      stepper_controller.runSpeed();
      break;
  }
}

void steering_limit_interrupt() {
  // this is done to avoid triggering the interrupt phase multiple times
  if (!CALIBRATE_INTERRUPT_FLAG) {
    calibration_phase = CALIBRATE_INTERRUPT;
  }

  LIMIT_SWITCH_FLAG = HIGH;
}

void horn_callback(const std_msgs::UInt8& horn_bool) {
  HORN_ROS = horn_bool.data;
}

void ackerman_callback(const ackermann_msgs::AckermannDrive& ackerman_data) {
  // prevent any control on vehicle unless autonomous button is pressed
  if (!CALIBRATION_MODE && AUTONOMOUS_MODE && !CONST_SPEED_MODE) {
    angle_value = map(ackerman_data.steering_angle * 100, STEERING_MAX_ANGLE * -100, STEERING_MAX_ANGLE * 100, -1 * STEERING_MAX_STEPS, STEERING_MAX_STEPS);
    angle_value = constrain(angle_value, -1 * STEERING_MAX_STEPS, STEERING_MAX_STEPS);

    // car movement min value 230, max value 1024
    throttle_value = map(ackerman_data.speed * 1500, 0, 1000 * MAX_CAR_SPEED_MS, 230, 1024);
  }
}