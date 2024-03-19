#include <SPI.h>
#include <mcp2515.h>
#include <ArduPID.h>
#include <Wire.h>
#include <AccelStepper.h>
#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);

uint16 pedal;
uint32 displacement;
uint32 speed;

ArduPID pid_controller;

byte I2C_B1, I2C_B2;

bool HORN_SW;
bool WARNING_SW, WARNING_FLAG, WARNING_VAL;
bool LEFT_WARNING_SW, LEFT_WARNING_FLAG, LEFT_WARNING_VAL;
bool RIGHT_WARNING_SW, RIGHT_WARNING_FLAG, RIGHT_WARNING_VAL;
bool HEADLIGHTS_SW, HEADLIGHTS_FLAG, HEADLIGHTS_VAL;

bool PID_SW, PID_FLAG, PID_VAL;
bool AUTO_SW, AUTO_FLAG, AUTO_VAL;
bool CONS_SW, CONS_FLAG, CONS_VAL;

uint32 current_millis;
uint32 previous_millis = 0;

DRIVING_MODES driving_mode;
CALIBRATION_PHASES calibration_phase;

AccelStepper stepper_controller(1, STEPPER_PIN, STEPPER_DIR_PIN);

// PID speed parameters
double pid_input;
double pid_desired;
double pid_output;

uint16 throttle_value;

void setup() {
  can_msg_send.can_id = 0x00;
  can_msg_send.can_dlc = 3;
  can_msg_send.data[0] = 0x00; // throttle
  can_msg_send.data[1] = 0x00; // throttle
  can_msg_send.data[2] = 0x00; // bulbs

  pid_controller.begin(&pid_input, &pid_output, &pid_desired, 160 , 0.005 , 0.005);
  pid_controller.setOutputLimits(0, 1024);

  Wire.begin(8); // join I2C bus with address #8
  Wire.onReceive(I2C_Read); // register event

  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  pinMode(HORN_PIN, OUTPUT);
  pinMode(LEFT_WARNING_PIN, OUTPUT);
  pinMode(RIGHT_WARNING_PIN, OUTPUT);
  pinMode(HEADLIGHTS_PIN, OUTPUT);
  pinMode(STEPPER_ENA_PIN, OUTPUT);

  // limit swtich pin definition and interrupt setup
  pinMode(STEERING_LIMIT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEERING_LIMIT_PIN), steering_limit_interrupt, FALLING);

  // set max speed for stepper to move on any condition
  stepper_controller.setMaxSpeed(STEPPER_SPEED);
}

void loop() {
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    displacement = (can_msg_receive.data[0] & 0xFF) | ((can_msg_receive.data[1] & 0xFF) << 8) | ((can_msg_receive.data[2] & 0xFF) << 16) | ((can_msg_receive.data[3] & 0xFF) << 24);
    speed = (can_msg_receive.data[4] & 0xFF) | ((can_msg_receive.data[5] & 0xFF) << 8) | ((can_msg_receive.data[6] & 0xFF) << 16) | ((can_msg_receive.data[7] & 0xFF) << 24);
  }
  
  current_millis = millis();

  // map pedal resolution to motor
  pedal = analogRead(PEDAL_PIN);

  // convert speed to appropriate m/s double
  pid_input = speed / 100.0;

  // specific switches use pull-down logic
  pull_down_switch(&WARNING_SW, &WARNING_FLAG, &WARNING_VAL);
  pull_down_switch(&LEFT_WARNING_SW, &LEFT_WARNING_FLAG, &LEFT_WARNING_VAL);
  pull_down_switch(&RIGHT_WARNING_SW, &RIGHT_WARNING_FLAG, &RIGHT_WARNING_VAL);
  pull_down_switch(&HEADLIGHTS_SW, &HEADLIGHTS_FLAG, &HEADLIGHTS_VAL);

  pull_down_switch(&PID_SW, &PID_FLAG, &PID_VAL);
  pull_down_switch(&AUTO_SW, &AUTO_FLAG, &AUTO_VAL);
  pull_down_switch(&CONS_SW, &CONS_FLAG, &CONS_VAL);

  digitalWrite(HORN_PIN, HORN_SW);
  digitalWrite(HEADLIGHTS_PIN, HEADLIGHTS_VAL);

  // enable switch led for it's relative mode
  digitalWrite(AUTO_LED, driving_mode == AUTONOMOUS_MODE);
  digitalWrite(PID_LED, driving_mode == PID_MODE);
  digitalWrite(CONS_LED, driving_mode == CONST_SPEED_MODE);

  // disable stepper's holding torque when its not autonomous mode
  digitalWrite(STEPPER_ENA_PIN, driving_mode != AUTONOMOUS_MODE);

  warning_led_controller();
  
  if (PID_VAL) {
    driving_mode = PID_MODE;

    AUTO_VAL = 0;
    CONS_VAL = 0;
  } else if (CONS_VAL) {
    driving_mode = CONST_SPEED_MODE;

    PID_VAL = 0;
    AUTO_VAL = 0;
  } else if (AUTO_VAL) {
    driving_mode = AUTONOMOUS_MODE;

    PID_VAL = 0;
    CONS_VAL = 0;
  } else {
    driving_mode = MANUAL_MODE;

    PID_VAL = 0;
    AUTO_VAL = 0;
    CONS_VAL = 0;
  }

  switch (driving_mode) {
    case (MANUAL_MODE):
      throttle_value = pedal;
      break;
    case (PID_MODE):
      // TODO: replace with variable input
      pid_desired = 1;

      pid_controller.compute();
      // throttle_value = pid_output;
      throttle_value = 0;
      break;
    case (CONST_SPEED_MODE):
      // throttle_value = 260;
      throttle_value = 0;
      break;

    case (AUTONOMOUS_MODE):
      steering_calibration();
      break;
    default:
      throttle_value = pedal;
      break;
  }
  
  can_msg_send.data[0] = (throttle_value >> 0) & 0xFF;
  can_msg_send.data[1] = (throttle_value >> 8) & 0xFF;

  mcp2515.sendMessage(&can_msg_send);
}

void I2C_Read(int how_many) {
  I2C_B1 = Wire.read();
  I2C_B2 = Wire.read();

  // left board switches
  HORN_SW           = bitRead(I2C_B1, HORN_I2C);
  WARNING_SW        = bitRead(I2C_B1, WARNING_I2C);
  LEFT_WARNING_SW   = bitRead(I2C_B1, LEFT_WARNING_I2C);
  RIGHT_WARNING_SW  = bitRead(I2C_B1, RIGHT_WARNING_I2C);
  HEADLIGHTS_SW     = bitRead(I2C_B1, HEADLIGHTS_I2C);

  // right board switches
  AUTO_SW            = bitRead(I2C_B2, AUTO_I2C);
  PID_SW            = bitRead(I2C_B2, PID_I2C);
  CONS_SW           = bitRead(I2C_B2, CONS_I2C);
}

void pull_down_switch(bool *SW_INPUT, bool *SW_FLAG, bool *SW_VALUE) {
  if (*SW_INPUT == 1 && *SW_FLAG == 1) {
    delay(50);

    if (*SW_INPUT == 1) {
      *SW_VALUE = (!*SW_VALUE);
      *SW_FLAG = 0;
    }
  }

  if (*SW_INPUT == 0) {
    *SW_FLAG = 1;
  }
}

void warning_led_controller() {
  // both left and right leds will be off
  if (!WARNING_VAL && !LEFT_WARNING_VAL && !RIGHT_WARNING_VAL) {
    digitalWrite(LEFT_WARNING_PIN, 0);
    bitWrite(can_msg_send.data[2], 0, 0);

    digitalWrite(RIGHT_WARNING_PIN, 0);
    bitWrite(can_msg_send.data[2], 1, 0);

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

      digitalWrite(RIGHT_WARNING_PIN, 0);
      bitWrite(can_msg_send.data[2], 1, 0);

    // right leds will blink
    } else if (!WARNING_VAL && !LEFT_WARNING_VAL && RIGHT_WARNING_VAL) {
      digitalWrite(LEFT_WARNING_PIN, 0);
      bitWrite(can_msg_send.data[2], 0, 0);

      digitalWrite(RIGHT_WARNING_PIN, right_warn);
      bitWrite(can_msg_send.data[2], 1, right_warn);
    }
  }
}

void steering_calibration() {
  if (calibration_phase == CALIBRATE_END) {
    return;
  }

  switch(calibration_phase) {
    case (CALIBRATE_INTERRUPT):
      stepper_controller.stop();
      delay(2000);

      calibration_phase = CALIBRATE_RESET_POSITION;
      break;
    case (CALIBRATE_RESET_POSITION):
      stepper_controller.setCurrentPosition(0);

      calibration_phase = CALIBRATE_CENTER;
      break;
    case (CALIBRATE_CENTER):
      stepper_controller.moveTo(-1 * STEPPER_STEERING_CENTER);
      stepper_controller.setSpeed(STEPPER_SPEED);
      stepper_controller.runSpeedToPosition();

      if (stepper_controller.distanceToGo() == 0) {
        stepper_controller.setCurrentPosition(0);
        calibration_phase = CALIBRATE_END;
      }

      break;
    default:
      stepper_controller.setSpeed(STEPPER_SPEED);
      stepper_controller.runSpeed();
      break;
  }
}

void steering_limit_interrupt() {
  calibration_phase = CALIBRATE_INTERRUPT;
}