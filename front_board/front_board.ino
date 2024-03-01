#include <SPI.h>
#include <mcp2515.h>
#include <ArduPID.h>
#include <Wire.h>
#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);

uint16 pedal;
uint32 displacement;
uint32 speed;

ArduPID pid_controller;

byte I2C_B1, I2C_B2;

bool HORN_SW, HORN_FLAG, HORN_VAL;
bool WARNING_SW, WARNING_FLAG, WARNING_VAL;
bool LEFT_WARNING_SW, LEFT_WARNING_FLAG, LEFT_WARNING_VAL;
bool RIGHT_WARNING_SW, RIGHT_WARNING_FLAG, RIGHT_WARNING_VAL;
bool HEADLIGHTS_SW, HEADLIGHTS_FLAG, HEADLIGHTS_VAL;
bool ACC_SW, ACC_FLAG, ACC_VAL;

uint32 current_millis;
uint32 previous_millis = 0;

DRIVING_MODES driving_mode;

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

  // debouncing switches
  debounce_switch(&HORN_SW, &HORN_FLAG, &HORN_VAL);
  debounce_switch(&WARNING_SW, &WARNING_FLAG, &WARNING_VAL);
  debounce_switch(&LEFT_WARNING_SW, &LEFT_WARNING_FLAG, &LEFT_WARNING_VAL);
  debounce_switch(&RIGHT_WARNING_SW, &RIGHT_WARNING_FLAG, &RIGHT_WARNING_VAL);
  debounce_switch(&HEADLIGHTS_SW, &HEADLIGHTS_FLAG, &HEADLIGHTS_VAL);
  debounce_switch(&ACC_SW, &ACC_FLAG, &ACC_VAL);

  digitalWrite(HORN_PIN, HORN_VAL);
  digitalWrite(HEADLIGHTS_PIN, HEADLIGHTS_VAL);
  digitalWrite(ACC_LED, ACC_VAL);

  led_controller();
  
  if (ACC_VAL) {
    driving_mode = PID_MODE;
  } else {
    driving_mode = PEDAL_MODE;
  }

  switch (driving_mode) {
    case (PEDAL_MODE):
      throttle_value = pedal;
      break;
    case (PID_MODE):
      // TODO: replace later with variable input
      pid_desired = 1;

      pid_controller.compute();
      throttle_value = pid_output;
      break;
    default:
      throttle_value = pedal;
      break;
  }
  
  can_msg_send.data[0] = (throttle_value >> 0) & 0xFF;
  can_msg_send.data[1] = (throttle_value >> 8) & 0xFF;

  mcp2515.sendMessage(&can_msg_send);
  
  Serial.println(pid_output);
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
  ACC_SW            = bitRead(I2C_B2, ACC_I2C);
}

void debounce_switch(bool *SW_INPUT, bool *SW_FLAG, bool *SW_VALUE) {
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

void led_controller() {
  // both left and right leds will be off
  if (!WARNING_VAL || !LEFT_WARNING_VAL || !RIGHT_WARNING_VAL) {
    digitalWrite(LEFT_WARNING_PIN, 0);
    bitWrite(can_msg_send.data[2], 0, 0);

    digitalWrite(LEFT_WARNING_PIN, 0);
    bitWrite(can_msg_send.data[2], 1, 0);

    return;
  }

  if (current_millis - previous_millis > WARNING_INTERVAL) {
    previous_millis = current_millis;

    // both left and right leds will blink
    if (WARNING_VAL) {
      digitalWrite(LEFT_WARNING_PIN, !digitalRead(LEFT_WARNING_PIN));
      bitWrite(can_msg_send.data[2], 0, !digitalRead(LEFT_WARNING_PIN));

      digitalWrite(LEFT_WARNING_PIN, !digitalRead(LEFT_WARNING_PIN));
      bitWrite(can_msg_send.data[2], 1, !digitalRead(LEFT_WARNING_PIN));

    // left leds will blink
    } else if (!WARNING_VAL && LEFT_WARNING_VAL && !RIGHT_WARNING_FLAG) {
      digitalWrite(LEFT_WARNING_PIN, !digitalRead(LEFT_WARNING_PIN));
      bitWrite(can_msg_send.data[2], 0, !digitalRead(LEFT_WARNING_PIN));

    // right leds will blink
    } else if (!WARNING_VAL && !LEFT_WARNING_VAL && RIGHT_WARNING_FLAG) {
      digitalWrite(LEFT_WARNING_PIN, !digitalRead(LEFT_WARNING_PIN));
      bitWrite(can_msg_send.data[2], 1, !digitalRead(LEFT_WARNING_PIN));
    }
  }
}