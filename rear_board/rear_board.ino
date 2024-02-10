#include <SPI.h>
#include <mcp2515.h>
#include <PWM.h>
#include "rear_board.h"

MCP2515 mcp2515(CAN_PIN);

uint16 throttle;
uint16 throttle_old;

volatile uint32 pulse_count = 0;
volatile uint32 displacement = 0;
volatile uint32 speed = 0;

volatile uint32 start_time = 0;
volatile uint32 current_time = 0;
volatile uint32 elapsed_time = 0;

void setup() {
  can_msg_send.can_id = 0x02;
  can_msg_send.can_dlc = 8;

  // displacement value
  can_msg_send.data[0] = 0x00;
  can_msg_send.data[1] = 0x00;
  can_msg_send.data[2] = 0x00;
  can_msg_send.data[3] = 0x00;

  // speed value
  can_msg_send.data[4] = 0x00;
  can_msg_send.data[5] = 0x00;
  can_msg_send.data[6] = 0x00;
  can_msg_send.data[7] = 0x00;

  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  InitTimersSafe();

  // defining light and motor pins
  pinMode(LEFT_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);
  pinMode(BRAKES_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // setting up counter  
  pinMode(COUNTER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN), calculate_speed, RISING);

  start_time = micros();
}

void loop() {
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    throttle = (can_msg_receive.data[0] & 0xFF) | ((can_msg_receive.data[1] & 0xFF) << 8);
  }

  // set the bulbs based on statements from front board
  digitalWrite(LEFT_PIN, bitRead(can_msg_receive.data[2], 0));  
  digitalWrite(RIGHT_PIN, bitRead(can_msg_receive.data[2], 1));
  digitalWrite(BRAKES_PIN, !bitRead(can_msg_receive.data[2], 2));

  // if there's a difference between old and current throttle value
  // then sequence the pwm signal, if not, set signal right away
  if(abs(throttle - throttle_old) > 1) {
    set_throttle(throttle); 
  } else {
    set_motor_value(throttle);
  }

  // save old throttle value for later computation
  throttle_old = throttle;

  // add displacement to CAN frame
  can_msg_send.data[0] = (displacement >> 0) & 0xFF;
  can_msg_send.data[1] = (displacement >> 8) & 0xFF;
  can_msg_send.data[2] = (displacement >> 16) & 0xFF;
  can_msg_send.data[3] = (displacement >> 24) & 0xFF;

  // add speed to CAN frame,
  // zero if elapsed time is above 1s
  if (elapsed_time > 1000000) {
    can_msg_send.data[4] = 0x00;
    can_msg_send.data[5] = 0x00;
    can_msg_send.data[6] = 0x00;
    can_msg_send.data[7] = 0x00;
  } else {
    can_msg_send.data[4] = (speed >> 0) & 0xFF;
    can_msg_send.data[5] = (speed >> 8) & 0xFF;
    can_msg_send.data[6] = (speed >> 16) & 0xFF;
    can_msg_send.data[7] = (speed >> 24) & 0xFF;
  }

  mcp2515.sendMessage(&can_msg_send);
}    

void set_motor_value(uint16 value) {
  pwmWriteHR(MOTOR_PIN, constrain(map(value, 0, 1024, 0, 65535), 0, 65535));
  SetPinFrequencySafe(MOTOR_PIN, 1000);
}

void set_throttle(uint16 value) {
  uint16 i = throttle_old;

  // fancy statements to avoid having two separate loops
  for (; i > value | i < value; throttle_old > value ? i-- : i++) {
    set_motor_value(i);
  }
}

void calculate_speed() {
  pulse_count++;

  current_time = micros();
  elapsed_time = current_time - start_time;

  float displacement_f = pulse_count / TICKS_PER_METER;
  float speed_f = (1 / TICKS_PER_METER) / (elapsed_time / 1000000.0);

  // we do this to shift the first two digits after decimal
  // and send it via CAN
  displacement = displacement_f * 100;
  speed = speed_f * 100;
  
  // shifting start time to current time in order
  // to get the next tick timeframe
  start_time = current_time;
}
