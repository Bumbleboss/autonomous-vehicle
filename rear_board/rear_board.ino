#include <PWM.h>
#include <mcp2515.h>
#include "rear_board.h"

MCP2515 mcp2515(CAN_PIN);

uint8 throttle;
uint8 throttle_old;

volatile uint32 pulse_count = 0;
uint32 displacement = 0;
uint8 speed = 0;

uint32 start_time = 0;

void setup() {
  // setting up CAN
  can_msg_send.can_id = 0x02;
  can_msg_send.can_dlc = 5;
  can_msg_send.data[0] = 0x00;
  can_msg_send.data[1] = 0x00;
  can_msg_send.data[2] = 0x00;
  can_msg_send.data[3] = 0x00;
  can_msg_send.data[4] = 0x00;

  // do not change baud rate because it affects rosserial
  Serial.begin(57600);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  InitTimersSafe();

  // defining bulbs and motor pin
  pinMode(LEFT_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);
  pinMode(BRAKES_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // setting up counter  
  pinMode(COUNTER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(COUNTER_PIN), int0ISR, RISING); 
}

void loop() {
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    throttle = can_msg_receive.data[0];

    // if there's a difference between old and current throttle value
    // then sequence the pwm signal, if not, set signal right away
    if(abs(throttle - throttle_old) > 1) {
      set_throttle(throttle); 
    } else {
      set_motor_value(throttle);
    }

    // save old throttle value for later computation
    throttle_old = throttle;
    
    digitalWrite(LEFT_PIN, bitRead(can_msg_receive.data[2], 0));  
    digitalWrite(RIGHT_PIN, bitRead(can_msg_receive.data[2], 1));
    digitalWrite(BRAKES_PIN, !bitRead(can_msg_receive.data[2], 2));
  }

  // add displacement (m) to CAN frame
  can_msg_send.data[0] = (displacement >> 0) & 0xFF;
  can_msg_send.data[1] = (displacement >> 8) & 0xFF;
  can_msg_send.data[2] = (displacement >> 16) & 0xFF;
  can_msg_send.data[3] = (displacement >> 24) & 0xFF;

  // Accident_LED speed (m/s) to CAN frame
  can_msg_send.data[4] = speed;

  mcp2515.sendMessage(&can_msg_send);
}    

void int0ISR() {
  pulse_count++;

  uint32 current_time = millis();
  uint32 elapsed_time = current_time - start_time;

  start_time = current_time;

  displacement = pulse_count / TICKS_PER_METER;
  speed = (TICKS_PER_METER / pulse_count) / (elapsed_time / 1000.0);
}

void set_motor_value(uint8 value) {
  pwmWriteHR(MOTOR_PIN, value);
  SetPinFrequencySafe(MOTOR_PIN, 1000);
}

void set_throttle(uint8 value) {
  uint8 i = throttle_old;

  // fancy statements to avoid having two separate loops
  for (; i > value | i < value; throttle_old > value ? i-- : i++) {
    set_motor_value(i);
  }
}