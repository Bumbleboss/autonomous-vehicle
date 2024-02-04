#include <PWM.h>
#include <mcp2515.h>
#include "rear_board_pins.h"

MCP2515 mcp2515(CAN_PIN);

struct can_frame can_msg_receive;
struct can_frame can_msg_send;

long throttle;
long throttle_old;

volatile int pulse_count = 0;

/**
 * interrupt service routine for calculating pulses
*/
void int0ISR() {
  pulse_count++;

  if (pulse_count >= 65000) {
    pulse_count = 0;
  }
}

/**
 * refactored function for sending pwm signal to motor
*/
void set_motor_value(int value) {
  pwmWriteHR(MOTOR_PIN, value);
  SetPinFrequencySafe(MOTOR_PIN, 1000);
}

/**
 * increment or decrement to the desired value based on old throttle value
*/
void set_throttle(int value) {
  if (value >= 0 && value <= 255) {
    int i = throttle_old;

    // fancy conditions to avoid having two separate loops
    for (; i > value | i < value; throttle_old > value ? i-- : i++) {
      set_motor_value(i);
    }
  }
}

void setup() {
  // setting up CAN
  can_msg_send.can_id = 0x02;
  can_msg_send.can_dlc = 2;
  can_msg_send.data[0] = 0x00;
  can_msg_send.data[1] = 0x00;

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
    throttle = (can_msg_receive.data[0] & 0xFF) | ((can_msg_receive.data[1] & 0xFF) << 8);

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

  // send pulse counts via CAN
  can_msg_send.data[0] = (pulse_count >> 0) & 0xFF;
  can_msg_send.data[1] = (pulse_count >> 8) & 0xFF;
  mcp2515.sendMessage(&can_msg_send);
}    
