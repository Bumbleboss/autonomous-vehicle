#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <mcp2515.h>
#include <PWM.h>
#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);

// computed feedback values (based on pulse counts)
uint8 fb_speed = 0;
uint8 fb_acceleration = 0;
uint16 fb_displacement = 0;

uint32 pulse_count = 0;
uint32 start_time = 0;

/**
 * calculate the displacement of vehicle using pulse counts
 * calculation:
 * diameter of the wheel = 50cm
 * circumference of the wheel 2*pi*r = 1.5m
 * 52.6 pulses per one rev
 * pulse factor = c / pulses per rev = 0.028517
*/
void calc_displacement() {
  fb_displacement = 0.028517 * pulse_count;
}

/**
 * calculate speed (m/s)
*/
void calc_speed() {
  uint32 current_time = millis();
  fb_speed = fb_displacement / (current_time - *time / 1000);
}

/**
 * set the speed of the vehicle
*/
void set_speed(uint8 speed) {
  // we need to map speed relative to
  // actual speed from feedback and use pid
  // speed = map(speed, 0, 255, 0, ??);
  // the result would be the computed speed
  uint8 computed_speed = speed;
  
  can_msg_send.data[0] = computed_speed;
}

/**
 * set the steering angle of the vehicle
*/
void set_steering(float angle) {
  // code for steering angle
}

/**
 * receive linear and angular velocity and send it to related functions.
*/
void twist_cb(const geometry_msgs::Twist& twist_msg) {
  // control speed
  set_speed(twist_msg.linear.x);

  // control steering angle
  set_steering(twist_msg.angular.z);
}

ros::NodeHandle node_handle;
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_cb);

void setup() {
  // setup CAN to send throttle data
  can_msg_send.can_id = 0x00;
  can_msg_send.can_dlc = 2;
  can_msg_send.data[0] = 0x00; // throttle
  can_msg_send.data[1] = 0x00; // bulb

  // do not change baud rate because it affects rosserial
  Serial.begin(57600);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  // initialize ros node and start handling with topics
  node_handle.initNode();
  node_handle.subscribe(twist_sub);

  start_time = millis();
}

void loop() {
  // receive pulses from rear board through CAN
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    pulse_count = (can_msg_receive.data[0] & 0xFF) | ((can_msg_receive.data[1] & 0xFF) << 8) | ((can_msg_receive.data[2] & 0xFF) << 16) | ((can_msg_receive.data[1] & 0xFF) << 24);
  }

  node_handle.spinOnce();
  delay(1);
}