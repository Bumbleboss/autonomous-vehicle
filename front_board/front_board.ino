#include <SPI.h>
#include <mcp2515.h>
#include <ArduPID.h>
#include <Wire.h>
// #include <std_msgs/Float64.h>
#include <ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);


uint16 pedal;
uint32 displacement;
uint32 speed;

ArduPID myController;


byte I2C_B1, I2C_B2;

bool ACC_FLAG, ACC_LOOP;
bool ACC_SW;

DRIVING_MODES driving_mode;

// PID parameters
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

  myController.begin(&pid_input, &pid_output, &pid_desired, 160 , 0.005 , 0.005);
  myController.setOutputLimits(0, 1024);


  Wire.begin(8); // join I2C bus with address #8
  Wire.onReceive(I2C_Read); // register event

  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

}

void loop() {
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    displacement = (can_msg_receive.data[0] & 0xFF) | ((can_msg_receive.data[1] & 0xFF) << 8) | ((can_msg_receive.data[2] & 0xFF) << 16) | ((can_msg_receive.data[3] & 0xFF) << 24);
    speed = (can_msg_receive.data[4] & 0xFF) | ((can_msg_receive.data[5] & 0xFF) << 8) | ((can_msg_receive.data[6] & 0xFF) << 16) | ((can_msg_receive.data[7] & 0xFF) << 24);
  }
  
  // map pedal resolution to motor
  pedal = analogRead(PEDAL_PIN);

  // convert speed to appropriate m/s double
  pid_input = speed / 100.0;

  // acceleration switch debouncing
  if (ACC_SW == 1 && ACC_FLAG == 1) {
    delay(50);

    if (ACC_SW == 1) {
      ACC_LOOP = !ACC_LOOP;
      ACC_FLAG = 0;
    }
  }
  
  if (ACC_SW == 0) {
    ACC_FLAG = 1;
  }

  if (ACC_LOOP) {
    driving_mode = PID_MODE;
  } else {
    driving_mode = PEDAL_MODE;
  }

  digitalWrite(ACC_LED, ACC_LOOP);

  switch (driving_mode) {
    case (PEDAL_MODE):
      // left rear bulb and right rear bulb
      bitWrite(can_msg_send.data[2], 0, 1);
      bitWrite(can_msg_send.data[2], 1, 0);

      throttle_value = pedal;
      break;
    case (PID_MODE):
      // left rear bulb and right rear bulb
      bitWrite(can_msg_send.data[2], 0, 1);
      bitWrite(can_msg_send.data[2], 1, 1);

      // replaced later with variable input
      pid_desired = 1;

      myController.compute();
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

  ACC_SW = bitRead(I2C_B2, 2);
}
