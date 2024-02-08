#include <mcp2515.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);

uint8 pedal_value;
uint32 displacement;
uint32 speed;

CAR_MODES car_mode;

byte I2C_B1, I2C_B2;

bool ACC_FLAG, ACC_LOOP;
bool ACC_SW;


// PID parameters
double input_speed;
double desired_speed = 0.01;
double pid_output;
double K_P = 50;
double K_I = 5;
double K_D = 0.5;

double throttle_output;

PID pid_control(&input_speed, &pid_output, &desired_speed, K_P, K_I, K_D, DIRECT);

void setup() {
  can_msg_send.can_id = 0x00;
  can_msg_send.can_dlc = 2;
  can_msg_send.data[0] = 0x00; // throttle
  can_msg_send.data[1] = 0x00; // bulb

  Wire.begin(8); // join I2C bus with address #8
  Wire.onReceive(I2C_Read); // register event

  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  pid_control.SetMode(AUTOMATIC);
}

void loop() {
  if (mcp2515.readMessage(&can_msg_receive) == MCP2515::ERROR_OK) {
    displacement = (can_msg_receive.data[0] & 0xFF) | ((can_msg_receive.data[1] & 0xFF) << 8) | ((can_msg_receive.data[2] & 0xFF) << 16) | ((can_msg_receive.data[3] & 0xFF) << 24);
    speed = (can_msg_receive.data[4] & 0xFF) | ((can_msg_receive.data[5] & 0xFF) << 8) | ((can_msg_receive.data[6] & 0xFF) << 16) | ((can_msg_receive.data[7] & 0xFF) << 24);
  }
  
  // map pedal resolution to motor
  pedal_value = map(analogRead(PEDAL_PIN), 0, 1024, 0, 255);

  // convert speed to appropriate m/s double
  input_speed = speed * (1000 / 100000000.0);

  if (ACC_SW == 1 && ACC_FLAG == 1) {
    delay(50);

    if (ACC_SW == 1) {
      ACC_LOOP = !ACC_LOOP;
      ACC_FLAG = 0;
    }
  }

  if (ACC_SW == 0) {
    ACC_FLAG = 1;
    car_mode = PID_MODE;
  } else {
    car_mode = PEDAL_MODE;
  }

  digitalWrite(ACC_LED, ACC_LOOP);


  switch (car_mode) {
    case (PEDAL_MODE):
      bitWrite(can_msg_send.data[1], 0, 1);
      bitWrite(can_msg_send.data[1], 1, 0);
      throttle_output = pedal_value;
      break;
    case (PID_MODE):
      bitWrite(can_msg_send.data[1], 0, 1);
      bitWrite(can_msg_send.data[1], 1, 1);

      // replaced later with variable input
      desired_speed = 1;

      pid_control.Compute();
      throttle_output = pid_output;
      break;
    default:
      throttle_output = pedal_value;
      break;
  }

  mcp2515.sendMessage(&can_msg_send);
}

void I2C_Read(int howMany) {
  I2C_B1 = Wire.read();
  I2C_B2 = Wire.read();

  ACC_SW = bitRead(I2C_B2, 2);
}

