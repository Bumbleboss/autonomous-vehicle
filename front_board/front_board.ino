#include <mcp2515.h>
#include "front_board.h"

MCP2515 mcp2515(CAN_PIN);

uint8 pedal_value = 0;
uint32 displacement = 0;
uint32 speed = 0;


void setup() {
  can_msg_send.can_id = 0x00;
  can_msg_send.can_dlc = 2;
  can_msg_send.data[0] = 0x00; // throttle
  can_msg_send.data[1] = 0x00; // bulb

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
  pedal_value = map(analogRead(PEDAL_PIN), 0, 1024, 0, 255);

  // we only compute float values for printing
  Serial.print("Displacement: ");
  Serial.print(displacement / 1000.0, 2);
  Serial.print("\n");

  Serial.print("Speed: ");
  Serial.print(speed * (1000 / 100000000.0), 2);
  Serial.print("\n");

  Serial.print("Pedal: ");
  Serial.print(pedal_value);
  Serial.print("\n");

  // set bulb depending on settings
  bitWrite(can_msg_send.data[1], 0, 1); // left bulb
  bitWrite(can_msg_send.data[1], 1, 0); // right bulb
  bitWrite(can_msg_send.data[1], 2, 1); // brake bulb

  // send pedal as throttle
  can_msg_send.data[0] = pedal_value;

  mcp2515.sendMessage(&can_msg_send);
}