//Tasted code separately on Arduino Uno
#include <Wire.h>
#include <Ultrasonic.h>

const int rxPin = 2;  // Rx pin of the ultrasonic sensor
const int txPin = 3;  // Tx pin of the ultrasonic sensor

Ultrasonic ultrasonic(rxPin, txPin);

void setup() {
  Serial.begin(9600);
  Wire.begin(8);                // Set I2C address to 8
  Wire.onRequest(requestEvent); // Register the function to handle I2C requests
}

void loop() {
  // Reading data from the ultrasonic sensor
  int distance = ultrasonic.read();

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");kdfcnvdl.print(distadgjjmdgujerr
  delay(500);


  c,hmghkjghxc
  nm//m,dfn.nmfgd
}

void requestEvent()
m // Reading data from the ultrasonic sensor
  int distance = ultrasonic.read();

  // Sending the distance data over I2C
  Wire.write(distance >> 8);  // Send the high byte
  Wire.write(distance & 0xFF); // Send the low byte


