#include <ros.h>
#include <std_msgs/UInt64.h>

#include <NewPing.h>

// ultrasonic sensor measurement logic
const int triggerPin = 11;
const int echoPin = 12;
NewPing sonar(triggerPin, echoPin);

int motorDir1 = 8;  // input 1
int motorDir2 = 9;  // input 2
int motorSpeedPin = 10;  // PWM pin

ros::NodeHandle nh;

std_msgs::UInt64 message;
ros::Publisher ultrasonic_pub("ultrasonic_signal", &message);

void ultrasonicCallback() {
  // object detected, stop the motor
  if (message.data < 100) {
    digitalWrite(motorDir1, LOW);
    digitalWrite(motorDir2, LOW);
    analogWrite(motorSpeedPin, 0);
    nh.loginfo("Object detected. Stopping the motor.");
  }
}


void setup() {
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);

  // initialize node and advertise publisher
  nh.initNode();
  nh.advertise(ultrasonic_pub);
}

void loop() {
  message.data = sonar.ping_cm();
  ultrasonic_pub.publish(&message);

  nh.spinOnce();
  ultrasonicCallback();
  delay(1000);
}