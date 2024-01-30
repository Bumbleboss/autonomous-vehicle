#include <ros.h>
#include <geometry_msgs/Twist.h>

/**
 * set the speed of vehicle by using x,y,z components
*/
void set_speed(float speed) {
  // code for throttle speed
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
  node_handle.initNode();
  node_handle.subscribe(twist_sub);
}

void loop() {
  node_handle.spinOnce();
  delay(1);
}