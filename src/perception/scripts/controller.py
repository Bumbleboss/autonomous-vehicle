#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDrive
from zed_interfaces.msg import ObjectsStamped
from std_msgs.msg import UInt8

import time

stop_car = False
object_limit = 5

stop_car_time = 7
stop_car_start_time = None

# sets the steering and speed values while passing other data the same
def ackermann_callback(msg: AckermannDrive, cmd_pub):
  if stop_car:
    msg.speed = 0.0
    msg.steering_angle = 0.0

  cmd_pub.publish(msg)

# uses the topic of detected objects from zed camera
# to check if the object is near a certain limit and for a periodic time
# which then sends signal for car to stop and horn as well
# the horning happens on a frequent rate
def objects_callback(msg: ObjectsStamped, horn_pub):
  global stop_car, stop_car_start_time
    
  if len(msg.objects) > 0:
    objects_pos = []

    for object in msg.objects:
      objects_pos.append(object.position[0])

    if max(objects_pos, default=0) < object_limit:
      if not stop_car:
          stop_car_start_time = time.time()
      stop_car = True
    else:
      stop_car = False
      stop_car_start_time = None
  else:
    # edge case scenario where all objects disappear when previous state
    stop_car = False
    stop_car_start_time = None

  # timer logic that resets every x seconds
  if stop_car and stop_car_start_time is not None:
    elapsed_time = time.time() - stop_car_start_time

    if elapsed_time > stop_car_time:
      horn_pub.publish(UInt8(1))
      rospy.sleep(0.169) # duration for how long the horn should be
      horn_pub.publish(UInt8(0))
      stop_car_start_time = time.time()

def ackermann_controller():
  rospy.init_node('perception')

  cmd_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
  horn_pub = rospy.Publisher('/horn', UInt8, queue_size=10)

  rospy.Subscriber('/priority_control', AckermannDrive, ackermann_callback, cmd_pub)
  rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, objects_callback, horn_pub)

  rospy.spin()

if __name__ == '__main__':
  try:
    ackermann_controller()
  except rospy.ROSInterruptException:
    pass
