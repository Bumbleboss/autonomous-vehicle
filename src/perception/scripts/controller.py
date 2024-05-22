#!/usr/bin/env python

import rospy

from ackermann_msgs.msg import AckermannDrive
from zed_interfaces.msg import ObjectsStamped

stop_car = False
object_limit = 5

def ackermann_callback(msg: AckermannDrive, cmd_pub):
    if stop_car:
        msg.speed = 0.0
        msg.steering_angle = 0.0

    cmd_pub.publish(msg)

def objects_callback(msg: ObjectsStamped):
    global stop_car
    
    if len(msg.objects) > 0:
        objects_pos = []

        for object in msg.objects:
            objects_pos.append(object.position[0])

        if max(objects_pos, default=0) < object_limit:
            stop_car = True
        else:
            stop_car = False
    else:
        # edge case scenario where all objects disappear when previous state
        # for stop_car was true
        stop_car = False

def ackermann_controller():
    rospy.init_node('perception')

    cmd_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

    rospy.Subscriber('/ackermann_controller', AckermannDrive, ackermann_callback, cmd_pub)
    rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, objects_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        ackermann_controller()
    except rospy.ROSInterruptException:
        pass
