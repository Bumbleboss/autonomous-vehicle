#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import NavSatFix

def gps_callback(msg):
    # Extract GPS data from NavSatFix message
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude

    x = -0.1
    y = 0.2
    z = 1.13

    # Create TF broadcaster
    br = tf.TransformBroadcaster()

    # Publish TF transform
    br.sendTransform((x, y, z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),  
                     rospy.Time.now(),
                     "GPS_link",
                     "base_link")  

def gps_driver():
    rospy.init_node('gps_driver')
    rospy.Subscriber('/fix', NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    gps_driver()
