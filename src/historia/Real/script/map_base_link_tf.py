#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import tf2_msgs
import tf

def callback_odom_to_base(msg):
    global T_odom_base_link
    for transform in msg.transforms:
        if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
            T_odom_base_link = msg.transform

def callback_map_to_base(msg):
    global T_map_base_link
    T_map_base_link = msg.transform

def calculate_map_to_odom():
    global T_map_odom
    T_map_odom = tf2_geometry_msgs.do_transform_pose(geometry_msgs.msg.PoseStamped(), T_map_base_link)
    T_map_odom = tf2_geometry_msgs.do_transform_pose(T_map_odom, T_odom_base_link)

if __name__ == "__main__":
    rospy.init_node('tf_subscriber')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)  # Give some time for the tf buffer to populate

    T_map_base_link = None
    T_odom_base_link = None
    T_map_odom = None

    # Subscribe to the transformation from odom to base_link
    # odom_to_base_sub = rospy.Subscriber('/tf', tf2_ros.TFMessage, callback_odom_to_base)

    # Subscribe to the transformation from map to base_link
    # map_to_base_sub = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, callback_map_to_base)

    # Create a TransformBroadcaster
    broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(1)  # Update rate (1 Hz)

    while not rospy.is_shutdown():
        listener = tf.TransformListener()
        try:
            trans = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.loginfo(trans)

        if T_odom_base_link is not None and T_map_base_link is not None:
            calculate_map_to_odom()
            # Publish the transformation from map to odom
            transform_stamped = geometry_msgs.msg.TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "map"
            transform_stamped.child_frame_id = "odom"
            transform_stamped.transform = T_map_odom.transform
            # broadcaster.sendTransform(transform_stamped)
            print("Published transformation from map to odom:")
            print(T_map_odom)
        
        rate.sleep()

    rospy.spin()
