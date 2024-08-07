#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch

# Initialize the ROS node
rospy.init_node('traffic_sign_detection', anonymous=True)

# Create a CvBridge object
bridge = CvBridge()

# Publishers
image_pub = rospy.Publisher('/image_traffic', Image, queue_size=10)
class_pub = rospy.Publisher('/traffic_class', String, queue_size=10)

# Load YOLOv9 model
model = torch.hub.load('ultralytics/yolov9', 'custom', '/home/hima/Downloads/best.pt')  # Update this line if necessary

# Define the process_image callback function
def process_image(image_msg):
    try:
        # Convert the ROS Image message to a CV2 Image
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Object Detection using YOLOv9
        results = model(cv_image)
        
        # Extract detected classes
        detected_classes = [model.names[int(x[-1])] for x in results.xyxy[0]]

        # Convert the image with bounding boxes to a ROS Image message
        results.render()
        rendered_image = results.ims[0]
        detection_result = bridge.cv2_to_imgmsg(rendered_image, "bgr8")

        # Publish the image with detections
        image_pub.publish(detection_result)

        # Publish the detected classes
        if detected_classes:
            for cls in detected_classes:
                class_pub.publish(String(cls))
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

# Subscribe to the image topic
rospy.Subscriber("/image_raw", Image, process_image)

# Keep the node running
rospy.spin()

# Close all OpenCV windows on shutdown
cv2.destroyAllWindows()
