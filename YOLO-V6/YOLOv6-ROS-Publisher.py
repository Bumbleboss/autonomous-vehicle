#!/usr/bin/env python

# ROS imports
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# DepthAI imports
import depthai as dai
import cv2

# Util imports
from pathlib import Path
import numpy as np
import time
import json


def main():
    # Initialize ROS node and publishers
    rospy.init_node('oakd_spatial_detection')
    img_pub = rospy.Publisher('/oakd/camera/image', Image, queue_size=10)
    point_pub = rospy.Publisher('/oakd/object/position', Point, queue_size=10)
    class_pub = rospy.Publisher('/oakd/object/class', String, queue_size=10)

    bridge = CvBridge()

    # Trained model exported from https://tools.luxonis.com/
    nnBlobPath = 'kaggle_69_n_openvino_2022.1_6shave.blob'

    # Also from Luxonis tools, contains model config.
    f = open('kaggle_69_n.json')
    nnConfig = json.load(f)

    # Class map labels
    labelMap = nnConfig['mappings']['labels']
    syncNN = True

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    nnNetworkOut = pipeline.create(dai.node.XLinkOut)

    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)
    xoutDepth = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")
    xoutDepth.setStreamName("depth")
    nnNetworkOut.setStreamName("nnNetwork")

    input_size = nnConfig['nn_config']['input_size'].split('x')
    camRgb.setPreviewSize(int(input_size[0]), int(input_size[1]))

    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")

    # Setting node configs
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

    # Align depth map to the perspective of RGB camera, on which inference is done
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
    stereo.setSubpixel(True)

    # YOLO model specific settings
    spatialDetectionNetwork.setBlobPath(nnBlobPath)
    spatialDetectionNetwork.setConfidenceThreshold(nnConfig['nn_config']['NN_specific_metadata']['confidence_threshold'])
    spatialDetectionNetwork.setNumClasses(nnConfig['nn_config']['NN_specific_metadata']['classes'])
    spatialDetectionNetwork.setCoordinateSize(nnConfig['nn_config']['NN_specific_metadata']['coordinates'])
    spatialDetectionNetwork.setAnchors(nnConfig['nn_config']['NN_specific_metadata']['anchors'])
    spatialDetectionNetwork.setAnchorMasks(nnConfig['nn_config']['NN_specific_metadata']['anchor_masks'])
    spatialDetectionNetwork.setIouThreshold(nnConfig['nn_config']['NN_specific_metadata']['iou_threshold'])

    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    camRgb.preview.link(spatialDetectionNetwork.input)
    if syncNN:
        spatialDetectionNetwork.passthrough.link(xoutRgb.input)
    else:
        camRgb.preview.link(xoutRgb.input)

    spatialDetectionNetwork.out.link(xoutNN.input)
    spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
    spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)

    while not rospy.is_shutdown():
        with dai.Device(pipeline) as device:

            # Output queues will be used to get the rgb frames and nn data from the outputs defined above
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)

            printOutputLayersOnce = True
            color = (255, 255, 255)

            while True:
                inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                depth = depthQueue.get()
                inNN = networkQueue.get()

                if printOutputLayersOnce:
                    toPrint = 'Output layer names:'
                    for ten in inNN.getAllLayerNames():
                        toPrint = f'{toPrint} {ten},'
                    print(toPrint)
                    printOutputLayersOnce = False

                frame = inPreview.getCvFrame()

                # If the frame is available, draw bounding boxes on it and show the frame
                height = frame.shape[0]
                width  = frame.shape[1]

                # Extract object detection results and spatial coordinates
                detections = inDet.detections
                for detection in detections:
                    spatial_coordinates = detection.spatialCoordinates

                    # Denormalize bounding box
                    x1 = int(detection.xmin * width)
                    x2 = int(detection.xmax * width)
                    y1 = int(detection.ymin * height)
                    y2 = int(detection.ymax * height)

                    try:
                        label = labelMap[detection.label]
                    except:
                        label = detection.label

                    # Draw detected objects box
                    cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                    # Create a Point message to publish the spatial coordinates
                    point_msg = Point()
                    point_msg.x = spatial_coordinates.x
                    point_msg.y = spatial_coordinates.y
                    point_msg.z = spatial_coordinates.z

                    # Create a class message to publish the object's class
                    class_msg=String()
                    class_msg.data= label
                   
                    # Publish the Point message
                    point_pub.publish(point_msg)

                    # Publish the class message
                    class_pub.publish(class_msg)

                # Publish the RGB image
                image_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
                img_pub.publish(image_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


