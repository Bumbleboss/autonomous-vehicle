import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge

# depthai imports
import depthai as dai
import cv2

# util imports
from pathlib import Path
import numpy as np
import json

def timeDeltaToMilliS(delta) -> float:
  return delta.total_seconds()*1000

def main():
  # initialize ros nodes and publishers
  rospy.init_node('oakd')
  img_pub = rospy.Publisher('/oakd/camera/image', Image, queue_size=10)
  point_pub = rospy.Publisher('/oakd/object/position', Point, queue_size=10)
  class_pub = rospy.Publisher('/oakd/object/class', String, queue_size=10)
  imu_pub = rospy.Publisher('/oakd/imu', Imu, queue_size=10)

  bridge = CvBridge()

  # trained model exported from https://tools.luxonis.com/
  nnBlobPath = 'src/oakd/yolo_models/yolov7-tiny.blob'

  # also from Luxonis tools, contains model config.
  f = open('src/oakd/yolo_models/yolov7-tiny.json')
  nnConfig = json.load(f)

  # class map labels
  labelMap = nnConfig['mappings']['labels']
  syncNN = True

  # start pipeline
  pipeline = dai.Pipeline()

  # define sources and outputs
  camRgb = pipeline.create(dai.node.ColorCamera)
  spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
  monoLeft = pipeline.create(dai.node.MonoCamera)
  monoRight = pipeline.create(dai.node.MonoCamera)
  stereo = pipeline.create(dai.node.StereoDepth)
  nnNetworkOut = pipeline.create(dai.node.XLinkOut)
  imu = pipeline.create(dai.node.IMU)

  xoutRgb = pipeline.create(dai.node.XLinkOut)
  xoutNN = pipeline.create(dai.node.XLinkOut)
  xoutDepth = pipeline.create(dai.node.XLinkOut)
  xlinkOut = pipeline.create(dai.node.XLinkOut)

  xoutRgb.setStreamName("rgb")
  xoutNN.setStreamName("detections")
  xoutDepth.setStreamName("depth")
  nnNetworkOut.setStreamName("nnNetwork")
  xlinkOut.setStreamName("imu")

  input_size = nnConfig['nn_config']['input_size'].split('x')
  camRgb.setPreviewSize(int(input_size[0]), int(input_size[1]))

  camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
  camRgb.setInterleaved(False)
  camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

  monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  monoLeft.setCamera("left")
  monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  monoRight.setCamera("right")

  # imu settings
  imu.enableIMUSensor([dai.IMUSensor.LINEAR_ACCELERATION, dai.IMUSensor.GYROSCOPE_CALIBRATED, dai.IMUSensor.ROTATION_VECTOR], 100)
  imu.setBatchReportThreshold(1)
  imu.setMaxBatchReports(10)

  # setting node configs
  stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

  # align depth map to the perspective of RGB camera, on which inference is done
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

  # linking
  monoLeft.out.link(stereo.left)
  monoRight.out.link(stereo.right)
  imu.out.link(xlinkOut.input)

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
      # output queues for rgb, detections, depth, nnNetwork, and imu
      previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
      detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
      depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
      networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)
      imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

      printOutputLayersOnce = True
      color = (255, 255, 255)

      baseTs = None

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

        # if the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]

        # extract object detection results and spatial coordinates
        detections = inDet.detections
        for detection in detections:
          spatial_coordinates = detection.spatialCoordinates

          # denormalize bounding box
          x1 = int(detection.xmin * width)
          x2 = int(detection.xmax * width)
          y1 = int(detection.ymin * height)
          y2 = int(detection.ymax * height)

          try:
            label = labelMap[detection.label]
          except:
            label = detection.label

          # draw detected objects box
          cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

          cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

          # create a point message to publish the spatial coordinates
          point_msg = Point()
          point_msg.x = spatial_coordinates.x
          point_msg.y = spatial_coordinates.y
          point_msg.z = spatial_coordinates.z

          # create a class message to publish the object's class
          class_msg= String()
          class_msg.data= label
          
          # publish the Point message
          point_pub.publish(point_msg)

          # publish the class message
          class_pub.publish(class_msg)

        # publish the RGB image
        image_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        img_pub.publish(image_msg)
        
        # extract imu data
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
        imuPackets = imuData.packets

        for imuPacket in imuPackets:
          acceleroValues = imuPacket.acceleroMeter
          gyroValues = imuPacket.gyroscope
          rVvalues = imuPacket.rotationVector

          acceleroTs = acceleroValues.getTimestampDevice()
          gyroTs = gyroValues.getTimestampDevice()
          rotateTs = rVvalues.getTimestampDevice()

          if baseTs is None:
            baseTs = min(acceleroTs, gyroTs, rotateTs)

          acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
          gyroTs = timeDeltaToMilliS(gyroTs - baseTs)
          rotateTs = timeDeltaToMilliS(rotateTs - baseTs)

          imu_msg = Imu()

          imu_msg.header.frame_id = 'oak_imu_frame'
          
          # linear acceleration
          imu_msg.linear_acceleration.x = acceleroValues.x
          imu_msg.linear_acceleration.y = acceleroValues.y
          imu_msg.linear_acceleration.z = acceleroValues.z

          # angular velocity
          imu_msg.angular_velocity.x = gyroValues.x
          imu_msg.angular_velocity.y = gyroValues.y
          imu_msg.angular_velocity.z = gyroValues.z

          # quaternion
          imu_msg.orientation.x = rVvalues.i
          imu_msg.orientation.y = rVvalues.j
          imu_msg.orientation.z = rVvalues.k
          imu_msg.orientation.w = rVvalues.real

          imu_pub.publish(imu_msg)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass


