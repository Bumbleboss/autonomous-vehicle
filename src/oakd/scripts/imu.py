import cv2

import rospy
import depthai as dai

import numpy as np
from sensor_msgs.msg import Imu

# def quaternion_to_euler(x, y, z, w):
#   t0 = +2.0 * (w * x + y * z)
#   t1 = +1.0 - 2.0 * (x * x + y * y)
#   roll_x = math.atan2(t0, t1)

#   t2 = +2.0 * (w * y - z * x)
#   t2 = +1.0 if t2 > +1.0 else t2
#   t2 = -1.0 if t2 < -1.0 else t2
#   pitch_y = math.asin(t2)

#   t3 = +2.0 * (w * z + x * y)
#   t4 = +1.0 - 2.0 * (y * y + z * z)
#   yaw_z = math.atan2(t3, t4)

#   return roll_x, pitch_y, yaw_z  # in radians

def timeDeltaToMilliS(delta) -> float:
  return delta.total_seconds()*1000

def main():
  rospy.init_node('oakd_imu')
  imu_pub = rospy.Publisher('/oakd/camera/imu', Imu, queue_size=10)

  # create pipeline
  pipeline = dai.Pipeline()

  # define sources and outputs
  imu = pipeline.create(dai.node.IMU)

  xlinkOut = pipeline.create(dai.node.XLinkOut)
  xlinkOut.setStreamName("imu")

  imu.enableIMUSensor([dai.IMUSensor.LINEAR_ACCELERATION, dai.IMUSensor.GYROSCOPE_CALIBRATED, dai.IMUSensor.ROTATION_VECTOR], 100)
  imu.setBatchReportThreshold(1)
  imu.setMaxBatchReports(10)

  # link plugins IMU -> XLINK
  imu.out.link(xlinkOut.input)

  velocity = np.zeros(3)
  position = np.zeros(3)

  while not rospy.is_shutdown():
    with dai.Device(pipeline) as device:
      
      # output queue for imu bulk packets
      imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
      baseTs = None

      while True:
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

          velocity += np.array([acceleroValues.x, acceleroValues.y, acceleroValues.z]) * acceleroTs / 1000
          position += velocity * acceleroTs / 1000

          imuF = "{:.06f}"
          tsF  = "{:.03f}"

          print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
          print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
          
          # print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
          # print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")
          
          print(f"Velocity [m/s]: x: {velocity[0]} y: {velocity[1]} z: {velocity[2]}")
          print(f"Position [m]: x: {position[0]} y: {position[1]} z: {position[2]}")

          # # Example usage:
          # quaternion_data = (rVvalues.i, rVvalues.j, rVvalues.k, rVvalues.real)  # Replace with your actual quaternion data
          # roll, pitch, yaw = quaternion_to_euler(*quaternion_data)

          # print(f"Roll: {math.degrees(roll)} degrees")
          # print(f"Pitch: {math.degrees(pitch)} degrees")
          # print(f"Yaw: {math.degrees(yaw)} degrees")

        if cv2.waitKey(1) == ord('q'):
          break

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass