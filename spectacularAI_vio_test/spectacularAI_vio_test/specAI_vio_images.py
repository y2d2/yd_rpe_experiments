import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from nav_msgs.msg import Odometry
import depthai
import spectacularAI
import numpy as np



class VIO(Node):
    def __init__(self):
        super().__init__('spectacularAI_vio_node')



        # Create pipeline and VIO session
        #
        # # Setup mono cameras
        # self.left = self.pipeline.createMonoCamera()
        # # self.right = self.pipeline.createMonoCamera()
        # self.left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
        # # self.right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
        # self.left.setBoardSocket(depthai.CameraBoardSocket.LEFT)
        # # self.right.setBoardSocket(depthai.CameraBoardSocket.RIGHT)
        #
        # # Setup video encoders
        # self.leftEncoder = self.pipeline.createVideoEncoder()
        # # self.rightEncoder = self.pipeline.createVideoEncoder()
        # self.leftEncoder.setDefaultProfilePreset(1, depthai.VideoEncoderProperties.Profile.MJPEG)
        # # self.rightEncoder.setDefaultProfilePreset(1, depthai.VideoEncoderProperties.Profile.MJPEG)
        #
        # # Link cameras to encoders
        # self.left.out.link(self.leftEncoder.input)
        # # self.right.out.link(self.rightEncoder.input)

        # Setup VIO
        self.pipeline = depthai.Pipeline()

        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline)


        self.vio_pub = self.create_publisher(Odometry, 'vio', 10)
        self.left_img_pub = self.create_publisher(CompressedImage, 'left/image/compressed', 10)
        self.right_img_pub = self.create_publisher(CompressedImage, 'right/image/compressed', 10)

        self.oakd_imu_pub = self.create_publisher(Imu, 'oakd/imu', 10)

        left_cam = self.vio_pipeline.monoLeft
        right_cam = self.vio_pipeline.monoRight
        # Create IMU node
        imu = self.vio_pipeline.imu

        # # Enable IMU sensors: accelerometer and gyroscope
        # imu.enableIMUSensor(depthai.IMUSensor.ACCELEROMETER_RAW, 500)
        # imu.enableIMUSensor(depthai.IMUSensor.GYROSCOPE_RAW, 500)
        #
        # # Optionally: batch reports together
        # imu.setBatchReportThreshold(1)
        # imu.setMaxBatchReports(10)
        #
        # # Create XLinkOut for IMU
        # xout_imu = self.pipeline.createXLinkOut()
        # xout_imu.setStreamName("imu")
        # imu.out.link(xout_imu.input)

        # print(left_can)

        leftEncoder = self.pipeline.createVideoEncoder()
        leftEncoder.setDefaultProfilePreset(
            30, depthai.VideoEncoderProperties.Profile.MJPEG
        )
        left_cam.out.link(leftEncoder.input)


        rightEncoder = self.pipeline.createVideoEncoder()
        rightEncoder.setDefaultProfilePreset(
            30, depthai.VideoEncoderProperties.Profile.MJPEG
        )
        right_cam.out.link(rightEncoder.input)


        left_xout = self.pipeline.createXLinkOut()
        right_xout = self.pipeline.createXLinkOut()

        imu_xout = self.pipeline.createXLinkOut()
        imu_xout.setStreamName('xoutimu')

        # xout1 = pipeline.create(dai.node.XLinkOut)
        #

        left_xout.setStreamName("xoutleft")
        leftEncoder.bitstream.link(left_xout.input)

        right_xout.setStreamName("xoutright")
        rightEncoder.bitstream.link(right_xout.input)

        # left_can.out.link(left_xout.input)

        # right_xout = self.vio_pipeline.monoRight.out
        # self.leftEncoder = self.pipeline.createVideoEncoder()
        # self.leftEncoder.setDefaultProfilePreset(1, depthai.VideoEncoderProperties.Profile.MJPEG)


        # self.rightEncoder = self.pipeline.createVideoEncoder()
        # self.rightEncoder.setDefaultProfilePreset(1, depthai.VideoEncoderProperties.Profile.MJPEG)


        # Create device and start VIO session
        self.device = depthai.Device(self.pipeline)
        self.vio_session = self.vio_pipeline.startSession(self.device)


        # self.left_q = self.device.getOutputQueue(name=self.left_enc.getOutputQueueName(),
        #                                          maxSize=4, blocking=False)
        # self.right_q = self.device.getOutputQueue(name=self.right_enc.getOutputQueueName(),
        #                                           maxSize=4, blocking=False)

        # Get encoder output queues

        # # Create IMU node
        # imu = self.pipeline.createIMU()
        #
        # # Enable IMU sensors: accelerometer and gyroscope
        # imu.enableIMUSensor(depthai.IMUSensor.ACCELEROMETER_RAW, 500)
        # imu.enableIMUSensor(depthai.IMUSensor.GYROSCOPE_RAW, 500)
        #
        # # Optionally: batch reports together
        # imu.setBatchReportThreshold(1)
        # imu.setMaxBatchReports(10)
        #
        # # Create XLinkOut for IMU
        # xout_imu = self.pipeline.createXLinkOut()
        # xout_imu.setStreamName("imu")
        # imu.out.link(xout_imu.input)


        # imu.enableIMUSensor(depthai.IMUSensor.ACCELEROMETER_RAW, 500)
        # imu.enableIMUSensor(depthai.IMUSensor.GYROSCOPE_RAW, 500)
        # imu.setBatchReportThreshold(1)
        # imu.setMaxBatchReports(10)
        # imu.out.link(imu_xout.input)


        self.imuQueue = self.device.getOutputQueue(name="xoutimu", maxSize=4, blocking=False)
        self.leftQueue = self.device.getOutputQueue(name="xoutleft", maxSize=4, blocking=False)
        self.rightQueue = self.device.getOutputQueue(name="xoutright", maxSize=4, blocking=False)
        # self.rightQueue = self.device.getOutputQueue(name=self.rightEncoder.getOutputQueueName(), maxSize=4, blocking=False)

        # Start timer
        self.timer = self.create_timer(0.01, self.processOutput)  # e.g., 100 Hz


    def processOutput(self):
        # Publish VIO
        while self.vio_session.hasOutput():
            out = self.vio_session.getOutput()
            vel = np.array([out.velocity.x, out.velocity.y, out.velocity.z])
            angul_vel = np.array([out.angularVelocity.x, out.angularVelocity.y, out.angularVelocity.z])
            pose = np.array([out.pose.position.x, out.pose.position.y, out.pose.position.z])
            orientation = np.array([out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w])
            self.publish_vio(vel, angul_vel, pose, orientation)



        # Publish compressed images
        if self.leftQueue.has():
            left_frame = self.leftQueue.get()
            # left_frame = self.leftQueue.tryGet()
            # print(left_frame)
            # if left_frame is not None:
            #     cv2.imshow('left', left_frame.getCvFrame())
            if left_frame is not None:
                self.publish_compressed_image(left_frame, self.left_img_pub, 'left_camera')

        if self.rightQueue.has():
            right_frame = self.rightQueue.get()

            if right_frame is not None:
                self.publish_compressed_image(right_frame, self.right_img_pub, 'right_camera')

        if self.imuQueue.has():
            print("hello")
            imu_frame = self.imuQueue.get()

            if imu_frame is not None:
                self.publish_imu(imu_frame, self.oakd_imu_pub, 'imu')




        # if self.rightQueue.has():
        #     right_frame = self.rightQueue.get()
        #     self.publish_compressed_image(right_frame, self.right_img_pub, 'right_camera')

    def publish_vio(self, vel, angul_vel, pose, orientation):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = pose[2]
        msg.pose.pose.orientation.x = orientation[0]
        msg.pose.pose.orientation.y = orientation[1]
        msg.pose.pose.orientation.z = orientation[2]
        msg.pose.pose.orientation.w = orientation[3]
        msg.twist.twist.linear.x = vel[0]
        msg.twist.twist.linear.y = vel[1]
        msg.twist.twist.linear.z = vel[2]
        msg.twist.twist.angular.x = angul_vel[0]
        msg.twist.twist.angular.y = angul_vel[1]
        msg.twist.twist.angular.z = angul_vel[2]
        self.vio_pub.publish(msg)

    def publish_compressed_image(self, frame, publisher, frame_id):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.format = "jpeg"
        # msg.data = frame.getData()
        msg.data = bytearray(frame.getData())
        # print("I'm here")
        publisher.publish(msg)

    def publish_imu(self, frame, publisher, frame_id):
        imu_packet = frame.get()
        print(imu_packet)
        if imu_packet is None:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id

        accel = imu_packet.acceleroMeter
        gyro = imu_packet.gyroscope

        # Linear acceleration (in m/s²)
        imu_msg.linear_acceleration.x = accel.x
        imu_msg.linear_acceleration.y = accel.y
        imu_msg.linear_acceleration.z = accel.z

        # Angular velocity (in rad/s)
        imu_msg.angular_velocity.x = gyro.x
        imu_msg.angular_velocity.y = gyro.y
        imu_msg.angular_velocity.z = gyro.z

        # Orientation unknown: disable it
        imu_msg.orientation_covariance[0] = -1.0

        publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VIO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()