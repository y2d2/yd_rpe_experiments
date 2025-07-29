import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
import depthai
import spectacularAI
import numpy as np

class VIO(Node):
    def __init__(self):
        super().__init__('spectacularAI_vio_node')

        # Create pipeline and VIO session
        self.pipeline = depthai.Pipeline()
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
        self.vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline)
        self.vio_pub = self.create_publisher(Odometry, 'vio', 10)
        self.left_img_pub = self.create_publisher(CompressedImage, 'left/image/compressed', 10)
        # self.right_img_pub = self.create_publisher(CompressedImage, 'right/image/compressed', 10)
        left_xout = self.vio_pipeline.monoLeft.out
        right_xout = self.vio_pipeline.monoRight.out
        self.leftEncoder = self.pipeline.createVideoEncoder()
        self.leftEncoder.setDefaultProfilePreset(1, depthai.VideoEncoderProperties.Profile.MJPEG)


        self.rightEncoder = self.pipeline.createVideoEncoder()
        self.rightEncoder.setDefaultProfilePreset(1, depthai.VideoEncoderProperties.Profile.MJPEG)


        # Create device and start VIO session
        self.device = depthai.Device(self.pipeline)
        self.vio_session = self.vio_pipeline.startSession(self.device)

        # self.left_q = self.device.getOutputQueue(name=self.left_enc.getOutputQueueName(),
        #                                          maxSize=4, blocking=False)
        # self.right_q = self.device.getOutputQueue(name=self.right_enc.getOutputQueueName(),
        #                                           maxSize=4, blocking=False)

        # Get encoder output queues
        self.leftQueue = self.device.getOutputQueue(name=self.leftEncoder.getOutputQueueName(), maxSize=4, blocking=False)
        self.rightQueue = self.device.getOutputQueue(name=self.rightEncoder.getOutputQueueName(), maxSize=4, blocking=False)

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
        # if self.leftQueue.has():
        #     left_frame = self.leftQueue.get()
        #     self.publish_compressed_image(left_frame, self.left_img_pub, 'left_camera')

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
        msg.data = frame.getData()
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VIO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()