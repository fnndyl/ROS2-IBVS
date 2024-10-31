#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CameraInfo
from time import sleep

class IBVSNode(Node):
    """Node for calculating V_cmd from image data"""

    def __init__(self):
        super().__init__('ibvs_node')

        # Collect camera information
        self.camera_info = None
        self.get_camera_info()
    
        self.create_timer(0.1, self.timer_callback)

    def get_camera_info(self):
        """ Get camera_info msg and destroy subscriber for neatness """
        # Create camera info message
        self.get_logger().info("Getting camera info")
        self.camera_info = None

        # Create camera info subscriber
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "/camera_info", self.camera_info_callback, 10
        )
        sleep(3)

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info("Camera info callback triggered")
            self.get_logger().info(f"Camera intrinsics {msg.k}")
            self.camera_info = msg
            self.destroy_subscription(self.camera_info_subscriber)

    def timer_callback(self):
        if self.camera_info is not None:
            pass

def main(args=None):
    print("Starting IBVS node")
    rclpy.init(args=args)
    ibvs_node = IBVSNode()
    rclpy.spin(ibvs_node)
    ibvs_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)