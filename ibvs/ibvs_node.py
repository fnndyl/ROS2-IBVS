#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import PadImCoord
from typing import List
from ibvs import ibvs_function
from time import sleep

class IBVSNode(Node):
    """Node for calculating V_cmd from image data"""

    def __init__(self):
        super().__init__('ibvs_node')

        # Collect camera information
        self.camera_info = None
        self.get_camera_info()

        # Subscribe to identified image coordinate points
        self.create_subscription(
            PadImCoord, "/cam_im_coord", self.cam_im_coord_callback, 10
        )

        # Create timer to handle IBVS
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

    def cam_im_coord_callback(self, msg: PadImCoord):
        self.pad_im_coord = msg

    def ibvs_function(self, pad_im_coord: PadImCoord, offset: int, camera_info: CameraInfo):
            """
            Function to find velocity CMD from pixel error between image plane coordinates of detected ArUco tag corners, and defined desired corner positions.
            Does not command roll or pitch as theses are not directly controlled by PX4

            Parameters:
            pad_im_coord (PadImCoord.msg) : ROS2 message containing image plane coordinates of detected corners
            des_coords (np.arr)           : 4x2 np.arr containing desired image plane coordiantes for detected corners

            Returns:
            v_cmd                         : Velocity command in format [vx_cmd, vy_cmd, vz_cmd, vyaw_cmd]
            """

            """
            TO DO 
            Ideally the orientation shouldnt matter, to decrease unnecessray yawing of the quadrotor - when first identified, run a routine to find the orientation of the goal points
            that minimizes the error, and then lock in that orientation as permanent
            For now, I am lazy
            """

            # Extract image properties
            height = camera_info.height
            width = camera_info.width

            # Camera intrinsics
            K = camera_info.k

            # Extract points
            p1, p2, p3, p4 = pad_im_coord

            # Create guide points (for now)
            u0 = round(width/2)
            v0 = round(height/2)




    def timer_callback(self):
        if self.camera_info is not None:
            
            offset = 80
            v_cmd = ibvs_function(self.pad_im_coord, offset, self.camera_info)

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