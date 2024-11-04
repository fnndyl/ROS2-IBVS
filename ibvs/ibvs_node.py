#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import PadImCoord, VelCmd
import numpy as np
from numpy.linalg import pinv
from time import sleep

"""
This (and the offboard_node) will need SERIOUS modifications to be ready to actually fly in case of dropped messages, undetected corners, outliers, etc.
TODO: Case where drone gets close to platform and loses imaging of corners is currently a serious issue and will cause unknown behaviour
"""

class IBVSNode(Node):
    """Node for calculating V_cmd from image data"""

    def __init__(self):
        super().__init__('ibvs_node')

        # QoS policy for ArUco publisher
        qos_aruco = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            depth = 1
        )

        # Collect camera information
        self.camera_info = None
        self.get_camera_info()

        # Most temporary of temporary variables here
        self.offset = 80

        # Subscribe to identified image coordinate points
        # Current setup the IBVS velocity command is calculated for every /cam_im_coord received
        self.pad_im_coord_subscriber = self.create_subscription(
            PadImCoord, "/pad_im_coord", self.cam_im_coord_callback, qos_aruco
        )

        # Create velocity command publisher using custom message
        self.vel_cmd_publisher = self.create_publisher(
            VelCmd, "/ibvs_vel_cmd", 10
        )

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
            self.get_logger().info("Destroying /camera_info subscription")

    def cam_im_coord_callback(self, msg: PadImCoord):
        self.pad_im_coord = msg

        # Get velocity command for given callback for given ArUco identification
        v = self.ibvs_function(self.pad_im_coord, self.offset, self.camera_info)

        # Create and populate VelCmd message
        vel_cmd = VelCmd()
        vel_cmd.vx = v[0]
        vel_cmd.vy = v[1]
        vel_cmd.vz = v[2]
        vel_cmd.vyaw = v[3]

        self.vel_cmd_publisher.publish(vel_cmd)

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
        TODO: Ideally the orientation shouldnt matter, to decrease unnecessray yawing of the quadrotor - when first identified, run a routine to find the orientation of the goal points
        that minimizes the error, and then lock in that orientation as permanent
        For now, I am lazy

        TODO: Some better way of estimating the height to the target
        """

        # Detected points
        p_arr = [pad_im_coord.p1, pad_im_coord.p2, pad_im_coord.p3, pad_im_coord.p4]

        # Extract image properties
        height = camera_info.height
        width = camera_info.width

        # Camera intrinsics
        K = camera_info.k
        f = K[0]

        # Distance to target
        Z = 2.5 # Improve this estimate!

        # Create guide points (for now)
        u0 = round(width/2)
        v0 = round(height/2)

        # Create goal points from image center and offset
        goal_points = np.array([[-offset, +offset],
                                [-offset, -offset],
                                [+offset, -offset],
                                [+offset, +offset]]) + np.array([u0, v0])

        curr_points = np.array([[pad_im_coord.p1.x, pad_im_coord.p1.y],
                                [pad_im_coord.p2.x, pad_im_coord.p2.y],
                                [pad_im_coord.p3.x, pad_im_coord.p3.y],
                                [pad_im_coord.p4.x, pad_im_coord.p4.y]])

        # Create image Jacobian 
        J = np.zeros([2*4, 4])

        for i in range(1,4+1):

            u = p_arr[i-1].x - u0
            v = p_arr[i-1].y - v0

            # First row of Jacobian for given point
            J[2*(i-1)] = [-f/Z, 0, u/Z, v]

            # Second row of jacobian for given point
            J[2*(i-1) + 1] = [0, -f/Z, v/Z, -u]

        # Calculate commanded velocity based on image error
        gain = 0.35
        err = goal_points - curr_points

        # Need to reformat err matrix into single vector using flatten command
        v = np.matmul(pinv(J), err.flatten())
        v = np.matmul(gain * np.eye(4,4), v)
        self.get_logger().info(f"Publishing velocity cmd: {v}")
        return v

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