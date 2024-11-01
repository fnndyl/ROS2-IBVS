from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import PadImCoord
from typing import List

import numpy as np
from numpy.linalg import pinv

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
            print("Goal Points:")
            print(goal_points)

            curr_points = np.array([[pad_im_coord.p1.x, pad_im_coord.p1.y],
                                    [pad_im_coord.p2.x, pad_im_coord.p2.y],
                                    [pad_im_coord.p3.x, pad_im_coord.p3.y],
                                    [pad_im_coord.p4.x, pad_im_coord.p4.y]])
            
            print("Current Points:")
            print(curr_points)

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
            print("Error between desired and current points:")
            print(err)

            # Need to reformat err matrix into single vector using flatten command
            v = np.matmul(pinv(J), err.flatten())
            v = np.matmul(gain * np.eye(4,4), v)
            print(v)

camera_info = CameraInfo()
camera_info.width = 1280
camera_info.height = 960
camera_info.k = np.array([540, 0, 640, 0, 540, 480, 0, 0, 1]).astype(float)

p1 = Point()
p1.x = 589.0
p1.y = 532.0
p1.z = 0.0


p2 = Point()
p2.x = 586.0
p2.y = 428.0
p2.z = 0.0

p3 = Point()
p3.x = 690.0
p3.y = 425.0
p3.z = 0.0

p4 = Point()
p4.x = 693.0
p4.y = 529.0
p4.z = 0.0

pad_im_coord = PadImCoord()
pad_im_coord.p1 = p1
pad_im_coord.p2 = p2
pad_im_coord.p3 = p3
pad_im_coord.p4 = p4

offset = 80

ibvs_function(0, pad_im_coord=pad_im_coord, offset=offset, camera_info=camera_info)