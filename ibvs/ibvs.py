import numpy as np
from sensor_msgs.msg import CameraInfo

def ibvs(pad_im_coord, des_coords, camera_info: CameraInfo):
    """
    Function to find velocity CMD from pixel error between image plane coordinates of detected ArUco tag corners, and defined desired corner positions.
    Does not command roll or pitch as theses are not directly controlled by PX4

    Parameters:
    pad_im_coord (PadImCoord.msg) : ROS2 message containing image plane coordinates of detected corners
    des_coords (np.arr)           : 4x2 np.arr containing desired image plane coordiantes for detected corners

    Returns:
    v_cmd                         : Velocity command in format [vx_cmd, vy_cmd, vz_cmd, vyaw_cmd]
    """

    Z = 2.5 # This is hardcoded for now - but this is TEMPORARY! A better solution is needed here.
    
    # Extract image dimensions
    height = camera_info.height
    width = camera_info.width

    # Camera intrinsics
    K = camera_info.k
    