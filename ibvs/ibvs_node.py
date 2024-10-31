#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class IBVSNode(Node):
    """Node for calculating V_cmd from image data"""
    

def main(args=None):
    print("Starting IBVS node")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)