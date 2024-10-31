#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleLandDetected
from custom_msgs.msg import PadImCoord
from time import sleep
import numpy as np

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode"""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # Configure QoS profile for publishing and subscribing to uOrb PX4 topics streamed through MicroXRCEAgent
        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )

        # Create subscribers
        #self.vehicle_local_position_subscriber = self.create_subscription(
        #    VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos
        #)

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos
        )

        #self.vehicle_land_subscriber = self.create_subscription(
        #   VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.vehicle_land_callback, qos
        #)

        # Initialize class variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -3.0
        self.state_change_counter = 0

        # Create timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def arm(self) -> None:
        """Send an arm command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info('Arm command sent')

    def disarm(self) -> None:
        """Send a disarm command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self) -> None:
        """Switch to offboard control mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to velocity control offboard mode")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal(self) -> None:
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz, vyaw) -> None:
        """Publish velocity setpoint without position data"""
        msg = TrajectorySetpoint()
        msg.position = np.array([np.nan, np.nan, np.nan]).astype(np.float32)
        msg.yaw = float(np.nan)

        # Populate velocity setpoints
        msg.velocity = np.array([vx, vy, vz]).astype(np.float32)
        msg.yawspeed = float(vyaw)

        # Populate other required parameters
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocty setpoints {[vx, vy, vz, vyaw]}")

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def vehicle_status_callback(self, msg) -> None:
        """Update status of vehicle in flight"""
        self.vehicle_staus = msg

    def timer_callback(self) -> None:
        """Callback function for the timer - this is where the sausage is made"""
        self.publish_offboard_control_heartbeat_signal()

        # Create a variable to time state changes - this is very temporary logic
        if self.state_change_counter < 2000:
            self.state_change_counter += 1

        if self.state_change_counter == 20:
            self.engage_offboard_mode()
            sleep(1)
            self.arm()

        vel_switch_count = 200

        #if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        if self.state_change_counter > 20 and self.state_change_counter < vel_switch_count:
            # Default values correspond to 0,0,self.takeoff_height
            self.publish_position_setpoint(0.0,0.0,self.takeoff_height)

        if self.state_change_counter > 20 and self.state_change_counter < vel_switch_count:
            self.get_logger().info(f"Velocity control countdown: {self.state_change_counter} < {vel_switch_count}")

        if self.state_change_counter >= vel_switch_count:
            self.publish_velocity_setpoint(0,0,0,0)

def main(args=None) -> None:
    print('Starting offboard mode node')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
