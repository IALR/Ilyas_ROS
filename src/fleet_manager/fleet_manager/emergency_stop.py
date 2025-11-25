#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fleet_interfaces.msg import FleetCommand


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        # Publisher
        self.pub = self.create_publisher(FleetCommand, 'fleet_command', 10)

        # Send the emergency stop immediately
        self.send_stop()

    def send_stop(self):
        cmd = FleetCommand()
        cmd.command_id = "EMERGENCY_STOP"
        cmd.command_type = "stop"
        cmd.priority = 10

        # IMPORTANT — your message uses target_robot_ids, NOT robot_ids
        cmd.target_robot_ids = []      # This targets ALL robots
        cmd.robot_type_filter = ""     # No type filter
        cmd.task_description = ""      # Not needed

        self.pub.publish(cmd)
        self.get_logger().info("🚨 EMERGENCY STOP SENT TO ALL ROBOTS")

        # Ensure message goes out
        rclpy.spin_once(self, timeout_sec=0.3)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    EmergencyStop()


if __name__ == "__main__":
    main()
