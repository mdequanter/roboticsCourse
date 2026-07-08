#!/usr/bin/env python3
"""Simple scripted drive for the TurtleBot 4.

Runs one fixed sequence and then exits:

    1. undock
    2. drive 0.5 m forward
    3. turn left 90 degrees
    4. turn left another 70 degrees
    5. dock again

It uses the irobot_create_msgs *actions* (not /cmd_vel) so every move is
closed-loop and precise. Run this ON the Raspberry Pi of the TurtleBot 4
with ROS 2 sourced:

    source /opt/ros/humble/setup.bash
    python3 drivePattern.py
"""

import os

# Force the correct ROS domain before rclpy reads the environment.
os.environ["ROS_DOMAIN_ID"] = "4"

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import Undock, Dock, DriveDistance, RotateAngle


class DrivePattern(Node):
    def __init__(self):
        super().__init__("drive_pattern")
        self._undock = ActionClient(self, Undock, "/undock")
        self._dock = ActionClient(self, Dock, "/dock")
        self._drive = ActionClient(self, DriveDistance, "/drive_distance")
        self._rotate = ActionClient(self, RotateAngle, "/rotate_angle")

    # ---- generic helper: send a goal and block until it finishes ----------
    def _run(self, client, goal, description):
        self.get_logger().info(f"→ {description}")
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"   action server not available: {description}")
            return False

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"   goal rejected: {description}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"   done: {description}")
        return True

    # ---- individual moves -------------------------------------------------
    def undock(self):
        return self._run(self._undock, Undock.Goal(), "undock")

    def dock(self):
        return self._run(self._dock, Dock.Goal(), "dock")

    def drive_forward(self, distance_m, speed=0.15):
        goal = DriveDistance.Goal()
        goal.distance = float(distance_m)
        goal.max_translation_speed = float(speed)
        return self._run(self._drive, goal, f"drive {distance_m} m forward")

    def turn_left(self, degrees, speed=0.8):
        # ROS convention: positive angle = counter-clockwise = left turn.
        goal = RotateAngle.Goal()
        goal.angle = math.radians(degrees)
        goal.max_rotation_speed = float(speed)
        return self._run(self._rotate, goal, f"turn left {degrees}°")

    # ---- the full sequence ------------------------------------------------
    def run_sequence(self):
        steps = [
            lambda: self.undock(),
            lambda: self.drive_forward(0.5),
            lambda: self.turn_left(90),
            lambda: self.turn_left(70),
            lambda: self.dock(),
        ]
        for step in steps:
            if not step():
                self.get_logger().error("Sequence aborted.")
                return
        self.get_logger().info("Sequence complete. 🐢")


def main():
    rclpy.init()
    node = DrivePattern()
    try:
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
