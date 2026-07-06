#!/usr/bin/env python3
"""Quick 'is it alive?' test for the TurtleBot 4.

Run this on the Raspberry Pi of the TurtleBot 4. It forces
ROS_DOMAIN_ID=4, starts a ROS 2 node and checks that the robot's
ROS graph is reachable by discovering the other nodes and topics.

Usage:
    python3 testalive.py
"""

import os

# Make sure we are on the right ROS domain before rclpy reads the env.
os.environ["ROS_DOMAIN_ID"] = "4"

import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("testalive")

    print(f"[testalive] ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID')}")
    print("[testalive] Discovering the ROS 2 graph (waiting a few seconds)...")

    # Spin briefly so discovery has time to find the other nodes.
    end = node.get_clock().now().nanoseconds + 5 * 1_000_000_000
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.5)

    # A node named "testalive" is always present, so filter it out.
    nodes = [n for n in node.get_node_names() if n != "testalive"]
    topics = node.get_topic_names_and_types()

    print(f"[testalive] Found {len(nodes)} other node(s):")
    for n in sorted(nodes):
        print(f"    - {n}")

    print(f"[testalive] Found {len(topics)} topic(s):")
    for name, types in sorted(topics):
        print(f"    - {name} {types}")

    if nodes:
        print("[testalive] TurtleBot 4 is ALIVE. ✓")
    else:
        print("[testalive] No other nodes found - is the robot running / "
              "on ROS_DOMAIN_ID 4?")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
