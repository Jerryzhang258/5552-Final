#!/usr/bin/env python3
"""Bypass YOLO: directly publish /target_point with known cube position.
The YOLO model can't detect our placeholder cubes, so we send coordinates directly.

Usage:
  python3 scripts/direct_pick.py            # pick red_cube (default)
  python3 scripts/direct_pick.py green 1    # pick green_cube, drop in box 1
  python3 scripts/direct_pick.py yellow 2   # pick yellow_cube, drop in box 2
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

# Cube positions in ROBOT BASE frame (approximate, may need calibration)
# The ur5 base is at roughly (-3.854, 0.028, table_z) in Isaac world coords
# Cubes are at: green(-4.004, 0.019, 0.479), red(-3.854, 0.139, 0.479), yellow(-3.704, 0.019, 0.479)
# Relative to ur5 base: x = cube_world_x - ur5_x, y = cube_world_y - ur5_y
UR5_X = -3.854
UR5_Y = 0.028

CUBES = {
    "green":  (-4.004, 0.019),
    "red":    (-3.854, 0.139),
    "yellow": (-3.704, 0.019),
}


def main():
    cube_name = sys.argv[1] if len(sys.argv) > 1 else "red"
    box = sys.argv[2] if len(sys.argv) > 2 else "1"

    if cube_name not in CUBES:
        print(f"Unknown cube: {cube_name}. Available: {list(CUBES.keys())}")
        return

    wx, wy = CUBES[cube_name]
    # Convert to robot base frame
    rx = wx - UR5_X
    ry = wy - UR5_Y
    yaw = 0.0

    print(f"Direct pick: {cube_name}_cube")
    print(f"  World pos: ({wx:.3f}, {wy:.3f})")
    print(f"  Robot base pos: ({rx:.3f}, {ry:.3f})")
    print(f"  Box: {box}")

    rclpy.init()
    node = Node("direct_pick_publisher")
    pub = node.create_publisher(Float64MultiArray, "/target_point", 10)

    # Wait for subscriber
    time.sleep(1.0)

    msg = Float64MultiArray()
    msg.data = [rx, ry, yaw, float(box)]
    pub.publish(msg)
    node.get_logger().info(f"Published /target_point: {msg.data}")

    # Publish a few times to make sure it's received
    for _ in range(5):
        time.sleep(0.5)
        pub.publish(msg)

    print("Done! The pick-place node should now move the arm.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
