#!/usr/bin/env python3
"""
ROS2 bridge: subscribes to MoveIt's trajectory controller output,
forwards joint positions to /isaac_joint_commands for Isaac Sim's OmniGraph.

Run alongside MoveIt demo:
  ros2 run ur5_moveit_config demo.launch.py  (terminal 1)
  python3 scripts/joint_command_bridge.py     (terminal 2)
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy


class JointCommandBridge(Node):
    def __init__(self):
        super().__init__("joint_command_bridge")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.pub = self.create_publisher(JointState, "/isaac_joint_commands", qos)

        # MoveIt's JointTrajectoryController publishes here
        self.sub_traj = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.traj_callback,
            qos,
        )

        # Also listen to /joint_states from mock controller (fallback)
        self.sub_js = self.create_subscription(
            JointState,
            "/joint_states",
            self.js_callback,
            qos,
        )

        self.latest_joint_state = None
        self.get_logger().info("Bridge started: /arm_controller/joint_trajectory -> /isaac_joint_commands")

    def traj_callback(self, msg: JointTrajectory):
        """Forward the last point of each trajectory to Isaac Sim."""
        if not msg.points:
            return
        last_pt = msg.points[-1]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(msg.joint_names)
        js.position = list(last_pt.positions)
        if last_pt.velocities:
            js.velocity = list(last_pt.velocities)
        self.pub.publish(js)
        self.get_logger().info(
            f"Forwarded trajectory ({len(msg.points)} pts) -> /isaac_joint_commands"
        )

    def js_callback(self, msg: JointState):
        """Track current joint states (for reference/debug)."""
        self.latest_joint_state = msg


def main():
    rclpy.init()
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
