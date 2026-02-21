#!/usr/bin/env python3
# 轨迹中继：MoveIt (action) -> /isaac_joint_commands。100Hz 插值发布，Isaac 与 RVIZ 同步且丝滑。

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

# 当前轨迹：(joint_names, points, goal_handle, start_time)，start_time 为墙钟
_traj_queue = []
_traj_start_time = 0.0  # 当前条目的开始时间


def _interpolate(points, t):
    """t 为 time_from_start（秒），返回 (positions, velocities) 的线性插值。"""
    if not points:
        return None, None
    if t <= points[0][2]:
        return list(points[0][0]), list(points[0][1]) if points[0][1] else None
    if t >= points[-1][2]:
        return list(points[-1][0]), list(points[-1][1]) if points[-1][1] else None
    for i in range(len(points) - 1):
        t0, t1 = points[i][2], points[i + 1][2]
        if t0 <= t <= t1:
            if t1 <= t0:
                s = 1.0
            else:
                s = (t - t0) / (t1 - t0)
            p0, p1 = points[i][0], points[i + 1][0]
            pos = [p0[j] + s * (p1[j] - p0[j]) for j in range(len(p0))]
            v0, v1 = points[i][1], points[i + 1][1]
            vel = None
            if v0 and v1:
                vel = [v0[j] + s * (v1[j] - v0[j]) for j in range(len(v0))]
            return pos, vel
    return list(points[-1][0]), list(points[-1][1]) if points[-1][1] else None


class TrajectoryToIsaacRelay(Node):
    def __init__(self):
        super().__init__("trajectory_to_isaac_relay")
        global _traj_queue, _traj_start_time
        _traj_queue = []
        _traj_start_time = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(JointState, "/isaac_joint_commands", qos)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            self._execute_callback,
        )
        # 100 Hz：按轨迹时间线性插值发布，Isaac 收到丝滑连续的目标
        self._timer = self.create_timer(0.01, self._timer_callback)
        self.get_logger().info(
            "Relay: arm_controller/follow_joint_trajectory -> /isaac_joint_commands (100Hz interpolated)"
        )

    def _timer_callback(self):
        global _traj_queue, _traj_start_time
        if not _traj_queue:
            return
        joint_names, points, goal_handle = _traj_queue[0]
        t_now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = t_now - _traj_start_time
        duration = points[-1][2] if points else 0.0
        if elapsed >= duration:
            _traj_queue.pop(0)
            if goal_handle is not None:
                try:
                    goal_handle.succeed()
                except Exception as e:
                    # 目标可能已被客户端 abort（如 MoveIt 超时），忽略
                    self.get_logger().debug(f"Goal already finished: {e}")
            if _traj_queue:
                _traj_start_time = self.get_clock().now().nanoseconds * 1e-9
            return
        pos, vel = _interpolate(points, elapsed)
        if pos is None:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = joint_names
        js.position = pos
        if vel:
            js.velocity = vel
        self.pub.publish(js)

    def _execute_callback(self, goal_handle):
        global _traj_queue, _traj_start_time
        traj = goal_handle.request.trajectory
        if not traj.points:
            self.get_logger().warn("Empty trajectory, succeeding anyway.")
            goal_handle.succeed()
            return FollowJointTrajectory.Result()
        joint_names = list(traj.joint_names)
        points = []
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            vel = list(pt.velocities) if pt.velocities else []
            points.append((list(pt.positions), vel, t))
        _traj_queue.append((joint_names, points, goal_handle))
        _traj_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"Relay: queued trajectory ({len(points)} pts, {points[-1][2]:.2f}s), 100Hz interpolated stream"
        )
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = TrajectoryToIsaacRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
