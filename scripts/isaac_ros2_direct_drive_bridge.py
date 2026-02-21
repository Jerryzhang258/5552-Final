"""
Run ONCE in Isaac Sim Script Editor (SPARC_ur5_fixed.usd open, simulation PLAYING).
Subscribes to /isaac_joint_commands and writes positions directly to USD joint drives,
bypassing the ArticulationController node. Uses the same code path as isaac_test_direct_drive.py.

After running, keep Isaac Sim playing and use Plan & Execute in RVIZ.
"""

import omni.usd
from pxr import Usd, UsdPhysics

# 默认关节路径（UR5）；若场景不同可改 ROBOT_ROOT
ROBOT_ROOT = "/World/ur5"
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
# 若某关节与 RVIZ 方向相反，设为 -1（例如 "shoulder_lift_joint": -1）
JOINT_SIGN = {}

# Latest received command
_latest_names = []
_latest_positions = []
_has_new_data = False
_apply_count = 0

# 缓存：关节名 -> prim 路径（首次收到消息时按场景自动查找）
_joint_path_cache = {}


def _find_joint_paths(stage):
    """在 ROBOT_ROOT 下按 prim 名查找关节路径，兼容不同 USD 结构。"""
    global _joint_path_cache
    from pxr import Usd
    root = stage.GetPrimAtPath(ROBOT_ROOT)
    if not root or not root.IsValid():
        return
    for prim in Usd.PrimRange(root):
        name = prim.GetName()
        if name in JOINT_NAMES and name not in _joint_path_cache:
            _joint_path_cache[name] = str(prim.GetPath())
    return _joint_path_cache


def _on_joint_command(msg):
    global _latest_names, _latest_positions, _has_new_data
    _latest_names = list(msg.name) if msg.name else []
    _latest_positions = list(msg.position) if msg.position else []
    _has_new_data = True


def _apply_to_usd():
    global _has_new_data, _apply_count, _joint_path_cache
    if not _has_new_data or not _latest_names or not _latest_positions:
        return
    _has_new_data = False
    stage = omni.usd.get_context().get_stage()
    if not stage:
        return
    if not _joint_path_cache:
        _find_joint_paths(stage)
        if _joint_path_cache:
            print(f"[DirectDrive] Found joints: {list(_joint_path_cache.keys())}")
    applied = 0
    for i, name in enumerate(_latest_names):
        if i >= len(_latest_positions):
            break
        path = _joint_path_cache.get(name) or f"{ROBOT_ROOT}/joints/{name}"
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive_api:
            continue
        target_attr = drive_api.GetTargetPositionAttr()
        if target_attr:
            val = float(_latest_positions[i])
            if name in JOINT_SIGN:
                val *= JOINT_SIGN[name]
            target_attr.Set(val)
            applied += 1
    if applied > 0:
        _apply_count += 1
        if _apply_count <= 3 or _apply_count % 20 == 0:
            print(f"[DirectDrive] Applied to {applied} joints (msg #{_apply_count})")


def _on_update(_):
    try:
        rclpy.spin_once(_ros_node, timeout_sec=0)
    except Exception:
        pass
    _apply_to_usd()


# -----------------------------------------------------------------------------
# Setup ROS2 subscriber (use Isaac Sim's internal rclpy)
# -----------------------------------------------------------------------------
try:
    import rclpy
    from sensor_msgs.msg import JointState
except ImportError as e:
    print("[ERROR] rclpy not available. Enable isaacsim.ros2.bridge and run with simulation PLAYING.")
    raise SystemExit

if not rclpy.ok():
    rclpy.init()

_ros_node = rclpy.create_node("isaac_direct_drive_bridge")
_ros_node.create_subscription(
    JointState,
    "/isaac_joint_commands",
    _on_joint_command,
    10,
)

# Subscribe to app update (run every frame)
import omni.kit.app
_stream = omni.kit.app.get_app().get_update_event_stream()
_sub = _stream.create_subscription_to_pop(_on_update)

print("=" * 60)
print("Direct drive bridge ACTIVE: /isaac_joint_commands -> USD joint targets")
print("Keep this session running. Use Plan & Execute in RVIZ.")
print("To stop: run stream.remove_subscription(_sub) and destroy _ros_node")
print("=" * 60)
