# UR5 Demo: Isaac Sim Script Editor only (no ROS). Use ArticulationController or USD drives.
# Usage: Open SPARC_ur5_fixed.usd -> Window -> Script Editor -> paste full script -> Run -> Play.
# Motion: shoulder right -> center -> lift shoulder+elbow -> back to initial, ~16s per loop.
# Original repo demo: Isaac Play + RVIZ + MoveIt + ur5_pick_place (ROS sends joint commands).

import omni.usd
from pxr import Usd, UsdPhysics
import math

# 关节路径（USD 直接写入时的 fallback）
JOINTS = [
    "/World/ur5/joints/shoulder_pan_joint",
    "/World/ur5/joints/shoulder_lift_joint",
    "/World/ur5/joints/elbow_joint",
    "/World/ur5/joints/wrist_1_joint",
    "/World/ur5/joints/wrist_2_joint",
    "/World/ur5/joints/wrist_3_joint",
]

# 初始姿态 (rad)，近似 arm_ready
INITIAL = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# 演示动作幅度（弧度）
SHOULDER_PAN_LEFT = 0.0
SHOULDER_PAN_RIGHT = 1.2
SHOULDER_LIFT_UP = -2.4
ELBOW_UP = 2.2

def _lerp(a, b, t):
    return a + (b - a) * t

def _ease(t):
    return t * t * (3.0 - 2.0 * t)  # smoothstep

import time
_demo_duration = 16.0
_demo_state = {"t": 0.0, "was_playing": False, "last_time": 0.0, "articulation": None, "use_controller": False}


def _get_targets(t_sec):
    # 根据时间(秒)返回 6 个关节目标 (rad)。
    t = (t_sec % _demo_duration) / _demo_duration
    if t < 0.25:
        s = _ease(t / 0.25)
        return [_lerp(SHOULDER_PAN_LEFT, SHOULDER_PAN_RIGHT, s)] + list(INITIAL[1:])
    elif t < 0.5:
        s = _ease((t - 0.25) / 0.25)
        return [_lerp(SHOULDER_PAN_RIGHT, SHOULDER_PAN_LEFT, s)] + list(INITIAL[1:])
    elif t < 0.75:
        s = _ease((t - 0.5) / 0.25)
        return [INITIAL[0], _lerp(INITIAL[1], SHOULDER_LIFT_UP, s), _lerp(INITIAL[2], ELBOW_UP, s)] + list(INITIAL[3:])
    else:
        s = _ease((t - 0.75) / 0.25)
        return [INITIAL[0], _lerp(SHOULDER_LIFT_UP, INITIAL[1], s), _lerp(ELBOW_UP, INITIAL[2], s)] + list(INITIAL[3:])


def _apply_targets_usd(targets):
    # Fallback: 直接写 USD Drive targetPosition（与 isaac_test_direct_drive 一致）。
    stage = omni.usd.get_context().get_stage()
    if not stage or len(targets) < 6:
        return
    for i, path in enumerate(JOINTS):
        if i >= len(targets):
            break
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive_api:
            continue
        target_attr = drive_api.GetTargetPositionAttr()
        if target_attr:
            target_attr.Set(float(targets[i]))


def _ensure_articulation():
    # 首次 Play 时尝试创建 SingleArticulation 并 initialize；失败则用 USD fallback。
    if _demo_state["articulation"] is not None:
        return _demo_state["use_controller"]
    if _demo_state.get("controller_tried"):
        return False
    _demo_state["controller_tried"] = True
    try:
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        import numpy as np
        # 本场景 articulation 根可能是 /World/ur5 或 /World/ur5/root_joint
        for path in ["/World/ur5", "/World/ur5/root_joint"]:
            prim = SingleArticulation(prim_path=path, name="ur5_demo")
            try:
                prim.initialize()
                _demo_state["articulation"] = prim
                _demo_state["ArticulationAction"] = ArticulationAction
                _demo_state["np"] = np
                _demo_state["use_controller"] = True
                print(f"[Demo] 使用 ArticulationController，prim_path={path}")
                return True
            except Exception:
                continue
    except Exception as e:
        print(f"[Demo] ArticulationController 不可用，改用 USD 写入: {e}")
    return False


def _apply_targets_controller(targets):
    # 用官方 ArticulationController 发送关节位置（与 ROS 图同路径）。
    prim = _demo_state.get("articulation")
    if not prim or len(targets) < 6:
        return
    try:
        ArticulationAction = _demo_state["ArticulationAction"]
        np = _demo_state["np"]
        action = ArticulationAction(joint_positions=np.array(targets, dtype=float))
        prim.apply_action(action)
    except Exception:
        pass


def _on_tick(_):
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    playing = timeline.is_playing()
    if not playing:
        _demo_state["was_playing"] = False
        return
    if not _demo_state["was_playing"]:
        _demo_state["t"] = 0.0
        _demo_state["last_time"] = timeline.get_current_time()
        _demo_state["was_playing"] = True
        _ensure_articulation()
    current_time = timeline.get_current_time()
    dt = current_time - _demo_state["last_time"]
    _demo_state["last_time"] = current_time
    if dt <= 0 or dt > 0.5:
        dt = 1.0 / 60.0
    _demo_state["t"] += dt
    targets = _get_targets(_demo_state["t"])
    if _demo_state["use_controller"]:
        _apply_targets_controller(targets)
    else:
        _apply_targets_usd(targets)


import omni.kit.app
_stream = omni.kit.app.get_app().get_update_event_stream()
_sub = _stream.create_subscription_to_pop(_on_tick)

print("=" * 60)
print("UR5 Demo 已启动：点 Play 后机械臂会自动做演示动作。")
print("顺序：肩右转 -> 回正 -> 抬肩抬肘 -> 回初始（约 16 秒一轮）。")
print("参考原仓库: github.com/sahilrajpurkar03/nlp-pnp-robotic-arm")
print("=" * 60)
