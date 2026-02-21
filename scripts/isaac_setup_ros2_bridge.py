"""
Run in Isaac Sim Script Editor (SPARC_ur5_fixed.usd open, simulation PLAYING).
Creates a robust ROS2 bridge Action Graph for UR5 + MoveIt topic_based_ros2_control.

Topics:
  - Publishes:  /isaac_joint_states   (Isaac Sim -> MoveIt)
  - Subscribes: /isaac_joint_commands  (MoveIt -> Isaac Sim)
"""

import omni.usd
import omni.graph.core as og
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
assert stage, "No stage open!"

GRAPH_PATH = "/World/ROS2_ActionGraph"

# ======================================================================
# STEP 0: Find the Articulation Root automatically
# ======================================================================
print("=" * 60)
print("STEP 0: Locating ArticulationRootAPI under /World/ur5 ...")
art_root_path = None
ur5 = stage.GetPrimAtPath("/World/ur5")
if ur5 and ur5.IsValid():
    for p in Usd.PrimRange(ur5):
        schemas = [str(s) for s in p.GetAppliedSchemas()]
        if any("ArticulationRootAPI" in s for s in schemas):
            art_root_path = str(p.GetPath())
            print(f"  [OK] ArticulationRootAPI found at: {art_root_path}")
            break

if not art_root_path:
    print("  [WARN] No ArticulationRootAPI found! Applying it to /World/ur5 ...")
    ur5_prim = stage.GetPrimAtPath("/World/ur5")
    if not ur5_prim or not ur5_prim.IsValid():
        print("  [ERROR] /World/ur5 does not exist! Aborting.")
        raise SystemExit
    UsdPhysics.ArticulationRootAPI.Apply(ur5_prim)
    art_root_path = "/World/ur5"
    print(f"  [OK] Applied ArticulationRootAPI to {art_root_path}")

ROBOT_PRIM = art_root_path

# ======================================================================
# STEP 1: Add missing wrist camera to prevent warning spam
# ======================================================================
print("\nSTEP 1: Checking wrist camera ...")
cam_path = "/World/ur5/wrist_3_link/detection_camera_01/Camera_SG2_OX03CC_5200_GMSL2_H60YA"
cam_parent = "/World/ur5/wrist_3_link/detection_camera_01"
if not (stage.GetPrimAtPath(cam_path) and stage.GetPrimAtPath(cam_path).IsValid()):
    if not (stage.GetPrimAtPath(cam_parent) and stage.GetPrimAtPath(cam_parent).IsValid()):
        UsdGeom.Xform.Define(stage, cam_parent)
    UsdGeom.Camera.Define(stage, cam_path)
    print("  [OK] Added missing wrist camera")
else:
    print("  [OK] Camera already exists")

# ======================================================================
# STEP 2: Remove old graph
# ======================================================================
print("\nSTEP 2: Removing old Action Graph ...")
old = stage.GetPrimAtPath(GRAPH_PATH)
if old and old.IsValid():
    stage.RemovePrim(GRAPH_PATH)
    print("  [OK] Removed old graph")
else:
    print("  [OK] No old graph found")

# ======================================================================
# STEP 3: Create Action Graph with correct node types
# ======================================================================
print("\nSTEP 3: Creating Action Graph ...")

node_configs = [
    {
        "label": "isaacsim (v4.5+)",
        "tick": "omni.graph.action.OnPlaybackTick",
        "pub": "isaacsim.ros2.bridge.ROS2PublishJointState",
        "sub": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
        "ctrl": "isaacsim.core.nodes.IsaacArticulationController",
        "simtime": "isaacsim.core.nodes.IsaacReadSimulationTime",
    },
    {
        "label": "omni.isaac (v4.0)",
        "tick": "omni.graph.action.OnPlaybackTick",
        "pub": "omni.isaac.ros2_bridge.ROS2PublishJointState",
        "sub": "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
        "ctrl": "omni.isaac.core_nodes.IsaacArticulationController",
        "simtime": "omni.isaac.core_nodes.IsaacReadSimulationTime",
    },
]

keys = og.Controller.Keys
graph_ok = False

for cfg in node_configs:
    print(f"  Trying {cfg['label']} ...")
    try:
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", cfg["tick"]),
                    ("ReadSimTime", cfg["simtime"]),
                    ("PubJointState", cfg["pub"]),
                    ("SubJointState", cfg["sub"]),
                    ("ArtController", cfg["ctrl"]),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "ReadSimTime.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubJointState.inputs:execIn"),
                    ("OnTick.outputs:tick", "SubJointState.inputs:execIn"),
                    ("OnTick.outputs:tick", "ArtController.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PubJointState.inputs:timeStamp"),
                    ("SubJointState.outputs:jointNames", "ArtController.inputs:jointNames"),
                    ("SubJointState.outputs:positionCommand", "ArtController.inputs:positionCommand"),
                ],
            },
        )
        print(f"  [OK] Graph created with {cfg['label']}")
        graph_ok = True
        break
    except Exception as e:
        err_msg = str(e)
        print(f"  [WARN] {cfg['label']} failed: {err_msg}")
        bad = stage.GetPrimAtPath(GRAPH_PATH)
        if bad and bad.IsValid():
            stage.RemovePrim(GRAPH_PATH)
        if "ReadSimTime" in err_msg or "simtime" in err_msg.lower():
            print("  Retrying without ReadSimTime ...")
            try:
                (graph, nodes, _, _) = og.Controller.edit(
                    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
                    {
                        keys.CREATE_NODES: [
                            ("OnTick", cfg["tick"]),
                            ("PubJointState", cfg["pub"]),
                            ("SubJointState", cfg["sub"]),
                            ("ArtController", cfg["ctrl"]),
                        ],
                        keys.CONNECT: [
                            ("OnTick.outputs:tick", "PubJointState.inputs:execIn"),
                            ("OnTick.outputs:tick", "SubJointState.inputs:execIn"),
                            ("OnTick.outputs:tick", "ArtController.inputs:execIn"),
                            ("SubJointState.outputs:jointNames", "ArtController.inputs:jointNames"),
                            ("SubJointState.outputs:positionCommand", "ArtController.inputs:positionCommand"),
                        ],
                    },
                )
                print(f"  [OK] Graph created without ReadSimTime ({cfg['label']})")
                graph_ok = True
                break
            except Exception as e2:
                print(f"  [WARN] Also failed: {e2}")
                bad2 = stage.GetPrimAtPath(GRAPH_PATH)
                if bad2 and bad2.IsValid():
                    stage.RemovePrim(GRAPH_PATH)

if not graph_ok:
    print("\n[ERROR] All methods failed. Please create the graph manually.")
    raise SystemExit

# ======================================================================
# STEP 4: Set node attributes
# ======================================================================
print(f"\nSTEP 4: Setting attributes (targetPrim = {ROBOT_PRIM}) ...")


def try_set(attr_path, value, label=""):
    try:
        og.Controller.set(og.Controller.attribute(attr_path), value)
        print(f"  [OK] {label}: {attr_path} = {value}")
        return True
    except Exception as e:
        print(f"  [WARN] {label}: {attr_path} -> {e}")
        return False


# PubJointState target
try_set(f"{GRAPH_PATH}/PubJointState.inputs:targetPrim", [ROBOT_PRIM], "PubJointState target")

# ArtController target - try multiple attribute names
if not try_set(f"{GRAPH_PATH}/ArtController.inputs:targetPrim", [ROBOT_PRIM], "ArtController targetPrim"):
    try_set(f"{GRAPH_PATH}/ArtController.inputs:robotPath", ROBOT_PRIM, "ArtController robotPath")

# Topic names
try_set(f"{GRAPH_PATH}/PubJointState.inputs:topicName", "isaac_joint_states", "Pub topic")
try_set(f"{GRAPH_PATH}/SubJointState.inputs:topicName", "isaac_joint_commands", "Sub topic")

# ======================================================================
# STEP 5: List all OG node attributes for debug
# ======================================================================
print(f"\nSTEP 5: Diagnostic - listing ArtController attributes ...")
try:
    ctrl_prim = stage.GetPrimAtPath(f"{GRAPH_PATH}/ArtController")
    if ctrl_prim and ctrl_prim.IsValid():
        for attr in ctrl_prim.GetAttributes():
            name = attr.GetName()
            if "input" in name.lower() or "output" in name.lower():
                val = attr.Get()
                print(f"  {name} = {val}")
except Exception as e:
    print(f"  [WARN] Could not list attributes: {e}")

print(f"\nSTEP 5b: Diagnostic - listing SubJointState output attributes ...")
try:
    sub_prim = stage.GetPrimAtPath(f"{GRAPH_PATH}/SubJointState")
    if sub_prim and sub_prim.IsValid():
        for attr in sub_prim.GetAttributes():
            name = attr.GetName()
            if "output" in name.lower() or "topic" in name.lower():
                val = attr.Get()
                print(f"  {name} = {val}")
except Exception as e:
    print(f"  [WARN] Could not list attributes: {e}")

# ======================================================================
# STEP 6: Save
# ======================================================================
print("\nSTEP 6: Saving ...")
try:
    rp = stage.GetRootLayer().realPath
    if rp:
        stage.GetRootLayer().Save()
        print(f"  [OK] Saved to {rp}")
except Exception:
    print("  [INFO] Please File -> Save manually")

print("\n" + "=" * 60)
print("DONE!")
print(f"  ArticulationRoot: {ROBOT_PRIM}")
print(f"  Publishes:  /isaac_joint_states")
print(f"  Subscribes: /isaac_joint_commands")
print("=" * 60)
print("\nNext: In RVIZ, drag the interactive markers and click 'Plan & Execute'.")
print("The UR5 in Isaac Sim should follow the motion.")
