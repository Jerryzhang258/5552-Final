# Run in Isaac Sim Script Editor (SPARC_ur5_fixed.usd open, simulation PLAYING).
# Fixes /Graph/ROS_JointStates: topic names, robotPath/targetPrim, removes broken /World/ROS2_ActionGraph.

import omni.usd
import omni.graph.core as og
from pxr import Usd

ctx = omni.usd.get_context()
stage = ctx.get_stage()
assert stage, "No stage open!"

# ======================================================================
# STEP 1: Inspect existing /Graph/ROS_JointStates
# ======================================================================
EXISTING_GRAPH = "/Graph/ROS_JointStates"
print("=" * 60)
print(f"STEP 1: Inspecting existing graph {EXISTING_GRAPH} ...")

graph_prim = stage.GetPrimAtPath(EXISTING_GRAPH)
if not graph_prim or not graph_prim.IsValid():
    print(f"  [ERROR] {EXISTING_GRAPH} not found!")
    # List all graphs
    print("  Searching for all OmniGraph prims ...")
    for p in Usd.PrimRange(stage.GetPrimAtPath("/")):
        tname = str(p.GetTypeName())
        if "Graph" in tname or "graph" in str(p.GetPath()).lower():
            print(f"    {p.GetPath()} ({tname})")
else:
    print(f"  [OK] Found {EXISTING_GRAPH}")
    # List all children (nodes)
    for child in Usd.PrimRange(graph_prim):
        path = str(child.GetPath())
        tname = str(child.GetTypeName())
        if child == graph_prim:
            continue
        depth = path.count("/") - EXISTING_GRAPH.count("/")
        if depth > 1:
            continue
        print(f"  Node: {child.GetName()} ({tname})")
        # Show key attributes
        for attr in child.GetAttributes():
            name = attr.GetName()
            if any(k in name.lower() for k in ["topic", "target", "robot", "joint"]):
                val = attr.Get()
                print(f"    {name} = {val}")

# ======================================================================
# STEP 2: Find and update topic names in existing graph
# ======================================================================
print(f"\nSTEP 2: Updating topic names in {EXISTING_GRAPH} ...")

# Find publisher node (publishes joint states FROM Isaac Sim)
pub_candidates = [
    f"{EXISTING_GRAPH}/PublishJointState",
    f"{EXISTING_GRAPH}/ROS2PublishJointState",
    f"{EXISTING_GRAPH}/publish_joint_state",
    f"{EXISTING_GRAPH}/PubJointState",
]
sub_candidates = [
    f"{EXISTING_GRAPH}/SubscribeJointState",
    f"{EXISTING_GRAPH}/ROS2SubscribeJointState",
    f"{EXISTING_GRAPH}/subscribe_joint_state",
    f"{EXISTING_GRAPH}/SubJointState",
]

# Also search by scanning children
for child in graph_prim.GetChildren() if graph_prim and graph_prim.IsValid() else []:
    name_lower = child.GetName().lower()
    path = str(child.GetPath())
    if "publish" in name_lower and "joint" in name_lower:
        if path not in pub_candidates:
            pub_candidates.insert(0, path)
    if "subscribe" in name_lower and "joint" in name_lower:
        if path not in sub_candidates:
            sub_candidates.insert(0, path)


def find_and_set_topic(candidates, new_topic, role):
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            print(f"  Found {role}: {path}")
            # Try to set topic via OG controller
            try:
                og.Controller.set(
                    og.Controller.attribute(f"{path}.inputs:topicName"),
                    new_topic,
                )
                print(f"    [OK] topicName = {new_topic}")
                return True
            except Exception as e:
                print(f"    [WARN] og.Controller.set failed: {e}")
                # Try direct USD attribute
                try:
                    topic_attr = prim.GetAttribute("inputs:topicName")
                    if topic_attr:
                        old_val = topic_attr.Get()
                        topic_attr.Set(new_topic)
                        print(f"    [OK] topicName: {old_val} -> {new_topic}")
                        return True
                except Exception as e2:
                    print(f"    [WARN] USD set failed: {e2}")
    print(f"  [WARN] No {role} node found in candidates: {candidates}")
    return False


find_and_set_topic(pub_candidates, "isaac_joint_states", "Publisher")
find_and_set_topic(sub_candidates, "isaac_joint_commands", "Subscriber")

# ======================================================================
# STEP 2b: Set PublisherJointState targetPrim (so it reads from correct robot)
# ======================================================================
print(f"\nSTEP 2b: Setting Publisher targetPrim ...")
for path in pub_candidates:
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        try:
            og.Controller.set(
                og.Controller.attribute(f"{path}.inputs:targetPrim"),
                ["/World/ur5/root_joint"],
            )
            print(f"  [OK] PublisherJointState.targetPrim = /World/ur5/root_joint")
        except Exception as e:
            print(f"  [WARN] {e}")
        break

# ======================================================================
# STEP 3: Check ArticulationController target in existing graph
# ======================================================================
print(f"\nSTEP 3: Checking ArticulationController in {EXISTING_GRAPH} ...")
ctrl_candidates = [
    f"{EXISTING_GRAPH}/ArticulationController",
    f"{EXISTING_GRAPH}/ArtController",
    f"{EXISTING_GRAPH}/IsaacArticulationController",
]
for child in graph_prim.GetChildren() if graph_prim and graph_prim.IsValid() else []:
    name_lower = child.GetName().lower()
    if "articulation" in name_lower or "controller" in name_lower:
        path = str(child.GetPath())
        if path not in ctrl_candidates:
            ctrl_candidates.insert(0, path)

ctrl_prim_path = None
for path in ctrl_candidates:
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        print(f"  Found controller: {path}")
        ctrl_prim_path = path
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if any(k in name.lower() for k in ["target", "robot", "joint", "input"]):
                val = attr.Get()
                print(f"    {name} = {val}")
        break

# ======================================================================
# STEP 3b: SET robotPath on ArticulationController (critical for motion)
# ======================================================================
if ctrl_prim_path:
    print(f"\nSTEP 3b: Setting ArticulationController robotPath ...")
    # Articulation root in this scene is at /World/ur5/root_joint
    robot_paths_to_try = ["/World/ur5/root_joint", "/World/ur5"]
    for rpath in robot_paths_to_try:
        try:
            og.Controller.set(
                og.Controller.attribute(f"{ctrl_prim_path}.inputs:robotPath"),
                rpath,
            )
            print(f"  [OK] robotPath = {rpath}")
            break
        except Exception as e:
            print(f"  [WARN] robotPath {rpath}: {e}")
    # Also try targetPrim (relationship)
    try:
        og.Controller.set(
            og.Controller.attribute(f"{ctrl_prim_path}.inputs:targetPrim"),
            [robot_paths_to_try[0]],
        )
        print(f"  [OK] targetPrim = {robot_paths_to_try[0]}")
    except Exception as e:
        print(f"  [INFO] targetPrim: {e}")

# ======================================================================
# STEP 4: Remove our broken /World/ROS2_ActionGraph
# ======================================================================
print("\nSTEP 4: Removing broken /World/ROS2_ActionGraph ...")
our_graph = stage.GetPrimAtPath("/World/ROS2_ActionGraph")
if our_graph and our_graph.IsValid():
    stage.RemovePrim("/World/ROS2_ActionGraph")
    print("  [OK] Removed /World/ROS2_ActionGraph")
else:
    print("  [OK] /World/ROS2_ActionGraph already gone")

# ======================================================================
# STEP 5: Save
# ======================================================================
print("\nSTEP 5: Saving ...")
try:
    rp = stage.GetRootLayer().realPath
    if rp:
        stage.GetRootLayer().Save()
        print(f"  [OK] Saved to {rp}")
except Exception:
    print("  [INFO] Please File -> Save manually")

print("\n" + "=" * 60)
print("DONE! The existing /Graph/ROS_JointStates is now configured:")
print("  Publishes:  /isaac_joint_states  (Isaac Sim -> MoveIt)")
print("  Subscribes: /isaac_joint_commands (MoveIt -> Isaac Sim)")
print("=" * 60)
print("\nStop+Play Isaac Sim, then Plan & Execute in RVIZ.")
