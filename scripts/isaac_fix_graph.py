# Run in Isaac Sim Script Editor (while PLAYING):
# 1. Delete the broken /World/ROS2_ActionGraph
# 2. Fix the original /Graph/ROS_JointStates to use correct topics for MoveIt
# 3. Set targetPrim and robotPath to /World/ur5/root_joint (the actual articulation root)

import omni.usd
import omni.graph.core as og
from pxr import Usd, Sdf

stage = omni.usd.get_context().get_stage()
assert stage, "No stage!"

# --- STEP 1: Delete broken graph ---
bad = stage.GetPrimAtPath("/World/ROS2_ActionGraph")
if bad and bad.IsValid():
    stage.RemovePrim("/World/ROS2_ActionGraph")
    print("[OK] Deleted /World/ROS2_ActionGraph")
else:
    print("[INFO] /World/ROS2_ActionGraph not found (already deleted)")

# --- STEP 2: Inspect original graph ---
graph_path = "/Graph/ROS_JointStates"
graph_prim = stage.GetPrimAtPath(graph_path)
if not graph_prim or not graph_prim.IsValid():
    print(f"[ERROR] {graph_path} not found!")
else:
    print(f"\n[OK] Found {graph_path}")

    # List all nodes
    try:
        graph = og.get_graph_by_path(graph_path)
        nodes = graph.get_nodes()
        print(f"  Nodes ({len(nodes)}):")
        for n in nodes:
            print(f"    {n.get_prim_path()} [{n.get_type_name()}]")
    except Exception as e:
        print(f"  [WARN] Could not enumerate nodes: {e}")

    # Find and fix ArticulationController
    for p in Usd.PrimRange(graph_prim):
        name = p.GetName()
        path = str(p.GetPath())

        # Fix ArticulationController robotPath
        if "ArticulationController" in name or "articulationcontroller" in name.lower():
            print(f"\n  Fixing {path} ...")
            # robotPath
            attr = p.GetAttribute("inputs:robotPath")
            if attr and attr.IsValid():
                old = attr.Get()
                attr.Set("/World/ur5/root_joint")
                print(f"    robotPath: {old} -> /World/ur5/root_joint")

            # targetPrim via OmniGraph Controller
            try:
                node = og.get_node_by_path(path)
                og.Controller.set(
                    og.Controller.attribute("inputs:targetPrim", node),
                    ["/World/ur5/root_joint"]
                )
                print(f"    targetPrim -> [/World/ur5/root_joint]")
            except Exception as e:
                print(f"    [WARN] targetPrim: {e}")

        # Fix PublisherJointState targetPrim
        if "PublisherJointState" in name or "publisherjointstate" in name.lower():
            print(f"\n  Fixing {path} ...")
            try:
                node = og.get_node_by_path(path)
                og.Controller.set(
                    og.Controller.attribute("inputs:targetPrim", node),
                    ["/World/ur5/root_joint"]
                )
                print(f"    targetPrim -> [/World/ur5/root_joint]")
            except Exception as e:
                print(f"    [WARN] targetPrim: {e}")

            # Topic name
            try:
                node = og.get_node_by_path(path)
                og.Controller.set(
                    og.Controller.attribute("inputs:topicName", node),
                    "isaac_joint_states"
                )
                print(f"    topicName -> isaac_joint_states")
            except Exception:
                pass

        # Fix SubscriberJointState topic
        if "SubscriberJointState" in name or "subscriberjointstate" in name.lower():
            print(f"\n  Fixing {path} ...")
            try:
                node = og.get_node_by_path(path)
                og.Controller.set(
                    og.Controller.attribute("inputs:topicName", node),
                    "isaac_joint_commands"
                )
                print(f"    topicName -> isaac_joint_commands")
            except Exception:
                pass

# --- STEP 3: Save ---
try:
    layer = stage.GetRootLayer()
    rp = layer.GetRealPath() if hasattr(layer, "GetRealPath") else ""
    if rp:
        layer.Save()
        print(f"\n[OK] Saved to {rp}")
    else:
        print("\n[INFO] File -> Save to persist.")
except Exception:
    print("\n[INFO] File -> Save to persist.")

print("\n=== Done! Stop -> Play to apply. ===")
