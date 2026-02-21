# Run in Isaac Sim Script Editor (simulation PLAYING). Diagnoses and fixes /Graph/ROS_JointStates.

import omni.usd
import omni.graph.core as og
from pxr import Usd

ctx = omni.usd.get_context()
stage = ctx.get_stage()
assert stage, "No stage open!"

GRAPH_PATH = "/Graph/ROS_JointStates"

# ======================================================================
# STEP 1: Get the OmniGraph and list ALL connections
# ======================================================================
print("=" * 60)
print("STEP 1: Listing all connections in the graph ...")

graph = og.get_graph_by_path(GRAPH_PATH)
if graph is None or not graph.is_valid():
    print(f"[ERROR] Graph {GRAPH_PATH} not found!")
    raise SystemExit

# List all nodes
nodes = graph.get_nodes()
print(f"\nNodes ({len(nodes)}):")
for node in nodes:
    print(f"  {node.get_prim_path()} (type: {node.get_node_type().get_node_type()})")

# List all connections
print(f"\nConnections:")
connection_count = 0
for node in nodes:
    for attr in node.get_attributes():
        if attr.get_port_type() == og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT:
            upstream = attr.get_upstream_connections()
            if upstream:
                for src_attr in upstream:
                    src_path = f"{src_attr.get_node().get_prim_path()}.{src_attr.get_name()}"
                    dst_path = f"{node.get_prim_path()}.{attr.get_name()}"
                    print(f"  {src_path} -> {dst_path}")
                    connection_count += 1
if connection_count == 0:
    print("  NONE found!")

# ======================================================================
# STEP 2: Check required connections
# ======================================================================
print(f"\nSTEP 2: Checking required Sub->Controller connections ...")

sub_node = None
ctrl_node = None
tick_node = None
for node in nodes:
    name = node.get_prim_path().split("/")[-1].lower()
    ntype = node.get_node_type().get_node_type().lower()
    if "subscrib" in name or "subscrib" in ntype:
        sub_node = node
        print(f"  Subscriber node: {node.get_prim_path()}")
    if "articulation" in name or "articulationcontroller" in ntype:
        ctrl_node = node
        print(f"  Controller node: {node.get_prim_path()}")
    if "tick" in name or "playbacktick" in ntype:
        tick_node = node
        print(f"  Tick node: {node.get_prim_path()}")

if not sub_node:
    print("  [ERROR] No subscriber node found!")
if not ctrl_node:
    print("  [ERROR] No articulation controller node found!")

# ======================================================================
# STEP 3: List subscriber outputs and controller inputs
# ======================================================================
if sub_node:
    print(f"\nSTEP 3a: Subscriber output attributes ...")
    for attr in sub_node.get_attributes():
        if attr.get_port_type() == og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT:
            conns = attr.get_downstream_connections()
            conn_strs = [f"{c.get_node().get_prim_path()}.{c.get_name()}" for c in conns]
            print(f"  {attr.get_name()} -> {conn_strs if conn_strs else 'NOT CONNECTED'}")

if ctrl_node:
    print(f"\nSTEP 3b: Controller input attributes ...")
    for attr in ctrl_node.get_attributes():
        if attr.get_port_type() == og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT:
            conns = attr.get_upstream_connections()
            conn_strs = [f"{c.get_node().get_prim_path()}.{c.get_name()}" for c in conns]
            val = None
            try:
                val = attr.get()
            except:
                pass
            print(f"  {attr.get_name()} <- {conn_strs if conn_strs else 'NOT CONNECTED'} (val={val})")

# ======================================================================
# STEP 4: Check and fix robotPath / targetPrim on controller
# ======================================================================
if ctrl_node:
    print(f"\nSTEP 4: Checking controller robot target ...")
    robot_path_attr = None
    target_prim_attr = None
    for attr in ctrl_node.get_attributes():
        if attr.get_name() == "inputs:robotPath":
            robot_path_attr = attr
        if attr.get_name() == "inputs:targetPrim":
            target_prim_attr = attr

    if robot_path_attr:
        val = robot_path_attr.get()
        print(f"  robotPath = '{val}'")
        if not val or val == "":
            # Find articulation root
            art_root = None
            ur5 = stage.GetPrimAtPath("/World/ur5")
            if ur5 and ur5.IsValid():
                for p in Usd.PrimRange(ur5):
                    schemas = [str(s) for s in p.GetAppliedSchemas()]
                    if any("ArticulationRootAPI" in s for s in schemas):
                        art_root = str(p.GetPath())
                        break
            if art_root:
                robot_path_attr.set(art_root)
                print(f"  [FIX] Set robotPath = '{art_root}'")
            else:
                robot_path_attr.set("/World/ur5")
                print(f"  [FIX] Set robotPath = '/World/ur5' (fallback)")

    # Fix targetPrim: must point to articulation root (root_joint), not base_link
    ART_ROOT = "/World/ur5/root_joint"
    if target_prim_attr:
        try:
            val = target_prim_attr.get()
            print(f"  targetPrim = {val}")
            # If wrong (e.g. base_link), set to articulation root
            if val and len(val) > 0 and "root_joint" not in str(val[0]):
                try:
                    og.Controller.set(
                        og.Controller.attribute(f"{ctrl_node.get_prim_path()}.inputs:targetPrim"),
                        [ART_ROOT],
                    )
                    print(f"  [FIX] Set targetPrim = [{ART_ROOT}]")
                except Exception as e:
                    print(f"  [WARN] Could not set targetPrim: {e}")
        except Exception as e:
            print(f"  targetPrim = (could not read: {e})")
            try:
                og.Controller.set(
                    og.Controller.attribute(f"{ctrl_node.get_prim_path()}.inputs:targetPrim"),
                    [ART_ROOT],
                )
                print(f"  [FIX] Set targetPrim = [{ART_ROOT}]")
            except Exception as e2:
                print(f"  [WARN] Could not set targetPrim: {e2}")

# ======================================================================
# STEP 4b: If 7 joints (arm+gripper) but UR5 only has 6, set jointIndices to [0..5]
# ======================================================================
if ctrl_node:
    try:
        jnames_attr = None
        jidx_attr = None
        for attr in ctrl_node.get_attributes():
            if attr.get_name() == "inputs:jointNames":
                jnames_attr = attr
            if attr.get_name() == "inputs:jointIndices":
                jidx_attr = attr
        if jnames_attr and jidx_attr:
            jnames = jnames_attr.get()
            if jnames and len(jnames) == 7 and any("robotiq" in str(j).lower() for j in jnames):
                og.Controller.set(
                    og.Controller.attribute(f"{ctrl_node.get_prim_path()}.inputs:jointIndices"),
                    [0, 1, 2, 3, 4, 5],
                )
                print(f"\nSTEP 4b: 7 joints (含 robotiq) 但 UR5 仅 6 轴 -> 已设 jointIndices = [0,1,2,3,4,5]")
    except Exception as e:
        print(f"\nSTEP 4b: jointIndices 设置跳过: {e}")

# ======================================================================
# STEP 5: Fix missing connections
# ======================================================================
if sub_node and ctrl_node:
    print(f"\nSTEP 5: Fixing missing connections ...")
    keys = og.Controller.Keys

    required_connections = [
        ("outputs:jointNames", "inputs:jointNames"),
        ("outputs:positionCommand", "inputs:positionCommand"),
        ("outputs:velocityCommand", "inputs:velocityCommand"),
        ("outputs:effortCommand", "inputs:effortCommand"),
    ]

    sub_path = sub_node.get_prim_path()
    ctrl_path = ctrl_node.get_prim_path()

    for src_name, dst_name in required_connections:
        src_attr = None
        dst_attr = None
        for a in sub_node.get_attributes():
            if a.get_name() == src_name:
                src_attr = a
        for a in ctrl_node.get_attributes():
            if a.get_name() == dst_name:
                dst_attr = a

        if not src_attr or not dst_attr:
            print(f"  [SKIP] {src_name} -> {dst_name} (attribute not found)")
            continue

        existing = src_attr.get_downstream_connections()
        already_connected = any(
            c.get_node().get_prim_path() == ctrl_path and c.get_name() == dst_name
            for c in existing
        )
        if already_connected:
            print(f"  [OK] {src_name} -> {dst_name} (already connected)")
        else:
            try:
                og.Controller.connect(
                    f"{sub_path}.{src_name}",
                    f"{ctrl_path}.{dst_name}",
                )
                print(f"  [FIX] Connected {src_name} -> {dst_name}")
            except Exception as e:
                print(f"  [WARN] Failed to connect {src_name} -> {dst_name}: {e}")

    # STEP 5a: 确保 Subscriber 订阅的 topic 为 isaac_joint_commands（与 ROS2 一致）
    try:
        topic_attr = sub_node.get_attribute("inputs:topicName")
        if topic_attr is not None:
            og.Controller.set(og.Controller.attribute(f"{sub_path}.inputs:topicName"), "isaac_joint_commands")
            print(f"  [FIX] Subscriber topicName = isaac_joint_commands")
    except Exception as e:
        print(f"  [WARN] Set topicName: {e}")

    # Also ensure tick -> controller execIn
    if tick_node:
        tick_path = tick_node.get_prim_path()
        tick_out = None
        ctrl_exec = None
        for a in tick_node.get_attributes():
            if a.get_name() == "outputs:tick":
                tick_out = a
        for a in ctrl_node.get_attributes():
            if a.get_name() == "inputs:execIn":
                ctrl_exec = a

        if tick_out and ctrl_exec:
            existing = tick_out.get_downstream_connections()
            already = any(
                c.get_node().get_prim_path() == ctrl_path and c.get_name() == "inputs:execIn"
                for c in existing
            )
            if already:
                print(f"  [OK] tick -> controller execIn (already connected)")
            else:
                try:
                    og.Controller.connect(
                        f"{tick_path}.outputs:tick",
                        f"{ctrl_path}.inputs:execIn",
                    )
                    print(f"  [FIX] Connected tick -> controller execIn")
                except Exception as e:
                    print(f"  [WARN] Failed: {e}")

# ======================================================================
# STEP 6: Save
# ======================================================================
print(f"\nSTEP 6: Saving ...")
try:
    rp = stage.GetRootLayer().realPath
    if rp:
        stage.GetRootLayer().Save()
        print(f"  [OK] Saved to {rp}")
except Exception:
    print("  [INFO] File -> Save manually")

print("\n" + "=" * 60)
print("DONE! Stop+Play Isaac Sim, then Plan & Execute in RVIZ.")
print("=" * 60)
