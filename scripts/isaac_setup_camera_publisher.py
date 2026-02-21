# Run in Isaac Sim Script Editor (WHILE PLAYING):
# Adds a ROS2 camera publisher that sends images from detection_camera to /rgb topic.
# YOLO subscribes to /rgb for object detection.

import omni.usd
import omni.graph.core as og
from pxr import Usd

stage = omni.usd.get_context().get_stage()
assert stage, "No stage open!"

GRAPH_PATH = "/World/ROS2_CameraGraph"
CAMERA_PATH = "/World/packing_table/detection_camera/Camera_SG2_OX03CC_5200_GMSL2_H60YA"

# Check camera exists
cam = stage.GetPrimAtPath(CAMERA_PATH)
if not cam or not cam.IsValid():
    print(f"[WARN] Camera not found at {CAMERA_PATH}")
    print("  Looking for alternative cameras ...")
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World")):
        if p.GetTypeName() == "Camera":
            print(f"  Found: {p.GetPath()}")
    print("  Update CAMERA_PATH in this script if needed.")

# Remove old graph if exists
old = stage.GetPrimAtPath(GRAPH_PATH)
if old and old.IsValid():
    stage.RemovePrim(GRAPH_PATH)
    print(f"[OK] Removed old {GRAPH_PATH}")

# Try different node type versions
NODE_VERSIONS = [
    {
        "label": "isaacsim (v4.5+)",
        "tick": "omni.graph.action.OnPlaybackTick",
        "cam_helper": "isaacsim.ros2.bridge.ROS2CameraHelper",
    },
    {
        "label": "omni.isaac (v4.0)",
        "tick": "omni.graph.action.OnPlaybackTick",
        "cam_helper": "omni.isaac.ros2_bridge.ROS2CameraHelper",
    },
]

created = False
for ver in NODE_VERSIONS:
    if created:
        break
    print(f"\n[...] Trying {ver['label']} ...")
    try:
        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", ver["tick"]),
                    ("CameraHelper", ver["cam_helper"]),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "CameraHelper.inputs:execIn"),
                ],
            },
        )
        print(f"  [OK] Graph created with {ver['label']}")

        # Set camera helper attributes
        cam_node = None
        for n in nodes:
            if "CameraHelper" in n.get_prim_path():
                cam_node = n
                break

        if cam_node:
            # Set camera prim path
            try:
                og.Controller.set(
                    og.Controller.attribute("inputs:cameraPrim", cam_node),
                    [CAMERA_PATH]
                )
                print(f"  cameraPrim = {CAMERA_PATH}")
            except Exception as e1:
                print(f"  [WARN] cameraPrim via Controller: {e1}")
                try:
                    prim = stage.GetPrimAtPath(cam_node.get_prim_path())
                    attr = prim.GetAttribute("inputs:cameraPrim")
                    if attr:
                        attr.Set([CAMERA_PATH])
                except Exception:
                    pass

            # Set topic name
            try:
                og.Controller.set(
                    og.Controller.attribute("inputs:topicName", cam_node),
                    "rgb"
                )
                print(f"  topicName = rgb")
            except Exception:
                pass

            # Set type to "rgb" (color image)
            try:
                og.Controller.set(
                    og.Controller.attribute("inputs:type", cam_node),
                    "rgb"
                )
                print(f"  type = rgb")
            except Exception:
                pass

            # Frame ID
            try:
                og.Controller.set(
                    og.Controller.attribute("inputs:frameId", cam_node),
                    "detection_camera"
                )
            except Exception:
                pass

        created = True

    except Exception as e:
        print(f"  [FAIL] {e}")
        old = stage.GetPrimAtPath(GRAPH_PATH)
        if old and old.IsValid():
            stage.RemovePrim(GRAPH_PATH)

if created:
    # Save
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

    print("\n=== Camera publisher ready! ===")
    print("Verify: in a terminal run:")
    print("  ros2 topic hz /rgb")
    print("Should show ~30 Hz if camera is working.")
else:
    print("\n[ERROR] Could not create camera graph with any known node types.")
    print("You may need to manually add ROS2CameraHelper via the OmniGraph editor.")
