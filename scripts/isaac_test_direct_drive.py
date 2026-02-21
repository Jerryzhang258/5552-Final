"""
Run in Isaac Sim Script Editor (SPARC_ur5_fixed.usd open, simulation PLAYING).
Test if the UR5 can move at all by directly setting joint position targets
via USD drive attributes (no ROS2, no Action Graph).
If the arm moves: problem is in the graph/ROS2. If not: problem is physics/drives.
"""

import omni.usd
from pxr import Usd, UsdPhysics, Gf
import math

stage = omni.usd.get_context().get_stage()
if not stage:
    print("[ERROR] No stage. Open SPARC_ur5_fixed.usd first.")
    raise SystemExit

# UR5 arm joint paths (from isaac_fix_joint_drives.py output)
JOINTS = [
    "/World/ur5/joints/shoulder_pan_joint",
    "/World/ur5/joints/shoulder_lift_joint",
    "/World/ur5/joints/elbow_joint",
    "/World/ur5/joints/wrist_1_joint",
    "/World/ur5/joints/wrist_2_joint",
    "/World/ur5/joints/wrist_3_joint",
]

print("=" * 60)
print("Direct drive test: setting joint targets (radians)")
print("  If the arm moves in the viewport, physics/drives are OK.")
print("  If it does not move, the issue is in the articulation/drives.")
print("=" * 60)

# Small motion: add 0.2 rad to shoulder_pan so the base rotates slightly
targets = [0.2, -2.05, 2.22, 1.45, 1.48, 0.0]  # roughly arm-like, shoulder_pan=0.2

for i, path in enumerate(JOINTS):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        print(f"  [WARN] Missing {path}")
        continue
    drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive_api:
        print(f"  [WARN] No angular drive at {path}")
        continue
    target_attr = drive_api.GetTargetPositionAttr()
    if not target_attr:
        target_attr = drive_api.CreateTargetPositionAttr()
    target_attr.Set(targets[i])
    print(f"  {prim.GetName()} -> targetPosition = {targets[i]:.3f} rad")

print("\nCheck the viewport: the UR5 base (shoulder_pan) should rotate slightly.")
print("Wait 2-3 seconds. If nothing moves, articulation/drives are the issue.")
print("=" * 60)
