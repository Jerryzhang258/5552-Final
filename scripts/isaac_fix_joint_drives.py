"""
Run in Isaac Sim Script Editor (SPARC_ur5_fixed.usd open, simulation STOPPED or PLAYING).
Diagnoses and fixes joint drive configuration on the UR5.
If joints have no position drives or zero stiffness, the ArticulationController
can't move the robot even if it receives correct commands.
"""

import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema, Gf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
assert stage, "No stage open!"

UR5_ROOT = "/World/ur5"

# Expected UR5 joint names
EXPECTED_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# ======================================================================
# STEP 1: Find all joints under /World/ur5
# ======================================================================
print("=" * 60)
print("STEP 1: Scanning all joints under /World/ur5 ...")

ur5_prim = stage.GetPrimAtPath(UR5_ROOT)
if not ur5_prim or not ur5_prim.IsValid():
    print(f"[ERROR] {UR5_ROOT} not found!")
    raise SystemExit

joints_found = []
for prim in Usd.PrimRange(ur5_prim):
    type_name = str(prim.GetTypeName())
    if "Joint" in type_name and "Fixed" not in type_name:
        path = str(prim.GetPath())
        name = prim.GetName()
        joints_found.append((name, path, prim, type_name))
        print(f"  {name} ({type_name}) at {path}")

if not joints_found:
    print("[ERROR] No joints found under /World/ur5!")
    raise SystemExit

# ======================================================================
# STEP 2: Check drive configuration on each joint
# ======================================================================
print(f"\nSTEP 2: Checking joint drive configuration ...")

joints_needing_fix = []
for name, path, prim, type_name in joints_found:
    if name not in EXPECTED_JOINTS:
        print(f"  {name}: skipping (not an arm joint)")
        continue

    # Check for DriveAPI
    has_drive = False
    drive_type = None
    stiffness = None
    damping = None

    # Check angular drive (for revolute joints)
    drive_api = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive_api:
        has_drive = True
        drive_type = "angular"
        stiffness_attr = drive_api.GetStiffnessAttr()
        damping_attr = drive_api.GetDampingAttr()
        if stiffness_attr:
            stiffness = stiffness_attr.Get()
        if damping_attr:
            damping = damping_attr.Get()

    if not has_drive:
        # Try linear drive
        drive_api = UsdPhysics.DriveAPI.Get(prim, "linear")
        if drive_api:
            has_drive = True
            drive_type = "linear"
            stiffness_attr = drive_api.GetStiffnessAttr()
            damping_attr = drive_api.GetDampingAttr()
            if stiffness_attr:
                stiffness = stiffness_attr.Get()
            if damping_attr:
                damping = damping_attr.Get()

    # Also check PhysxJointAPI for drive settings
    physx_drive_stiffness = None
    physx_drive_damping = None
    for attr in prim.GetAttributes():
        attr_name = attr.GetName()
        if "drive" in attr_name.lower() and "stiffness" in attr_name.lower():
            physx_drive_stiffness = attr.Get()
        if "drive" in attr_name.lower() and "damping" in attr_name.lower():
            physx_drive_damping = attr.Get()

    status = "OK" if has_drive and stiffness and stiffness > 0 else "NEEDS FIX"
    print(f"  {name}: drive={drive_type}, stiffness={stiffness}, damping={damping}, physx_stiffness={physx_drive_stiffness}, physx_damping={physx_drive_damping} [{status}]")

    if status == "NEEDS FIX":
        joints_needing_fix.append((name, path, prim, type_name))

# ======================================================================
# STEP 3: List ALL attributes of one joint for full debug
# ======================================================================
print(f"\nSTEP 3: Full attribute dump of first arm joint ...")
for name, path, prim, type_name in joints_found:
    if name == "shoulder_pan_joint":
        print(f"  Joint: {name} at {path}")
        for attr in prim.GetAttributes():
            val = attr.Get()
            print(f"    {attr.GetName()} = {val}")
        print(f"  Applied schemas: {[str(s) for s in prim.GetAppliedSchemas()]}")
        break

# ======================================================================
# STEP 4: Fix joints that need drive configuration
# ======================================================================
if joints_needing_fix:
    print(f"\nSTEP 4: Fixing {len(joints_needing_fix)} joints ...")
    for name, path, prim, type_name in joints_needing_fix:
        print(f"  Fixing {name} ...")
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive_api.CreateTypeAttr("force")
        drive_api.CreateStiffnessAttr(10000.0)
        drive_api.CreateDampingAttr(1000.0)
        drive_api.CreateMaxForceAttr(1000.0)
        print(f"    [OK] Applied angular drive: stiffness=10000, damping=1000")
else:
    print(f"\nSTEP 4: All arm joints already have drives configured.")

# ======================================================================
# STEP 5: Check ArticulationRootAPI
# ======================================================================
print(f"\nSTEP 5: Checking ArticulationRootAPI ...")
art_found = False
for prim in Usd.PrimRange(ur5_prim):
    schemas = [str(s) for s in prim.GetAppliedSchemas()]
    if any("ArticulationRootAPI" in s for s in schemas):
        print(f"  ArticulationRootAPI at: {prim.GetPath()}")
        art_found = True
        # Check if enabled
        for attr in prim.GetAttributes():
            attr_name = attr.GetName()
            if "articulationEnabled" in attr_name or "enabled" in attr_name.lower():
                print(f"    {attr_name} = {attr.Get()}")
        break
if not art_found:
    print("  [WARN] No ArticulationRootAPI found! Applying to /World/ur5 ...")
    UsdPhysics.ArticulationRootAPI.Apply(ur5_prim)

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
print("DONE! Stop + Play Isaac Sim, then Plan & Execute in RVIZ.")
print("=" * 60)
