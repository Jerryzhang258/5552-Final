# Run in Isaac Sim Script Editor (NO need to Play):
# Directly sets UR5 link transforms to upright pose via forward kinematics.
# Works without simulation - the visual pose is written into the USD layer.
# Usage: Open SPARC_ur5_fixed.usd -> Script Editor -> run -> File -> Save.

import math
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf


# --- FK helpers (column-vector convention) ---

def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def trans(x, y, z):
    m = np.eye(4); m[0,3]=x; m[1,3]=y; m[2,3]=z; return m

def rpy(r, p, y):
    return rotz(y) @ roty(p) @ rotx(r)

def mat_to_quat(R):
    """3x3 rotation matrix -> quaternion (w, x, y, z)."""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return (0.25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return ((R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s)
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return ((R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s)
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return ((R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s)


# --- Compute FK for upright pose ---
PI = math.pi

# Desired joint angles: arm straight up
Q = [0.0, -PI/2, 0.0, -PI/2, 0.0, 0.0]

# UR5 URDF joint chain: (xyz, rpy, axis, joint_angle)
T_base = np.eye(4)

T_shoulder = T_base @ trans(0, 0, 0.089159) @ rpy(0, 0, 0) @ rotz(Q[0])
T_upper    = T_shoulder @ trans(0, 0.13585, 0) @ rpy(0, PI/2, 0) @ roty(Q[1])
T_forearm  = T_upper @ trans(0, -0.1197, 0.425) @ rpy(0, 0, 0) @ roty(Q[2])
T_wrist1   = T_forearm @ trans(0, 0, 0.39225) @ rpy(0, PI/2, 0) @ roty(Q[3])
T_wrist2   = T_wrist1 @ trans(0, 0.093, 0) @ rpy(0, 0, 0) @ rotz(Q[4])
T_wrist3   = T_wrist2 @ trans(0, 0, 0.09465) @ rpy(0, 0, 0) @ roty(Q[5])
T_ee       = T_wrist3 @ trans(0, 0.0823, 0) @ rpy(0, 0, PI/2)

LINK_TRANSFORMS = [
    ("/World/ur5/base_link",      T_base),
    ("/World/ur5/shoulder_link",   T_shoulder),
    ("/World/ur5/upper_arm_link",  T_upper),
    ("/World/ur5/forearm_link",    T_forearm),
    ("/World/ur5/wrist_1_link",    T_wrist1),
    ("/World/ur5/wrist_2_link",    T_wrist2),
    ("/World/ur5/wrist_3_link",    T_wrist3),
    ("/World/ur5/ee_link",         T_ee),
]


def set_link_xform(stage, path, T):
    """Set a link's translate + orient from a 4x4 FK matrix."""
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        print(f"  [SKIP] {path} not found")
        return

    pos = T[:3, 3]
    w, x, y, z = mat_to_quat(T[:3, :3])

    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()

    t_done = False
    r_done = False
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and not t_done:
            op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
            t_done = True
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient and not r_done:
            try:
                op.Set(Gf.Quatd(float(w), float(x), float(y), float(z)))
            except Exception:
                op.Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
            r_done = True

    if not t_done:
        xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    if not r_done:
        xf.AddOrientOp().Set(Gf.Quatd(float(w), float(x), float(y), float(z)))

    print(f"  [OK] {path}: pos=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")


def set_joint_initial_state(stage):
    """Also set physics drive targets so Play starts from upright."""
    ANGLES_DEG = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -90.0,
        "elbow_joint": 0.0,
        "wrist_1_joint": -90.0,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }
    joints = stage.GetPrimAtPath("/World/ur5/joints")
    if not joints:
        return
    for p in Usd.PrimRange(joints):
        name = p.GetName()
        if name not in ANGLES_DEG:
            continue
        try:
            drive = UsdPhysics.DriveAPI.Get(p, "angular")
            if drive:
                attr = drive.GetTargetPositionAttr()
                if attr:
                    attr.Set(ANGLES_DEG[name])
        except Exception:
            pass


# --- Main ---
stage = omni.usd.get_context().get_stage()
if not stage:
    print("No stage open.")
else:
    print("=" * 60)
    print("Setting UR5 to UPRIGHT pose (direct FK, no Play needed)")
    print("=" * 60)

    for path, T in LINK_TRANSFORMS:
        set_link_xform(stage, path, T)

    print("")
    print("Setting physics drive targets ...")
    set_joint_initial_state(stage)

    print("")
    print("=" * 60)
    print("Done! The arm should now appear upright in the viewport.")
    print("File -> Save to persist. Play will also start from this pose.")
    print("=" * 60)
