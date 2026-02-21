"""
Run in Isaac Sim Script Editor: add detection camera back to wrist_3_link.
Open SPARC_ur5_fixed.usd first, then run this, then save.
"""

import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf

ctx = omni.usd.get_context()
stage = ctx.get_stage()
assert stage, "No stage open!"

wrist = stage.GetPrimAtPath("/World/ur5/wrist_3_link")
if not wrist or not wrist.IsValid():
    print("[ERROR] /World/ur5/wrist_3_link not found!")
else:
    cam_parent_path = "/World/ur5/wrist_3_link/detection_camera_01"
    cam_path = cam_parent_path + "/Camera_SG2_OX03CC_5200_GMSL2_H60YA"

    # Create parent Xform if not exists
    parent = stage.GetPrimAtPath(cam_parent_path)
    if not parent or not parent.IsValid():
        parent = UsdGeom.Xform.Define(stage, cam_parent_path)
        print(f"[OK] Created {cam_parent_path}")

    # Create camera
    cam_prim = stage.GetPrimAtPath(cam_path)
    if cam_prim and cam_prim.IsValid():
        print(f"[INFO] Camera already exists at {cam_path}")
    else:
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.GetFocalLengthAttr().Set(24.0)
        cam.GetHorizontalApertureAttr().Set(36.0)
        cam.GetVerticalApertureAttr().Set(24.0)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

        # Point camera downward (typical wrist-mount detection camera)
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))

        print(f"[OK] Created camera at {cam_path}")
        print("  You may need to adjust the camera transform in the Property panel")
        print("  to match the original orientation for YOLO detection.")

    # Save
    try:
        out = stage.GetRootLayer().realPath
        if out:
            stage.GetRootLayer().Save()
            print(f"[OK] Saved to {out}")
        else:
            print("[INFO] Use File -> Save to persist changes.")
    except Exception as e:
        print(f"[WARN] Auto-save failed: {e}")
        print("  Please File -> Save manually.")

print("\n=== DONE ===")
