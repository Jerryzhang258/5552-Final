"""
ONE-CLICK FIX: paste into Isaac Sim Script Editor and Run.
  1) Removes missing panda + revel prims
  2) Records old /World/ur5 transform, deletes it
  3) Re-imports ur5_isaac.urdf with proper physics to /World/ur5
  4) Restores transform
  5) Saves as SPARC_ur5_fixed.usd
"""

import os
import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, Gf

PROJECT = os.path.expanduser("~/nlp-pnp-robotic-arm")
URDF = os.path.join(PROJECT, "src/robot_description/urdf/ur5_isaac.urdf")
OUT = os.path.join(PROJECT, "SPARC_ur5_fixed.usd")

ctx = omni.usd.get_context()
stage = ctx.get_stage()
assert stage, "No stage open! Open SPARC.usd first."

# ---- 1) Remove missing prims (panda + revel) ----
remove_paths = [
    "/World/panda",
    "/World/ur5/stanley_knife",
    "/World/ur5/tape",
    "/World/ur5/allen_key",
    "/World/ur5/wood_chisel",
    "/World/packing_table/plastic_pallet",
    "/World/packing_table/plastic_pallet_01",
]
for p in remove_paths:
    prim = stage.GetPrimAtPath(p)
    if prim and prim.IsValid():
        stage.RemovePrim(p)
        print(f"[OK] Removed {p}")

# ---- 2) Record old /World/ur5 transform, then delete ----
old_ur5 = stage.GetPrimAtPath("/World/ur5")
saved_pos = None
saved_rot = None
if old_ur5 and old_ur5.IsValid():
    xf = UsdGeom.Xformable(old_ur5)
    try:
        mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        saved_pos = mat.ExtractTranslation()
        saved_rot = mat.ExtractRotationQuat()
        print(f"[OK] Saved ur5 transform: pos={saved_pos}")
    except Exception:
        print("[WARN] Could not read ur5 transform, will use default")
    stage.RemovePrim("/World/ur5")
    print("[OK] Deleted old /World/ur5")
else:
    print("[INFO] /World/ur5 not found, skipping delete")

# Also remove ur5_new if leftover from previous attempt
if stage.GetPrimAtPath("/World/ur5_new"):
    stage.RemovePrim("/World/ur5_new")
    print("[OK] Deleted leftover /World/ur5_new")

# ---- 3) Import ur5_isaac.urdf ----
assert os.path.isfile(URDF), f"URDF not found: {URDF}"
print(f"[...] Importing {URDF}")

imported = False
for module_path in [
    "isaacsim.asset.importer.urdf",
    "omni.isaac.urdf",
]:
    try:
        mod = __import__(module_path, fromlist=["_urdf"])
        _urdf = mod._urdf
        urdf_interface = _urdf.acquire_urdf_interface()

        cfg = _urdf.ImportConfig()
        cfg.merge_fixed_joints = False
        cfg.fix_base = True
        cfg.make_default_prim = False
        cfg.create_physics_scene = False

        # drive type: use enum if available, fallback to int
        try:
            cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        except AttributeError:
            try:
                cfg.set_default_drive_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION)
            except Exception:
                print("[WARN] Could not set drive type, using default")

        cfg.default_drive_strength = 1e4
        cfg.default_position_drive_damping = 1e3

        urdf_dir = os.path.dirname(URDF)
        urdf_file = os.path.basename(URDF)

        # Isaac Sim 4.5+: parse_urdf(asset_root, filename, config)
        try:
            result = urdf_interface.parse_urdf(urdf_dir, urdf_file, cfg)
        except TypeError:
            result = urdf_interface.parse_urdf(URDF, cfg)

        # Isaac Sim 4.5+: import_robot(asset_root, filename, robot, config, dest)
        try:
            prim_path = urdf_interface.import_robot(
                urdf_dir, urdf_file, result, cfg, "/World/ur5"
            )
        except TypeError:
            prim_path = urdf_interface.import_robot(URDF, result, cfg, "/World/ur5")
        print(f"[OK] Imported UR5 to: {prim_path}")
        imported = True
        break
    except Exception as e:
        print(f"[WARN] {module_path} failed: {e}")

if not imported:
    print("[ERROR] URDF import failed with all methods.")
    print("  Manual fix: Isaac Utils -> URDF Importer")
    print(f"  File: {URDF}")
    print("  Target: /World/ur5")
else:
    # ---- 4) Restore transform ----
    new_ur5 = stage.GetPrimAtPath("/World/ur5")
    if new_ur5 and new_ur5.IsValid() and saved_pos is not None:
        xf = UsdGeom.Xformable(new_ur5)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(saved_pos)
        if saved_rot is not None:
            try:
                xf.AddOrientOp().Set(Gf.Quatf(saved_rot))
            except Exception:
                pass
        print(f"[OK] Restored transform: {saved_pos}")

    # ---- 5) Verify base_link exists ----
    bl = stage.GetPrimAtPath("/World/ur5/base_link")
    if bl and bl.IsValid():
        schemas = [str(s) for s in bl.GetAppliedSchemas()]
        has_art = any("ArticulationRootAPI" in s for s in schemas)
        has_rb = any("RigidBodyAPI" in s for s in schemas)
        print(f"[OK] /World/ur5/base_link exists! ArticulationRoot={has_art}, RigidBody={has_rb}")
    else:
        print("[WARN] /World/ur5/base_link still not found after import!")
        print("  Listing /World/ur5 children:")
        ur5p = stage.GetPrimAtPath("/World/ur5")
        if ur5p and ur5p.IsValid():
            for c in ur5p.GetChildren():
                print(f"    {c.GetPath()}")

    # ---- 6) Save ----
    try:
        stage.Export(OUT)
        print(f"\n[DONE] Saved to: {OUT}")
        print("Next: open SPARC_ur5_fixed.usd, press Play, then run ./run_demo.sh")
    except Exception as e:
        print(f"[ERROR] Save failed: {e}")
        print("Please manually: File -> Save As -> SPARC_ur5_fixed.usd")

print("\n=== FINISHED ===")
