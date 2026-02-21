"""
Run in Isaac Sim Script Editor: re-import UR5 URDF with proper articulation physics,
then replace the existing /World/ur5 in the current stage.

Steps:
  1. Open SPARC.usd (or SPARC_ur5_only.usd) in Isaac Sim
  2. Window -> Script Editor -> paste this -> Run
  3. Save as SPARC_ur5_fixed.usd
"""

import os
import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, Gf

PROJECT_DIR = os.path.expanduser("~/nlp-pnp-robotic-arm")
URDF_PATH = os.path.join(
    PROJECT_DIR,
    "src/robot_description/urdf/ur5_joint_limited_robot.urdf",
)

# ---- Step 1: Print current /World/ur5 info ----
ctx = omni.usd.get_context()
stage = ctx.get_stage()

if not stage:
    print("[ERROR] No stage open. Open SPARC.usd first.")
else:
    print("=== Current /World/ur5 check ===")
    ur5_old = stage.GetPrimAtPath("/World/ur5")
    if ur5_old and ur5_old.IsValid():
        print("Found existing /World/ur5 - will list children:")
        for p in Usd.PrimRange(ur5_old):
            depth = str(p.GetPath()).count("/") - 2
            if depth <= 3:
                schemas = [str(s) for s in p.GetAppliedSchemas()]
                tags = []
                if any("ArticulationRootAPI" in s for s in schemas):
                    tags.append("ART")
                if any("RigidBodyAPI" in s for s in schemas):
                    tags.append("RB")
                t = str(p.GetTypeName())
                if "Joint" in t:
                    tags.append("J")
                tag = " " + str(tags) if tags else ""
                print("  " * depth + f"{p.GetName()} ({t}){tag}")
    else:
        print("/World/ur5 does NOT exist in current stage.")

    # ---- Step 2: Try URDF import ----
    print(f"\n=== URDF import ===")
    print(f"URDF path: {URDF_PATH}")
    if not os.path.isfile(URDF_PATH):
        print(f"[ERROR] URDF file not found at {URDF_PATH}")
    else:
        try:
            from isaacsim.asset.importer.urdf import _urdf
            urdf_interface = _urdf.acquire_urdf_interface()

            import_config = _urdf.ImportConfig()
            import_config.merge_fixed_joints = False
            import_config.fix_base = True
            import_config.make_default_prim = False
            import_config.create_physics_scene = False
            import_config.default_drive_type = 1  # 1 = position drive
            import_config.default_drive_strength = 1e4
            import_config.default_position_drive_damping = 1e3

            dest_path = "/World/ur5_new"
            result = urdf_interface.parse_urdf(URDF_PATH, import_config)
            prim_path = urdf_interface.import_robot(
                URDF_PATH, result, import_config, dest_path
            )
            print(f"Imported UR5 to: {prim_path}")

            # List what was created
            new_ur5 = stage.GetPrimAtPath(dest_path)
            if new_ur5 and new_ur5.IsValid():
                print("\nNew /World/ur5_new children:")
                for p in Usd.PrimRange(new_ur5):
                    depth = str(p.GetPath()).count("/") - 2
                    if depth <= 3:
                        schemas = [str(s) for s in p.GetAppliedSchemas()]
                        tags = []
                        if any("ArticulationRootAPI" in s for s in schemas):
                            tags.append("ART")
                        if any("RigidBodyAPI" in s for s in schemas):
                            tags.append("RB")
                        t = str(p.GetTypeName())
                        if "Joint" in t:
                            tags.append("J")
                        tag = " " + str(tags) if tags else ""
                        print("  " * depth + f"{p.GetName()} ({t}){tag}")

                print("\n=== Next steps ===")
                print("1) In the Stage panel, verify /World/ur5_new has all links with physics")
                print("2) Delete /World/ur5 (old broken one)")
                print("3) Rename /World/ur5_new -> /World/ur5")
                print("4) Re-position the robot if needed (copy transform from old ur5)")
                print("5) Re-wire the ROS2 Action Graph to point to /World/ur5/base_link")
                print("6) File -> Save As -> SPARC_ur5_fixed.usd")
            else:
                print("[ERROR] Import succeeded but prim not found at", dest_path)

        except ImportError:
            print("[WARN] isaacsim.asset.importer.urdf not available.")
            print("Trying alternative: omni.isaac.urdf extension...")
            try:
                from omni.isaac.urdf import _urdf
                urdf_interface = _urdf.acquire_urdf_interface()

                import_config = _urdf.ImportConfig()
                import_config.merge_fixed_joints = False
                import_config.fix_base = True
                import_config.make_default_prim = False
                import_config.create_physics_scene = False
                import_config.default_drive_type = 1
                import_config.default_drive_strength = 1e4
                import_config.default_position_drive_damping = 1e3

                dest_path = "/World/ur5_new"
                result = urdf_interface.parse_urdf(URDF_PATH, import_config)
                prim_path = urdf_interface.import_robot(
                    URDF_PATH, result, import_config, dest_path
                )
                print(f"Imported UR5 to: {prim_path}")
                print("Check /World/ur5_new in the Stage panel.")
                print("Then: delete old /World/ur5, rename ur5_new -> ur5, save.")

            except Exception as e2:
                print(f"[ERROR] Could not import URDF: {e2}")
                print("\nManual fix: Isaac Sim menu -> Isaac Utils -> URDF Importer")
                print(f"  Select: {URDF_PATH}")
                print("  Target path: /World/ur5_new")
                print("  Enable: Fix Base, Position Drive")
                print("  Then delete old /World/ur5, rename ur5_new -> ur5")

        except Exception as e:
            print(f"[ERROR] URDF import failed: {e}")
            print("\nManual fix: Isaac Sim menu -> Isaac Utils -> URDF Importer")
            print(f"  Select: {URDF_PATH}")

    print("\n=== DONE ===")
