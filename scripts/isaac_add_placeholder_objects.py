# Run in Isaac Sim Script Editor: place 3 colored cubes ON TOP of the table.
# Usage: Open SPARC_ur5_fixed.usd -> Script Editor -> run -> File -> Save.

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf


def get_world_bbox(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return None
    try:
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = cache.ComputeWorldBound(prim)
        rng = bbox.GetRange()
        if rng.IsEmpty():
            return None
        return rng.GetMin(), rng.GetMax()
    except Exception:
        return None


def find_table_surface_z(stage):
    """Find table surface Z by looking at the bottom of crates (they sit on the table)."""
    ct = stage.GetPrimAtPath("/World/packing_table/SM_CratePacking_Table_A1")
    if not ct:
        return None, None, None

    crate_bottoms = []
    for child in ct.GetChildren():
        name = child.GetName()
        if "Crate" in name or "crate" in name:
            bb = get_world_bbox(stage, str(child.GetPath()))
            if bb:
                crate_bottoms.append(bb[0][2])  # min Z = bottom of crate

    if crate_bottoms:
        surface_z = min(crate_bottoms)
        print(f"  Table surface Z = {surface_z:.4f} (from crate bottoms)")
    else:
        # Fallback: use top of the full assembly bbox
        bb = get_world_bbox(stage, str(ct.GetPath()))
        if bb:
            surface_z = bb[1][2]  # max Z
            print(f"  Table surface Z ~ {surface_z:.4f} (from assembly top, rough)")
        else:
            surface_z = 0.35
            print(f"  Table surface Z = {surface_z} (hardcoded fallback)")

    # Table center XY
    bb = get_world_bbox(stage, "/World/packing_table")
    if bb:
        cx = (bb[0][0] + bb[1][0]) / 2.0
        cy = (bb[0][1] + bb[1][1]) / 2.0
    else:
        cx, cy = -3.85, 0.03

    return surface_z, cx, cy


def delete_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)
        return True
    return False


def create_cube(stage, name, size, wx, wy, wz, color):
    path = f"/World/{name}"
    delete_prim(stage, path)
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
    cube = UsdGeom.Cube.Define(stage, f"{path}/geom")
    cube.CreateSizeAttr(size)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/geom"))
    UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(path))
    print(f"  [OK] {path}  size={size}m  world=({wx:.3f}, {wy:.3f}, {wz:.3f})")


stage = omni.usd.get_context().get_stage()
if not stage:
    print("No stage open.")
else:
    # Clean old
    for n in ["green_cube", "red_cube", "yellow_cube"]:
        delete_prim(stage, f"/World/{n}")
        delete_prim(stage, f"/World/packing_table/{n}")

    surface_z, cx, cy = find_table_surface_z(stage)
    if surface_z is None:
        print("[ERROR] Could not determine table surface.")
    else:
        CUBE_SIZE = 0.05
        # Place cube center well above surface (+10cm extra to avoid clipping into table)
        z = surface_z + CUBE_SIZE / 2.0 + 0.10
        print(f"  Cube center Z = {z:.4f}  (surface {surface_z:.4f} + {CUBE_SIZE/2} + 0.005)")
        print(f"  Table center XY = ({cx:.3f}, {cy:.3f})")
        print("")

        create_cube(stage, "green_cube",  CUBE_SIZE, cx - 0.15, cy,        z, (0.1, 0.8, 0.1))
        create_cube(stage, "red_cube",    CUBE_SIZE, cx,        cy + 0.12, z, (0.9, 0.1, 0.1))
        create_cube(stage, "yellow_cube", CUBE_SIZE, cx + 0.15, cy,        z, (0.9, 0.9, 0.1))

        print("\nDone! File -> Save to persist.")
        print(f"Cubes are at Z={z:.3f}, which is {CUBE_SIZE/2+0.005:.3f}m above table surface (Z={surface_z:.3f}).")
