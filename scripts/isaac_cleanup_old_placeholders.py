# Run in Isaac Sim Script Editor: remove old duplicate placeholder objects under /World/packing_table.
# The new correct ones are directly under /World (placed inside crates by isaac_add_placeholder_objects.py).

import omni.usd
from pxr import Sdf

OLD_PATHS = [
    "/World/packing_table/green_cube",
    "/World/packing_table/red_cube",
    "/World/packing_table/yellow_cube",
    "/World/packing_table/container_h20",
    "/World/packing_table/stanley_knife",
    "/World/packing_table/allen_key",
    "/World/packing_table/wood_chisel",
    "/World/packing_table/tape",
]

stage = omni.usd.get_context().get_stage()
if not stage:
    print("No stage open.")
else:
    layer = stage.GetRootLayer()
    removed = 0
    for path in OLD_PATHS:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            layer.RemoveSubLayerPath if False else None
            try:
                stage.RemovePrim(path)
                print(f"  [DEL] {path}")
                removed += 1
            except Exception as e:
                print(f"  [WARN] {path}: {e}")
        else:
            print(f"  [SKIP] {path} (not found)")
    print(f"\nRemoved {removed} old placeholders. File -> Save to persist.")
