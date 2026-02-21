"""
Run in Isaac Sim Script Editor: diagnose /World/ur5 hierarchy, Articulation, and physics.
Steps: Open SPARC.usd -> Window -> Script Editor -> paste this -> Run
"""

import omni.usd
from pxr import Usd

ctx = omni.usd.get_context()
stage = ctx.get_stage()

if not stage:
    print("[ERROR] No stage open. Please open SPARC.usd first.")
else:
    # 1) Check /World/ur5 exists
    ur5 = stage.GetPrimAtPath("/World/ur5")
    if not ur5 or not ur5.IsValid():
        print("[ERROR] /World/ur5 does not exist!")
        w = stage.GetPrimAtPath("/World")
        if w and w.IsValid():
            print("Children under /World:")
            for c in w.GetChildren():
                print("  ", c.GetPath())
    else:
        # 2) Print prim tree under /World/ur5 (max depth 4)
        print("=== /World/ur5 tree ===")
        for p in Usd.PrimRange(ur5):
            depth = len(str(p.GetPath()).split("/")) - 3
            if depth > 4:
                continue
            indent = "  " * depth
            schemas = [str(s) for s in p.GetAppliedSchemas()]
            tags = []
            if any("ArticulationRootAPI" in s for s in schemas):
                tags.append("ART_ROOT")
            if any("RigidBodyAPI" in s for s in schemas):
                tags.append("RIGID")
            tname = str(p.GetTypeName())
            if "Joint" in tname:
                tags.append("JOINT")
            tag = " " + str(tags) if tags else ""
            print(f"{indent}{p.GetName()} ({tname}){tag}")

        # 3) Check base_link
        print("\n=== Check /World/ur5/base_link ===")
        bl = stage.GetPrimAtPath("/World/ur5/base_link")
        if bl and bl.IsValid():
            print("FOUND: /World/ur5/base_link, type:", bl.GetTypeName())
            print("  schemas:", [str(s) for s in bl.GetAppliedSchemas()])
        else:
            print("NOT FOUND: /World/ur5/base_link")

        # 4) Find all ArticulationRootAPI prims
        print("\n=== All prims with ArticulationRootAPI ===")
        count = 0
        for p in Usd.PrimRange(stage.GetPrimAtPath("/World")):
            if any("ArticulationRootAPI" in str(s) for s in p.GetAppliedSchemas()):
                print(" ", p.GetPath())
                count += 1
        if count == 0:
            print("  NONE found! UR5 may be missing ArticulationRootAPI.")

        # 5) Find all RigidBodyAPI prims under ur5
        print("\n=== RigidBodyAPI prims under /World/ur5 ===")
        count2 = 0
        for p in Usd.PrimRange(ur5):
            if any("RigidBodyAPI" in str(s) for s in p.GetAppliedSchemas()):
                print(" ", p.GetPath())
                count2 += 1
        if count2 == 0:
            print("  NONE found! UR5 links may be missing RigidBodyAPI.")

    print("\n=== DONE ===")
