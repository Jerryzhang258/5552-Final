# 在 Isaac Sim Script Editor 中运行：查找桌面的实际高度，用于放置物品。

import omni.usd
from pxr import Usd, UsdGeom, Gf

def main():
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("No stage.")
        return

    # 查看 packing_table 的所有 xform ops
    table = stage.GetPrimAtPath("/World/packing_table")
    if table:
        xf = UsdGeom.Xformable(table)
        print("packing_table xformOps:")
        for op in xf.GetOrderedXformOps():
            print(f"  {op.GetOpName()} = {op.Get()}")

        # 计算 packing_table 的世界变换
        cache = UsdGeom.XformCache()
        world_tf = cache.GetLocalToWorldTransform(table)
        print(f"packing_table 世界变换矩阵对角: translate = {world_tf.ExtractTranslation()}")

    # SM_CratePacking_Table_A1 的局部变换
    crate = stage.GetPrimAtPath("/World/packing_table/SM_CratePacking_Table_A1")
    if crate:
        xf = UsdGeom.Xformable(crate)
        print("\nSM_CratePacking_Table_A1 xformOps:")
        for op in xf.GetOrderedXformOps():
            print(f"  {op.GetOpName()} = {op.Get()}")

        # 世界包围盒
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(crate)
        rng = bbox.GetRange()
        print(f"\nSM_CratePacking_Table_A1 世界包围盒:")
        print(f"  min = {rng.GetMin()}")
        print(f"  max = {rng.GetMax()}")
        print(f"  桌面大约在世界 Z = {rng.GetMax()[2]:.4f}")

        # 换算到 packing_table 局部坐标
        if table:
            cache = UsdGeom.XformCache()
            table_world = cache.GetLocalToWorldTransform(table)
            table_world_inv = table_world.GetInverse()
            local_max_z = (table_world_inv * Gf.Vec4d(0, 0, rng.GetMax()[2], 1))[2]
            print(f"  桌面在 packing_table 局部 Z ≈ {local_max_z:.4f}")

    # ur5 的位置
    ur5 = stage.GetPrimAtPath("/World/ur5")
    if ur5:
        xf = UsdGeom.Xformable(ur5)
        print("\nur5 xformOps:")
        for op in xf.GetOrderedXformOps():
            print(f"  {op.GetOpName()} = {op.Get()}")

    # 检查一个占位方块的当前位置
    for name in ["green_cube", "red_cube", "yellow_cube"]:
        p = stage.GetPrimAtPath(f"/World/packing_table/{name}")
        if p:
            xf = UsdGeom.Xformable(p)
            print(f"\n{name} xformOps:")
            for op in xf.GetOrderedXformOps():
                print(f"  {op.GetOpName()} = {op.Get()}")

if __name__ == "__main__":
    main()
