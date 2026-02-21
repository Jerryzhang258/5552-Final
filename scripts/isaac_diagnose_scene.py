# 在 Isaac Sim Script Editor 中运行：诊断当前打开的场景，列出 /World 下有哪些东西。
# 用法：打开场景 -> Window -> Script Editor -> 粘贴运行 -> 看输出。

import omni.usd
from pxr import Usd, UsdGeom

def main():
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("No stage open.")
        return

    # 当前打开的文件
    layer = stage.GetRootLayer()
    print(f"当前场景: {layer.GetRealPath() if hasattr(layer, 'GetRealPath') else layer.identifier}")
    print("")

    # /World 的直接子节点
    world = stage.GetPrimAtPath("/World")
    if not world:
        print("/World 不存在！场景可能为空。")
        return

    print("========== /World 直接子节点 ==========")
    for child in world.GetChildren():
        path = str(child.GetPath())
        typ = child.GetTypeName()
        n_children = len(child.GetChildren())
        print(f"  {path}  [{typ}]  子节点数: {n_children}")

    # /World/packing_table 详情
    table = stage.GetPrimAtPath("/World/packing_table")
    if table:
        print("")
        print("========== /World/packing_table 子节点 ==========")
        for child in table.GetChildren():
            path = str(child.GetPath())
            typ = child.GetTypeName()
            has_payload = child.HasPayload() if hasattr(child, "HasPayload") else False
            has_ref = child.HasAuthoredReferences() if hasattr(child, "HasAuthoredReferences") else False
            n_sub = len(child.GetChildren())
            tag = ""
            if has_payload:
                tag += " [PAYLOAD]"
            if has_ref:
                tag += " [REF]"
            print(f"  {path}  [{typ}]  子节点: {n_sub}{tag}")
    else:
        print("\n/World/packing_table 不存在！")

    # /World/ur5 详情
    ur5 = stage.GetPrimAtPath("/World/ur5")
    if ur5:
        print("")
        print("========== /World/ur5 直接子节点 ==========")
        for child in ur5.GetChildren():
            path = str(child.GetPath())
            typ = child.GetTypeName()
            n_sub = len(child.GetChildren())
            print(f"  {path}  [{typ}]  子节点: {n_sub}")
    else:
        print("\n/World/ur5 不存在！")

    # 总 prim 数
    total = 0
    for _ in stage.Traverse():
        total += 1
    print(f"\n总 prim 数: {total}")

if __name__ == "__main__":
    main()
