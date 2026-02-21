#!/usr/bin/env python3
# 在 Isaac Sim 中运行：移除 SPARC.usd 里缺失的 panda 与 revel 引用，另存为 SPARC_ur5_only.usd。
# 用法：在 Isaac Sim 中先打开 SPARC.usd，再打开 Window -> Script Editor，粘贴本脚本并运行。

def main():
    try:
        from pxr import Usd
        import omni.usd
    except ImportError as e:
        print("请在 Isaac Sim 的 Script Editor 中运行此脚本。", e)
        return

    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if not stage:
        print("未打开任何 Stage。请先 File -> Open 打开 SPARC.usd 再运行。")
        return

    # 要删除的 prim 路径（缺失 panda.usd 与 revel_assets 的引用）
    paths_to_remove = [
        "/World/panda",
        "/World/ur5/stanley_knife",
        "/World/ur5/tape",
        "/World/ur5/allen_key",
        "/World/ur5/wood_chisel",
        "/World/packing_table/plastic_pallet",
        "/World/packing_table/plastic_pallet_01",
    ]

    removed = []
    for path in paths_to_remove:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            stage.RemovePrim(path)
            removed.append(path)

    if not removed:
        print("没有找到需要删除的 prim（可能已删除或路径不同）。")
        return

    print("已删除:", removed)

    # 另存为 SPARC_ur5_only.usd（与 SPARC.usd 同目录，或项目根目录）
    import os
    root_layer = stage.GetRootLayer()
    current_path = getattr(root_layer, "realPath", None) or getattr(root_layer, "identifier", None) or ""
    if current_path and os.path.isabs(current_path) and os.path.exists(os.path.dirname(current_path)):
        base_dir = os.path.dirname(current_path)
    else:
        try:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        except NameError:
            base_dir = os.path.expanduser("~/nlp-pnp-robotic-arm")
    out_path = os.path.join(base_dir, "SPARC_ur5_only.usd")

    # 保存到新文件（不覆盖 SPARC.usd）
    try:
        stage.Export(out_path)
        print("已另存为:", out_path)
        print("下次在 Isaac Sim 中直接打开 SPARC_ur5_only.usd 即可避免缺失资产警告。")
    except Exception as e:
        print("Export 失败:", e)
        print("请手动在 Isaac Sim 中 File -> Save As 另存为 SPARC_ur5_only.usd。")


if __name__ == "__main__":
    main()
