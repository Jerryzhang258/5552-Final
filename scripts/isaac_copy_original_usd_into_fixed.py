# 在 Isaac Sim Script Editor 中运行：读取原版 SPARC.usd 里有多少东西，并把这些内容合并进当前打开的 fixed USD。
# 用法：先打开 SPARC_ur5_fixed.usd，再 Window -> Script Editor 运行本脚本。
# 会从项目根目录的 SPARC.usd 读取，把 /World 下原版有而当前没有的 prim 复制到当前场景，最后 File -> Save 保存。

import os
import omni.usd
from pxr import Usd, Sdf


def _find_project_root_and_original_usd():
    """从当前打开的场景路径推导项目根目录，避免在 Isaac Script Editor 里 __file__ 指向 /tmp。"""
    def _path_from_identifier(iden):
        if not iden or not (iden.startswith("file:") or iden.startswith("/")):
            return None
        if iden.startswith("file://"):
            path = iden[7:]
        elif iden.startswith("file:"):
            path = iden[5:]
        else:
            path = iden
        try:
            import urllib.parse
            path = urllib.parse.unquote(path)
        except Exception:
            pass
        return path if path else None

    # 1) 优先：当前 stage 的 root layer 路径（打开 SPARC.usd 或 SPARC_ur5_fixed.usd 时）
    try:
        stage = omni.usd.get_context().get_stage()
        if stage:
            root_layer = stage.GetRootLayer()
            if root_layer:
                real_path = None
                if hasattr(root_layer, "GetRealPath"):
                    real_path = root_layer.GetRealPath()
                if not real_path and hasattr(root_layer, "identifier"):
                    real_path = _path_from_identifier(root_layer.identifier)
                if real_path and os.path.isfile(real_path):
                    proj = os.path.dirname(real_path)
                    return proj, os.path.join(proj, "SPARC.usd")
                if real_path and os.path.isdir(os.path.dirname(real_path)):
                    proj = os.path.dirname(real_path)
                    return proj, os.path.join(proj, "SPARC.usd")
    except Exception:
        pass
    # 2) __file__ 在 scripts/ 下（仅当路径在项目内时用，避免 /tmp）
    try:
        f = __file__
        if f and os.path.exists(f):
            ab = os.path.abspath(f)
            if "/tmp" not in ab and "nlp-pnp-robotic-arm" in ab:
                proj = os.path.dirname(os.path.dirname(ab))
                return proj, os.path.join(proj, "SPARC.usd")
    except Exception:
        pass
    # 3) 常见项目路径（按你本机路径优先）
    for candidate in ["/home/abcd1234/nlp-pnp-robotic-arm", os.path.expanduser("~/nlp-pnp-robotic-arm")]:
        if candidate and os.path.isdir(candidate):
            return candidate, os.path.join(candidate, "SPARC.usd")
    return None, None


def list_all_prim_paths(stage):
    """返回 stage 中所有 prim 路径列表"""
    paths = []
    for p in stage.Traverse():
        paths.append(p.GetPath())
    return paths


def list_prims_under(stage, root_path="/World"):
    """返回 root_path 及其下所有 prim 的 (path, typeName)。用 PrimRange 兼容 Isaac 的 pxr。"""
    result = []
    root = stage.GetPrimAtPath(root_path)
    if not root:
        return result
    for p in Usd.PrimRange(root):
        result.append((str(p.GetPath()), p.GetTypeName()))
    return result


def copy_prim_spec(source_layer, target_layer, path):
    """把 source_layer 中 path 及其子树的 spec 复制到 target_layer。path 为 Sdf.Path"""
    if isinstance(path, str):
        path = Sdf.Path(path)
    try:
        Sdf.CopySpec(source_layer, path, target_layer, path)
        return True
    except Exception as e:
        print(f"  [WARN] CopySpec {path} failed: {e}")
        return False


def list_original_items(stage):
    """从原版 stage 里读出「物品」：桌上、机械臂旁的可放置/抓取物体（路径 + 是否外部引用）。"""
    items = []
    def _is_external(p):
        try:
            return p.HasPayload() or (hasattr(p, "HasAuthoredReferences") and p.HasAuthoredReferences())
        except Exception:
            return False
    # 桌上物品：/World/packing_table 的直接子节点
    table = stage.GetPrimAtPath("/World/packing_table")
    if table:
        for p in table.GetChildren():
            path = str(p.GetPath())
            items.append((path, "桌上", "外部引用" if _is_external(p) else "场景内"))
    # 机械臂上的工具/物体：/World/ur5 下非典型关节名的直接子节点
    ur5 = stage.GetPrimAtPath("/World/ur5")
    if ur5:
        skip = {"base_link", "shoulder_pan_link", "shoulder_lift_link", "upper_arm_roll_link",
                "elbow_flex_link", "forearm_roll_link", "wrist_flex_link", "wrist_roll_link",
                "ee_link", "tool0", "gripper", "gripper_base", "left_finger", "right_finger"}
        for p in ur5.GetChildren():
            name = p.GetName()
            if name in skip or "link" in name or "finger" in name:
                continue
            path = str(p.GetPath())
            items.append((path, "ur5旁/手上", "外部引用" if _is_external(p) else "场景内"))
    return items


def merge_world_from_original_into_current():
    stage_current = omni.usd.get_context().get_stage()
    if not stage_current:
        print("请先在 Isaac Sim 中打开 SPARC_ur5_fixed.usd，再运行此脚本。")
        return

    # 检查当前打开的是不是「要写入」的 fixed，而不是原版（否则会复制到原版自己，新复制数=0）
    current_path = ""
    try:
        layer = stage_current.GetRootLayer()
        if hasattr(layer, "GetRealPath"):
            current_path = layer.GetRealPath() or ""
        if not current_path and hasattr(layer, "identifier"):
            iden = layer.identifier
            if iden.startswith("file:"):
                current_path = (iden.replace("file://", "").replace("file:", "").strip())
    except Exception:
        pass
    if current_path and "SPARC.usd" in current_path and "SPARC_ur5_fixed" not in current_path:
        print("【注意】当前打开的是 原版 SPARC.usd，复制会写回原版，不会改 fixed。")
        print("请先 File -> Open 打开 SPARC_ur5_fixed.usd（要写入的场景），再运行本脚本。")
        print("")
        return

    PROJECT_ROOT, ORIGINAL_USD = _find_project_root_and_original_usd()
    print(f"[解析] 项目根: {PROJECT_ROOT}, 原版 USD: {ORIGINAL_USD}")
    if not ORIGINAL_USD or not os.path.isfile(ORIGINAL_USD):
        print("未找到原版 USD。请先 File -> Open 打开项目目录下的 SPARC_ur5_fixed.usd 或 SPARC.usd，或运行 scripts/restore_original_scene.sh 将 SPARC.usd 放到项目根目录。")
        return

    stage_orig = Usd.Stage.Open(ORIGINAL_USD)
    if not stage_orig:
        print("无法打开原版 SPARC.usd。")
        return

    # 0) 原版里有哪些「物品」（桌上、机械臂旁），后面复制时会用到
    item_list = []
    try:
        item_list = list_original_items(stage_orig)
        print("========== 原版场景中的物品 ==========")
        if not item_list:
            print("  （未识别到桌上/ur5旁物品，可能结构不同）")
        else:
            for path, where, ref_type in item_list:
                print(f"  {path}  [{where}] {ref_type}")
        print("")
    except Exception as e:
        print(f"  [WARN] 物品列表: {e}\n")

    # 1) 列出原版里有多少东西
    all_orig = list_all_prim_paths(stage_orig)
    world_orig = list_prims_under(stage_orig, "/World")
    print("========== 原版 SPARC.usd 内容 ==========")
    print(f"  总 prim 数: {len(all_orig)}")
    print(f"  /World 及其下 prim 数: {len(world_orig)}")
    for path, typ in world_orig[:80]:  # 最多打印 80 条
        print(f"    {path}  [{typ}]")
    if len(world_orig) > 80:
        print(f"    ... 还有 {len(world_orig) - 80} 个")

    layer_orig = stage_orig.GetRootLayer()
    layer_cur = stage_current.GetRootLayer()

    # 2) 把原版的东西复制到当前 fixed USD（先复制识别的「物品」，再复制全部缺失的 prim）
    print("========== 正在把原版物体复制到你的 fixed USD ==========")
    copied = []
    # 2a) 先复制「原版物品」里当前没有的
    try:
        for path_str, where, ref_type in item_list:
            if stage_current.GetPrimAtPath(path_str):
                continue
            if copy_prim_spec(layer_orig, layer_cur, path_str):
                copied.append(path_str)
                print(f"  + {path_str}  [{where}]")
    except Exception:
        pass
    # 2b) 再复制原版 /World 下所有「当前没有」的 prim（避免漏掉任何东西）
    for path_str, _ in world_orig:
        if path_str == "/World":
            continue
        if path_str in copied:
            continue
        path = Sdf.Path(path_str)
        if stage_current.GetPrimAtPath(path_str):
            continue
        if copy_prim_spec(layer_orig, layer_cur, path):
            copied.append(path_str)

    print("========== 已合并到当前场景 ==========")
    print(f"  新复制的 prim 数: {len(copied)}")
    for p in copied[:50]:
        print(f"    + {p}")
    if len(copied) > 50:
        print(f"    ... 还有 {len(copied) - 50} 个")

    stage_orig = None
    print("")
    print("请使用 File -> Save 保存当前场景，这样合并内容会写入你的 fixed USD。")
    if copied:
        print("若原版中有外部引用（如 Revel/Panda）且你本地没有，对应物体可能仍显示缺失，可再用 isaac_add_placeholder_objects.py 补立方体。")


if __name__ == "__main__":
    merge_world_from_original_into_current()
