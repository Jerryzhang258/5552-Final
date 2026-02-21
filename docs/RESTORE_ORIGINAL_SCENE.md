# 从原本的 GitHub 恢复初始场景

原仓库的**初始场景**是 **SPARC.usd**（根目录），见：<https://github.com/sahilrajpurkar03/nlp-pnp-robotic-arm>。

---

## 方式一：直接下载覆盖（推荐）

在项目根目录执行（会覆盖当前 `SPARC.usd`，建议先备份）：

```bash
cd ~/nlp-pnp-robotic-arm
# 可选：备份当前场景
cp -n SPARC.usd SPARC.usd.bak 2>/dev/null || true
# 从原仓库下载 SPARC.usd
curl -L -o SPARC.usd "https://github.com/sahilrajpurkar03/nlp-pnp-robotic-arm/raw/master/SPARC.usd"
```

恢复后，在 Isaac Sim 里 **File → Open → SPARC.usd** 即可用原版初始场景。

---

## 方式二：用 git 从原仓库取回

若本项目是从原仓库 fork 或 clone 的：

```bash
cd ~/nlp-pnp-robotic-arm
git fetch origin
git checkout origin/master -- SPARC.usd
```

若原仓库是别的 remote（例如 `upstream`），把 `origin` 换成对应 remote 名。

---

## 把原版 USD 里的东西都放进 fixed USD

若你已有原版 **SPARC.usd**，想看到「原版里有多少东西」并把能复制的都合并进 **SPARC_ur5_fixed.usd**：

1. 在 Isaac Sim 里先打开 **SPARC_ur5_fixed.usd**（当前要写入的场景）。
2. **Window → Script Editor** 中运行：**`scripts/isaac_copy_original_usd_into_fixed.py`**。
3. 脚本会打印原版 SPARC.usd 中 prim 数量和路径，并把原版 `/World` 下、当前场景里**还没有**的 prim 复制进来（不覆盖你 fixed 里已有的机械臂等）。
4. **File → Save** 保存，合并结果就写入你的 fixed USD。

若原版里有外部引用（Revel、Panda 等）且本地没有，对应物体可能仍会缺失，可再运行 **`scripts/isaac_add_placeholder_objects.py`** 在桌上补立方体。

---

## 说明

- **SPARC.usd**：原仓库的初始场景（可能含 Panda、Revel 等引用；若缺资产会报错，可用本仓库的 `scripts/isaac_remove_missing_assets.py` 另存为仅 UR5 场景）。
- **SPARC_ur5_fixed.usd**：本仓库里已修好的 UR5 场景（去缺失资产、接好关节等），日常 demo 建议继续用这个；只有需要「完全原版初始」时再按上面恢复 **SPARC.usd**。
