# SyntheticData 报错：stageFrameHistoryCount of 3

## 现象

Isaac Sim 启动或运行时报：

```text
[Error] [omni.syntheticdata.scripts.extension] SyntheticData extension needs at least a stageFrameHistoryCount of 3
```

## 说明

- 这是 **SyntheticData / Replicator** 扩展的要求：需要“阶段帧历史”至少为 3。
- 若不使用 **合成数据生成 / Replicator 录制** 等功能，可以**忽略**该错误，不影响本项目的机械臂、YOLO、抓放 demo。
- 若希望去掉该报错或要使用 SyntheticData，可尝试下面设置。

## 解决方法

### 方法一：在 Isaac Sim 里改设置（推荐）

1. 菜单 **Edit → Preferences**（或 **Window → Settings**）。
2. 在搜索框输入 **stageFrameHistory** 或 **frame history**。
3. 找到与 **Stage Frame History Count** 或 **runLoops** 相关的项，将数值改为 **3** 或更大（例如 4）。
4. 若在 **Preferences** 里没有，可试 **Window → Stats** 或扩展 **omni.kit.loop-default** 相关设置。

### 方法二：改用户配置文件

Isaac Sim 的用户配置一般在：

```text
~/.local/share/ov/data/Kit/omni.app.full/1.0/user.config.json
```

在 JSON 里增加或修改（具体路径以你本机可用的 setting 为准，可能类似）：

```json
"/app/runLoops/main/stageFrameHistoryCount": 3
```

或：

```json
"/app/runLoops/present/stageFrameHistoryCount": 3
```

保存后重启 Isaac Sim。若 key 不对，可用 **Edit → Preferences** 里改对应项后，再打开该 JSON 看实际写入的 key。

### 方法三：忽略

不做任何修改，仅忽略该 Error。本仓库的 demo（UR5、轨迹中继、YOLO 抓取、Chatbot）不依赖 SyntheticData，可正常使用。
