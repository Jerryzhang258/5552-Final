# 原仓库 Demo 代码结构参考 (sahilrajpurkar03/nlp-pnp-robotic-arm)

不搭原仓库环境，只看 demo 代码怎么写时参考本文档。原仓库：<https://github.com/sahilrajpurkar03/nlp-pnp-robotic-arm>。

---

## 1. 启动脚本

### main_launch.sh（原版）

- 开两个 `gnome-terminal`：
  1. **RVIZ + MoveIt**：`ros2 launch ur5_moveit_config demo.launch.py`
  2. **UR5 抓放节点**：`ros2 run ur5_moveit_config ur5_pick_place_cpp_r`
- 中间 `sleep 3`。

### pick_place_chatbot_ui/launch.sh（原版）

- 后台启动：`target_publisher.py` → `yolov8_obb_publisher` → `cmd_bridge.py`
- `sleep 3` 后用 Firefox 打开 `http://localhost:8000`。

---

## 2. 话题与数据流

| 话题 | 类型 | 说明 |
|------|------|------|
| `/target_class_cmd` | `std_msgs/String` | Chatbot 发出的目标类别（及可选 box），格式 `"label"` 或 `"label,box"` |
| `/Yolov8_Inference` | `yolov8_msgs/Yolov8Inference` | YOLO 检测结果 |
| `/target_point` | `std_msgs/Float64MultiArray` | 目标点 [x, y, yaw, box]，单位 m / rad，box 为 1/2 或 -1 |
| `/pick_place_status` | `std_msgs/String` | 机械臂状态，如 "IDLE" |

流程：**Chatbot** → `/target_class_cmd` → **target_publisher** 结合 YOLO 算 (x,y,yaw,box) → `/target_point` → **ur5_pick_place** 执行 MoveIt 抓放。

---

## 3. pick_place_chatbot_ui 核心逻辑

### cmd_bridge.py

- **ROS 节点**：发布 `/target_class_cmd`，订阅 `/pick_place_status`、`/Yolov8_Inference`。
- **FastAPI**：`/` 返回前端 HTML，`/chat` 接收用户输入，调 `llm_chat_or_pick()`，根据返回类型：
  - `pick`：直接 `publish_label(label, box)`
  - `ask_box`：存队列，等用户选 box（1/2）
  - `pick_queue`：发整队，先发第一个并返回 “Picking … in progress”
- **WebSocket** `/ws`：把机器人状态/完成消息推给前端。
- 当 `/pick_place_status` 回到 "IDLE" 且 `last_pick` 有队列时，自动发下一个 `publish_label`。

### target_publisher.py

- 订阅 `/target_class_cmd`、`/Yolov8_Inference`。
- 用**相机内参 (fx, fy, cx, cy)** 和**外参 (C, R)** 把检测框中心像素转为机器人基座标 (x, y)；用 bbox 边长算 yaw。
- 只对当前 `target_class` 且未发布过时发布一次 `Float64MultiArray`：[x, y, yaw, box]，带 offset 微调。

### llm_mapper.py

- **VALID_LABELS**：如 banana, green_cube, red_cube, yellow_cube, measuring_tape, knife 等。
- **llm_chat_or_pick(user_text, visible_objects)** 返回 `{ "type": "chat"|"ask_box"|"pick"|"pick_queue", "reply": "...", "label"/"labels"/"box" }`：
  - “revel items” → 选 revel 物体 → `ask_box`。
  - 用户说 “1” 或 “2” 且有待选队列 → `pick_queue`。
  - 检测到 pick 意图 + 识别到 label → `ask_box`（或直接 pick 若已带 box）。
  - 模糊匹配到 label → 先问 “Did you mean … ? (yes/no)”。
  - 其余走 **Ollama** (`ollama run phi3`) 做闲聊，返回 `type: "chat"`。

---

## 4. 机械臂侧（原仓库）

- **ur5_pick_place_cpp_r**：C++ 节点，订阅 `/target_point`，用 MoveIt 规划并执行抓取、放置（box 1 或 2），并发布 `/pick_place_status`。

---

## 5. 与本仓库对应关系

| 原仓库 | 本仓库 |
|--------|--------|
| main_launch.sh | main_launch.sh / run_full_demo.sh 前两段 |
| pick_place_chatbot_ui/launch.sh | run_full_demo.sh 中 Chatbot 启动段 |
| cmd_bridge + target_publisher + llm_mapper | pick_place_chatbot_ui/ 下同逻辑 |
| ur5_pick_place_cpp_r | src/ur5_moveit_config 中同名节点 |

本仓库 `run_full_demo.sh` 已按上述流程串起：编译 → RVIZ+MoveIt → ur5_pick_place_cpp_r → Chatbot(YOLO + target_publisher + cmd_bridge)。只需 Isaac Sim 打开并 Play 场景，ROS 与 Isaac 通过 joint 话题/图连接即可。
