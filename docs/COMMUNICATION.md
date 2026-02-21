# 通信逻辑（简单有效）

整条系统就两条通信链：**手动控制机械臂** 和 **语音/聊天抓取**。所有节点和话题都围绕这两条链。

---

## 一、两条链路（概览）

```
链路 A：手动控制（RVIZ → Isaac）
  RVIZ Plan & Execute
    → MoveIt 发 action: arm_controller/follow_joint_trajectory
    → 轨迹中继 收 action，发 /isaac_joint_commands
    → Isaac 收 /isaac_joint_commands，驱动机械臂

链路 B：聊天抓取（Chatbot → YOLO → 机械臂 → Isaac）
  用户在网页说「抓 red cube」「放盒子 1」
    → Chatbot 发 /target_class_cmd（如 "red_cube,1"）
    → target_publisher 收指令 + /Yolov8_Inference，算 (x,y,yaw)，发 /target_point
    → ur5_pick_place 收 /target_point，用 MoveIt 执行抓放
    → MoveIt 轨迹仍经 轨迹中继 → /isaac_joint_commands → Isaac
```

两条链在「MoveIt → 轨迹中继 → /isaac_joint_commands → Isaac」这一段是共用的。

---

## 二、话题一览（谁发谁收）

| 话题 | 类型 | 谁发 | 谁收 | 用途 |
|------|------|------|------|------|
| `arm_controller/follow_joint_trajectory` | action | MoveIt | 轨迹中继 | 规划好的关节轨迹 |
| `/isaac_joint_commands` | JointState | 轨迹中继 | Isaac | 驱动仿真机械臂 |
| `/target_class_cmd` | String | Chatbot | target_publisher | 要抓的类别+盒子，如 "red_cube,1" |
| `/Yolov8_Inference` | Yolov8Inference | yolov8_obb_publisher | target_publisher | 检测到的物体框 |
| `/target_point` | Float64MultiArray | target_publisher | ur5_pick_place | [x, y, yaw, box] 抓取目标 |
| `/pick_place_status` | String | ur5_pick_place | Chatbot | 状态 "IDLE" 等，用于界面提示 |
| `/rgb` | Image | 相机/Isaac | yolov8_obb_publisher | 图像输入（YOLO 需要） |

---

## 三、节点与脚本（各干一件事）

| 节点/脚本 | 作用 |
|-----------|------|
| **轨迹中继** `trajectory_to_isaac_relay.py` | 收 action，发 `/isaac_joint_commands`；让 RVIZ 和 Chatbot 的规划都能到 Isaac。 |
| **ur5_pick_place_cpp_r** | 收 `/target_point`，用 MoveIt 做抓放，并发 `/pick_place_status`。 |
| **target_publisher** | 收 `/target_class_cmd` + `/Yolov8_Inference`，算 (x,y,yaw)，发 `/target_point`。 |
| **cmd_bridge** | 网页聊天 + 调用 LLM，发 `/target_class_cmd`，收 `/pick_place_status`、`/Yolov8_Inference`。 |
| **yolov8_obb_publisher** | 收 `/rgb`，发 `/Yolov8_Inference`。 |

Isaac 侧：要么用 **OmniGraph** 订阅 `/isaac_joint_commands`，要么用 **isaac_ros2_direct_drive_bridge.py** 直接写关节目标（绕过图）。

---

## 四、Isaac 与 RVIZ 姿势一致

- **轨迹中继**已按时间顺序发送轨迹的**每一帧**（不再只发最后一点），Isaac 会按与 RVIZ 相同的路径运动，终点姿势一致。
- 若某关节在 Isaac 里方向与 RVIZ 相反：在 **isaac_ros2_direct_drive_bridge.py** 里给该关节设符号，例如 `JOINT_SIGN = {"shoulder_lift_joint": -1}`。

---

## 五、排查（简单有效）

- **RVIZ 动、Isaac 不动**：先跑 `./scripts/check_isaac_arm_chain.sh`；再在 Isaac Play 下运行 `isaac_debug_graph_connections.py` 并 Save；仍不行就用 `isaac_ros2_direct_drive_bridge.py` 直接驱动。
- **说了「抓」但不动**：看 `/target_class_cmd`、`/Yolov8_Inference`、`/target_point` 是否都有数据；ur5_pick_place 终端是否有「Accepted new target_point」。
- **Chatbot 报错 fastapi**：`conda activate ros2_humble_py310` 且 `pip install -r requirements.txt`。

---

**全部重启**：在本机执行 `./restart_demo.sh` 会先结束本次 RVIZ、中继、Chatbot、UR5 节点等，再重新跑 `run_full_demo.sh`。Isaac Sim 需手动关掉或保留后重新打开场景并 Play。

---

以上即整条系统的通信逻辑：两条链、一张表、几个节点、三步排查，简单有效。
