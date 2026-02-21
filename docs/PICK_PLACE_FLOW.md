# 根据原版 GitHub 怎么做抓取物品

原仓库流程：**Chatbot 说「抓某某」→ 选盒子 → YOLO 检测该物体 → 算出 (x,y,yaw) → 发 /target_point → ur5_pick_place 用 MoveIt 执行抓放 → 轨迹经中继到 Isaac。**

---

## 1. 先保证这些都在跑

- **Isaac Sim**：场景打开并 **Play**，图已修好（运行过 `isaac_debug_graph_connections.py` 并 Save）。
- **run_full_demo.sh** 已跑：RVIZ、MoveIt、**ur5_pick_place_cpp_r**、**轨迹中继**、**Chatbot**（含 target_publisher、cmd_bridge、yolov8_obb_publisher）。
- 浏览器打开 **http://localhost:8000**（Chatbot 界面）。

---

## 2. 抓取流程（和原版一致）

1. **场景里要有可检测的物体**  
   YOLO 能识别的类别（如 `green_cube`, `red_cube`, `yellow_cube` 等，见 `pick_place_chatbot_ui/llm_mapper.py` 里 `VALID_LABELS`）。  
   若用 Isaac，可在场景里放方块，或在 Isaac 里跑 `scripts/isaac_add_placeholder_objects.py` 加占位物体。

2. **相机图像要进 YOLO**  
   - 仿真：Isaac 里相机图通过 ROS2 发到 `/rgb`（或你配置的相机 topic），**yolov8_obb_publisher** 订阅并发布 `/Yolov8_Inference`。  
   - 若没有 `/rgb`，YOLO 不会出检测，target_publisher 就不会收到目标。

3. **在 Chatbot 里说抓什么、放哪盒**  
   - 例如：「pick the red cube」→ 机器人问放哪个盒子 → 回「1」或「2」。  
   - 或：「pick green cube」→ 「2」。

4. **背后数据流**  
   - Chatbot 发 **/target_class_cmd**（如 `"red_cube,1"`）→ **target_publisher** 收到，设 `target_class=red_cube`、`target_box=1`。  
   - target_publisher 订阅 **/Yolov8_Inference**，当检测到 `red_cube` 时，用像素中心+相机标定算 (x, y, yaw)，发布 **/target_point** = [x, y, yaw, box]。  
   - **ur5_pick_place_cpp_r** 订阅 **/target_point**，收到后调用 MoveIt 执行抓取与放置。  
   - MoveIt 轨迹经 **轨迹中继** 发到 **/isaac_joint_commands**，Isaac 里机械臂跟着动。

---

## 3. 若「说了抓但不动」

按顺序查：

1. **Chatbot 是否发 /target_class_cmd**  
   在终端：`ros2 topic echo /target_class_cmd`，在网页里再发一次「pick red cube」+ 选盒，看是否有输出。

2. **YOLO 是否在发检测**  
   `ros2 topic echo /Yolov8_Inference --once`，看是否有 `yolov8_inference` 数据；相机是否在发图（如 `/rgb`）。

3. **是否发 /target_point**  
   `ros2 topic echo /target_point --once`，在 Chatbot 选完物体和盒子后，看是否有一条 [x, y, yaw, box]。

4. **ur5_pick_place 是否在跑**  
   run_full_demo 会起「UR5 Pick Place」终端；若没起，手动：  
   `ros2 run ur5_moveit_config ur5_pick_place_cpp_r`  
   看该终端是否打印「Accepted new target_point」并执行。

5. **Isaac 是否收到关节命令**  
   若 RVIZ 里机械臂在动但 Isaac 不动，按 `docs/ISAAC_ARM_CHECK.md` 检查 Isaac 图与 topic。

---

## 4. 话题小结（和原版一致）

| 话题 | 方向 | 说明 |
|------|------|------|
| `/target_class_cmd` | Chatbot → target_publisher | 要抓的类别 + 盒子，如 `"red_cube,1"` |
| `/Yolov8_Inference` | yolov8_obb_publisher → target_publisher | 检测结果 |
| `/target_point` | target_publisher → ur5_pick_place | [x, y, yaw, box]（米、弧度、盒号） |
| `/pick_place_status` | ur5_pick_place → Chatbot | 状态，如 "IDLE" |
| `/isaac_joint_commands` | 轨迹中继 → Isaac | 关节目标，驱动仿真里的机械臂 |

按上面做，就和原版 GitHub 的抓取流程一致。
