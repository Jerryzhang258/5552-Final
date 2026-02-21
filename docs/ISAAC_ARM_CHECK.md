# 中继已发 /isaac_joint_commands，但 Isaac 里机械臂不动

说明：ROS2 侧正常（有 "Relayed trajectory" 日志），问题在 **Isaac Sim 的图或 ROS2 桥**。

**若 `ros2 topic info /isaac_joint_commands` 显示 Subscription count: 1，但机械臂仍不动**：多半是图里 Subscriber→Articulation Controller 的连线或参数不对。可直接用 **方案 B（直接驱动）** 绕过图。

## 在 Isaac Sim 里按顺序做

1. **打开场景并 Play**  
   打开 `SPARC_ur5_fixed.usd`（或你的带 UR5 场景），点击 **Play**。

2. **打开 Script Editor**  
   菜单：Window → Script Editor（或 Console）。

3. **运行图诊断并修复**  
   在 Script Editor 里打开并运行（或粘贴运行）：  
   **`scripts/isaac_debug_graph_connections.py`**  
   看输出里是否有：
   - `[FIX] Subscriber topicName = isaac_joint_commands`
   - `[FIX] Connected outputs:positionCommand -> inputs:positionCommand`
   - `robotPath`、`targetPrim`、`jointIndices` 被设置

4. **保存场景**  
   菜单 **File → Save**（或 Ctrl+S），确保场景已保存。

5. **Stop 再 Play**  
   点 **Stop**，再点 **Play**，让图重新加载。

6. **再在 RVIZ 里 Plan & Execute**  
   看 Isaac 里机械臂是否动。

---

## 若仍不动：手动检查图

- 在 **Stage** 里找到 **/Graph/ROS_JointStates**。
- 确认有一个 **ROS2 Subscriber** 节点，且 **inputs:topicName** = `isaac_joint_commands`（无前缀 `/`）。
- 确认该 Subscriber 的 **outputs:positionCommand** 已连到 **Articulation Controller** 的 **inputs:positionCommand**。
- 确认 Articulation Controller 的 **inputs:robotPath** 指向你的机械臂根（如 `/World/ur5`），**inputs:targetPrim** 为 articulation 根（如 `/World/ur5/root_joint`），**inputs:jointIndices** 为 `[0,1,2,3,4,5]`（6 轴手臂）。

---

## 方案 B：直接驱动（绕过 OmniGraph，机械臂没动时优先用这个）

不依赖 OmniGraph，在 Isaac 里用脚本直接订阅 `/isaac_joint_commands` 并写入关节目标。

1. **Isaac Sim** 用 **带 ROS2 桥** 的方式启动（如 `./scripts/launch_isaac_sim.sh`），打开场景（如 SPARC_ur5_fixed.usd），点 **Play**。
2. 打开 **Window → Script Editor**，在编辑器里**打开并运行**：**`scripts/isaac_ros2_direct_drive_bridge.py`**。
3. 看到 `Direct drive bridge ACTIVE` 以及（若有消息）`[DirectDrive] Found joints: ...`、`Applied to 6 joints` 后，**不要关 Script Editor**，保持 Isaac 在 **Play**。
4. 在 **RVIZ** 里 **Plan & Execute**，Isaac 里机械臂应跟着动。

**若仍不动**：看 Script Editor 是否打印 `[DirectDrive] Applied to N joints`。若无，说明 Isaac 没收到 topic（检查 ROS2 桥、DDS 域）。若有 Applied 但臂不动，多半是关节路径不对：脚本会从 `/World/ur5` 下自动查找 6 个关节 prim；若机器人不在 `/World/ur5`，在脚本里改 `ROBOT_ROOT = "/World/你的机器人路径"` 后重跑。

---

## 可选：用 topic 直接测 Isaac 是否收得到

在**本机终端**（已 source ROS2）执行一次：

```bash
ros2 topic pub --once /isaac_joint_commands sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_link], position: [0.0, -1.0, 1.2, 0.0, 0.0, 0.0]}"
```

若此时 **Isaac 里机械臂有动**，说明 Isaac 能收到 topic，问题只是图里连线或参数；若**完全不动**，可能是 Isaac 的 ROS2 桥未起或 DDS 域不一致（Isaac 与 ROS2 是否同一网络/同一机）。
