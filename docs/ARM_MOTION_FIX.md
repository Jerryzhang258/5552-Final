# 机械臂通信（尽量简单）

**链路：** RVIZ Plan & Execute → action `arm_controller/follow_joint_trajectory` → 中继 → `/isaac_joint_commands` → Isaac 动。

- 中继：`scripts/trajectory_to_isaac_relay.py`（run_full_demo 会自动起）。
- **机械臂还是不动时**：先跑排查脚本  
  `./scripts/check_isaac_arm_chain.sh`  
  按输出检查：① 中继是否在发 ② 在 Isaac **Play** 下运行 `isaac_debug_graph_connections.py` 并 **Save**。
