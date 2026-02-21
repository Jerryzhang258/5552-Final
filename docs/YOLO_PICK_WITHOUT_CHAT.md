# 恢复桌上物品 + 不用对话、用 YOLO 抓取

## 1. 把方块放进 fixed 的 USD 里（框里没有东西时必做）

**现象**：SPARC_ur5_fixed.usd 里没有绿/红/黄方块，或物体被抓走后想再摆回去。

**做法（做一次并 Save，方块就会永久保存在该 USD 里）**：

在 **Isaac Sim** 里：

1. 打开场景 **SPARC_ur5_fixed.usd**（Play 可选）。
2. **Window → Script Editor**，打开并运行：**`scripts/isaac_add_placeholder_objects.py`**。
   - 会把**已有**的 green_cube / red_cube / **yellow_cube** 移回桌上默认位置；
   - 若某个被删了，会**重新创建**（含黄色方块）。
3. **File → Save** 保存场景。保存后，这些方块就写在 **SPARC_ur5_fixed.usd** 里，下次直接打开就有。

之后若物体被移乱或删掉，可再运行该脚本一次，再 Save。

---

## 2. 不用对话、直接用 YOLO 抓取

不打开 Chatbot 网页时，只要这些在跑即可：

- **Isaac Sim**（场景 Play，相机发 `/rgb`）
- **yolov8_obb_publisher**（订阅 `/rgb`，发布 `/Yolov8_Inference`）
- **target_publisher**（订阅 `/target_class_cmd` 和 `/Yolov8_Inference`，发布 `/target_point`）
- **ur5_pick_place_cpp_r**（订阅 `/target_point`，执行抓放）
- **轨迹中继**（让 Isaac 机械臂动）

然后在本机终端发一条「抓哪个、放哪盒」的指令，等价于在聊天框里选物体+盒子：

```bash
cd ~/nlp-pnp-robotic-arm
source install/setup.bash
./scripts/pick_by_label.sh red_cube 1
```

- `red_cube`：要抓的类别（需在 `llm_mapper.py` 的 VALID_LABELS 里，且 YOLO 能检测到）。
- `1`：放到盒子 1（可改为 `2`）。

脚本会发布一次 **/target_class_cmd**（如 `"red_cube,1"`），target_publisher 收到后等 YOLO 检测到 red_cube 就会发 **/target_point**，ur5_pick_place 执行抓放。

**其他示例：**

```bash
./scripts/pick_by_label.sh green_cube 2
./scripts/pick_by_label.sh yellow_cube 1
```

---

## 3. 前提检查

- **相机图进 YOLO**：Isaac 里相机要发布到 **/rgb**，否则 YOLO 无检测，不会发 /target_point。
- **场景里有对应物体**：如 red_cube、green_cube、yellow_cube（用上面「恢复桌上物品」脚本添加并 Save）。
- **run_full_demo 已起**：或至少已单独起 target_publisher、yolov8_obb_publisher、ur5_pick_place、轨迹中继（可不起 Chatbot）。
