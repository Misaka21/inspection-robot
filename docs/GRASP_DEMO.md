# 抓取 Demo 操作指南（规划中）

> **文档状态：骨架**。抓取方向尚在落地中，本文档先固定**预期操作流程与验收标准**，具体命令/参数/脚本会随 `TODO.md` P0 任务逐步补齐。

## 1. Demo 场景

- AGV 开机就位（不移动，或移动到一个固定抓取点位）
- 工作台上放 **1 个已知工业件**（姿态基本固定）
- 机械臂执行：到观察位 → RealSense 拍一帧 → 感知出抓取位姿 → MoveIt 规划 → Pilz LIN 下压 → 夹爪闭合 → 抬起 → 放置到固定盒子

## 2. 前置准备

### 2.1 硬件
- [ ] AGV 已开机、地图载入、定位完成
- [ ] 机械臂已上电、EtherCAT 网卡插对（见 `src/inspection_bringup/config/arm_driver.yaml` 的 `elfin_ethernet_name`）
- [ ] RealSense D435 安装在末端（eye-in-hand），USB3.0 接 Jetson Orin
- [ ] 气动二指夹爪气管已接、电磁阀 DO 线接到工控机 DIO（确认 DO 线号：默认 DO0=闭合，DO1=松开）
- [ ] 工作台上放置标定过的已知工业件

### 2.2 标定（一次性工作，结果存 yaml）
- [ ] `base_link → arm_base`：机械臂安装静态标定 → `inspection_bringup/config/mount_calib.yaml`
- [ ] `tool0 → camera_link`：手眼标定（`easy_handeye2` + ChArUco）→ `inspection_bringup/config/handeye_calib.yaml`
- [ ] `tool0 → gripper_tip`：夹爪机械 offset（量一次写死）→ `inspection_bringup/config/gripper_offset.yaml`
- [ ] 验证：RViz 打开所有 TF，工作台位置/相机视野/夹爪末端的可视位置与实物对齐

### 2.3 感知模型
- [ ] 训练好的 YOLOv8 权重（`grasp_perception/models/yolov8n_parts.pt`）
- [ ] 每类工业件的默认抓取姿态（yaml 查表：`grasp_perception/config/class_grasp_poses.yaml`）

## 3. 启动流程（规划）

```bash
# Terminal 1 — 机械臂底层驱动（需要 sudo 跑 EtherCAT）
cd ~/huo_ws/inspection-robot
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch arm_driver arm_driver.launch.py"

# Terminal 2 — 完整系统（相机/控制/感知/夹爪/编排/网关）
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch inspection_bringup system.launch.py use_agv:=false rviz:=true
```

`system.launch.py` 预期拉起：
- `realsense_driver`（color + aligned_depth + camera_info + TF）
- `dio_driver`（DIO 底层）
- `gripper_driver`（open/close 语义）
- `arm_controller`（MoveIt2 + Pilz LIN plugin）
- `grasp_perception`（YOLOv8 + 深度 + TF 变换）
- `task_coordinator`（抓取状态机）
- `inspection_gateway`（:8080 REST/WS）

## 4. 触发任务

### 方式 A：ROS2 命令行
```bash
ros2 service call /inspection/start inspection_interface/srv/StartGrasp \
  "{task_name: 'demo_pick_screw', object_class: 'screw_m8', dry_run: false}"
```

### 方式 B：网关 REST
```bash
curl -X POST http://localhost:8080/api/v1/tasks \
  -H "Content-Type: application/json" \
  -d '{"task_name":"demo_pick_screw","object_class":"screw_m8"}'
```

### 方式 C：HMI 浏览器
打开 `http://<robot_ip>:8080/`，选择任务模板"demo_pick_screw"，点击"开始"。

## 5. 观察执行

```bash
# 状态
ros2 topic echo /inspection/state
# 感知候选（可视化）
ros2 topic echo /inspection/grasp/candidates
# 夹爪状态
ros2 topic echo /inspection/gripper/status
```

RViz：订阅 `/inspection/grasp/candidates`（PoseArray），能看到抓取候选箭头出现在工件上；随后机械臂运动到 pre-grasp → grasp。

## 6. 验收标准

P0 Demo 通过标准：

- [ ] 能启动：所有节点 `ros2 node list` 都在线，没有 `[ERROR]` 循环
- [ ] 能感知：调 `PerceiveGrasp` service 能返回至少 1 个合理候选（可达 + 在工件上）
- [ ] 能规划：MoveIt 对候选 `move_to_pose` 不报"IK failed" / "Planning failed"
- [ ] 能下压：Pilz LIN 在 pre-grasp → grasp 段是直线，不画弧
- [ ] 能夹爪：`gripper/close` 真的带动电磁阀，夹爪闭合（观察或听气阀声）
- [ ] 能抬起：夹爪闭合后 LIFT 段工件确实被带起来
- [ ] 能放置：PLACE 阶段能放到预设位置附近（精度不要求）
- [ ] 能收尾：HOME 后状态回到 IDLE，可再次触发

## 7. 安全注意

1. 启动前确保机械臂工作空间内**无人、无障碍、无线缆缠绕**
2. 第一次跑 `velocity_scaling=0.1`，确认无误再逐步提速
3. 急停按钮放手边
4. 气路确认：气泵压力在夹爪规定范围（通常 0.4–0.6 MPa）
5. 如果某一步动作明显不对，立刻 Ctrl+C 并按急停

## 8. 常见问题（占位）

待 P0 落地过程中积累问题，具体症状/原因/解决方案陆续补充到这里。

## 9. 相关文档

- 状态机与接口：`src/task_coordinator/CLAUDE.md`
- 感知模块：`src/grasp_perception/CLAUDE.md`（待创建）
- 夹爪：`src/gripper_driver/CLAUDE.md`（待创建）+ `src/dio_driver/CLAUDE.md`
- 手眼标定：`docs/HANDEYE_CALIB.md`（待创建）
- 原巡检 demo（已归档）：`demo操作指南.md`
