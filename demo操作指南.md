# 机械臂定点拍照 Demo 操作指南（巡检场景）

> **文档状态：巡检方向可用参考**
>
> 当前课题方向已改回巡检/视觉检测。本 demo 属于“8 点位机械臂预设位姿 + 工业相机拍照”的巡检演示，可作为论文演示流程的基础版本。
>
> 代码位置：
>
> - `src/task_coordinator/scripts/demo_inspection.py`
> - `src/inspection_bringup/scripts/start_demo.sh`
>
> 新开发工作请看：
>
> - 当前方向总纲：`README.md`、`CLAUDE.md`
> - 任务清单：`TODO.md`

---

## 一、前置说明

- **AGV 已禁用**：本次 demo 不会移动 AGV，只动机械臂 + 拍照。
- **网卡已切换**：`arm_driver` 的 EtherCAT 网卡已改为 `lan1`（如果你的实际网卡名不同，需再调整 `src/inspection_bringup/config/arm_driver.yaml` 里的 `elfin_ethernet_name`）。
- **速度设置**：机械臂速度缩放为 `0.1`（最大速度的 10%），安全平稳。

---

## 二、环境准备

### 2.1 检查环境变量
确保 `~/.bashrc` 里已加入 FastDDS 配置（只需配置一次，新终端自动生效）：

```bash
grep "FASTRTPS_DEFAULT_PROFILES_FILE" ~/.bashrc
```

如果看到输出，说明已配置好。如果还没加过，执行：

```bash
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=~/huo_ws/inspection-robot/src/inspection_bringup/config/fastdds_no_shm.xml' >> ~/.bashrc
```

然后**重新打开终端**或执行 `source ~/.bashrc` 生效。

### 2.2 修复日志权限（如之前用 sudo 跑过 ROS）
如果遇到过 `Permission denied: '/home/mic-733ao/.ros/log/...'`，执行一次：

```bash
sudo chown -R $(whoami):$(whoami) ~/.ros/log
```

---

## 三、启动流程（按顺序开 4 个终端）

每个终端都要先进入工作空间并 source 环境：

```bash
cd ~/huo_ws/inspection-robot
source /opt/ros/humble/setup.bash && source install/setup.bash
```

### 终端 1 — 机械臂底层驱动（arm_driver）
**需要 sudo，且必须先启动。**

```bash
cd ~/huo_ws/inspection-robot
sudo -E bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch arm_driver arm_driver.launch.py"
```

**预期输出**：成功初始化 EtherCAT，没有 `No slaves are found` 报错。

---

### 终端 2 — 机械臂控制 + MoveIt + RViz（arm_controller）
**等终端 1 正常后再启动。**

```bash
cd ~/huo_ws/inspection-robot
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch arm_controller arm_controller.launch.py rviz:=true
```

**预期输出**：RViz 窗口弹出，机械臂模型正常显示，没有红色报错。

---

### 终端 3 — 海康相机驱动（hikvision_driver）
**必须以触发模式启动，才能响应软触发拍照。**

```bash
cd ~/huo_ws/inspection-robot
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch hikvision_driver hikvision_driver.launch.py use_trigger_mode:=true
```

**预期输出**：相机成功打开，图像流正常发布。

> 可用 `ros2 topic hz /inspection/hikvision/image_raw` 检查图像流是否正常。

---

### 终端 4 — 运行 demo 脚本
**确认以上 3 个终端都正常运行后，再启动 demo。**

```bash
cd ~/huo_ws/inspection-robot
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run task_coordinator demo_inspection
```

---

## 四、Demo 执行流程

脚本会按以下顺序依次执行（共 8 个点）：

1. **Point_leftback**
2. **Point_leftfront**
3. **Point_left**
4. **Point_rightfront**
5. **Point_rightback**
6. **Point_right**
7. **Point_front**
8. **Point_top**

### 每个点位的动作：
1. 机械臂先回到 `home` 位（第 1 个点除外）
2. 机械臂运动到目标关节角度
3. 到达后**调用相机软触发拍照**
4. 停留 3 秒（等待曝光/落盘完成）
5. 进入下一个点

全部 8 个点拍完后，机械臂最后回一次 `home`。

---

## 五、安全注意事项

1. **启动 demo 前，确保机械臂工作空间内无障碍物、无缠绕线缆。**
2. **人员站在机械臂运动范围外。**
3. **急停按钮放在手边**，万一异常可立即拍下。
4. 如果某一步机械臂运动方向明显不对，立刻在终端 4 按 `Ctrl+C` 中断脚本，并按下急停。
5. 第一次跑建议速度保持 `0.1`，确认所有点位轨迹安全后再考虑提速。

---

## 六、常见问题

### Q1: `arm_driver` 报错 `No slaves are found on lan1`
说明网卡名不对。先用 `ip link show` 查看实际网卡名（如 `eno1`、`eth0`、`enp3s0`），然后修改：

```bash
# 编辑参数文件
nano src/inspection_bringup/config/arm_driver.yaml
```

把 `elfin_ethernet_name` 改成正确的网卡名，保存后重新编译：

```bash
colcon build --packages-select inspection_bringup
```

### Q2: 启动 `arm_controller` 时报 `Permission denied: ~/.ros/log/...`
执行：

```bash
sudo chown -R $(whoami):$(whoami) ~/.ros/log
```

### Q3: 相机没有触发 / 没存图
检查终端 3 是否以 `use_trigger_mode:=true` 启动，且 `ros2 service list | grep trigger_capture` 能看到服务。

---

## 七、文件位置备忘

| 文件 | 路径 |
|------|------|
| Demo 脚本 | `src/task_coordinator/scripts/demo_inspection.py` |
| 机械臂参数 | `src/inspection_bringup/config/arm_driver.yaml` |
| 环境变量 | `~/.bashrc` |
