# inspection-robot 环境安装与编译记录

- 机器: `172.16.25.15` (`mic-733ao`)
- 日期: `2026-03-17`（原始记录）/ 后续按巡检方向维护
- 系统: `Ubuntu 22.04`
- 工作区: `~/inspection-robot`

> 注：本文档只涉及**环境安装与编译**。`inspection-robot` 当前方向为大型工件视觉检测/巡检，详见 `README.md`、`CLAUDE.md`。

## 1. 基础环境检查

已确认以下工具可用：

- ROS 2 Humble: `/opt/ros/humble`
- `colcon`
- `rosdep`
- `python3` / `cmake` / `gcc` / `g++`

## 2. rosdep 初始化问题与处理

### 2.1 问题

直接执行 `sudo rosdep init` 超时，原因是该机器无法访问：

- `https://raw.githubusercontent.com/ros/rosdistro/...`

导致 `rosdep init` / `rosdep update` 失败。

### 2.2 处理方式（切换 USTC 镜像）

手动写入 `/etc/ros/rosdep/sources.list.d/20-default.list` 为 USTC 源：

```bash
# os-specific listings first
yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/osx-homebrew.yaml osx

# generic
yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/base.yaml
yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/python.yaml
yaml https://mirrors.ustc.edu.cn/rosdistro/rosdep/ruby.yaml
gbpdistro https://mirrors.ustc.edu.cn/rosdistro/releases/fuerte.yaml fuerte
```

并在执行 rosdep 时设置：

```bash
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
```

执行成功：

```bash
sudo env ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml rosdep update
```

随后安装依赖：

```bash
cd ~/inspection-robot
source /opt/ros/humble/setup.bash
sudo env ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml \
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

结果：`#All required rosdeps installed successfully`

> 说明：过程里有 `ament_python` key 无法解析的提示（`inspection_gateway` / `inspection_sim`），但不影响最终构建通过。

## 3. 额外安装的关键依赖

在首次全量编译时缺少 `moveit_ros_planning_interface`，已安装：

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-moveit
```

rosdep 后续也自动安装了 `realsense2_camera`、`foxglove_bridge`、`image_transport_plugins` 等相关依赖。

### 3.1 巡检方向可选依赖

巡检场景按最终缺陷检测算法选择额外依赖：

```bash
# YOLOv8-seg 缺陷分割路线
pip install ultralytics opencv-python

# 局部点云 / 非平面工作距估计路线（可选）
pip install open3d
```

`dio_driver` 是历史通用 DIO 能力，当前巡检主线通常不需要。若单独调试该包，需要 `libgpiod`：

```bash
sudo apt-get install -y python3-libgpiod
# 非 root 运行需加入 gpio 组
sudo usermod -aG gpio $USER
```

## 4. 编译路径问题与解决

### 4.1 问题

在 `~/inspection-robot` 直接默认构建时，`inspection_interface` 出现路径截断报错（IDL 生成路径异常）。

### 4.2 解决方案

保持源码目录不变，仅将 `build/install/log` 指到纯英文路径：

```bash
cd ~/inspection-robot
source /opt/ros/humble/setup.bash
colcon --log-base ~/ir_log build --symlink-install \
  --build-base ~/ir_build \
  --install-base ~/ir_install
```

## 5. 最终编译结果

最后一次全量编译结果：

- `Summary: 17 packages finished [44.0s]`
- 仅有 warning（`defect_detector`、`elfin_ethercat_driver`），无失败包。

可直接使用：

```bash
source ~/ir_install/setup.bash
```

## 6. 一键复现实操命令（推荐）

```bash
# 0) 切到工作区
cd ~/inspection-robot

# 1) ROS 环境
source /opt/ros/humble/setup.bash

# 2) rosdep（使用 USTC rosdistro 镜像）
export ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
sudo env ROSDISTRO_INDEX_URL=$ROSDISTRO_INDEX_URL rosdep update
sudo env ROSDISTRO_INDEX_URL=$ROSDISTRO_INDEX_URL \
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# 3) 编译（build/install/log 用英文路径）
colcon --log-base ~/ir_log build --symlink-install \
  --build-base ~/ir_build \
  --install-base ~/ir_install

# 4) 生效环境
source ~/ir_install/setup.bash
```
