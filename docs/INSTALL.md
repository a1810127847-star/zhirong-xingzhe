# 安装与复现

## 已验证环境

- Windows 11 + WSL2
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10

项目在 WSL 中的标准构建目录为：

```text
~/zhirong_xingzhe_ws
```

Windows 仓库中的 ROS 包通过符号链接接入该工作空间，源码留在 Windows
目录，`build`、`install` 和 `log` 留在 Ubuntu 文件系统。

## 依赖

ROS 软件包：

```text
ros-humble-desktop
ros-humble-gazebo-ros-pkgs
ros-humble-slam-toolbox
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-nav2-collision-monitor
```

Ubuntu Python/系统软件包：

```text
python3-opencv
python3-numpy
python3-yaml
python3-pyzbar
libzbar0
python3-tk
```

`python3-pyzbar` 和 `libzbar0` 用于二维码解码。Ubuntu 22.04 自带的
OpenCV 4.5.4 提供 `QRCodeDetector` 接口，但本机版本没有链接 QUIRC，
不能单独完成二维码文本解码。

## 建立工作空间

进入 WSL：

```bash
wsl -d Ubuntu-22.04
```

运行仓库内的初始化脚本：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/setup_workspace.sh"
```

脚本只会创建缺失的包链接；如果目标已存在且不是正确链接，会停止并提示，
不会覆盖现有目录。

## 手动构建

```bash
source /opt/ros/humble/setup.bash
cd ~/zhirong_xingzhe_ws
colcon build --symlink-install
source install/setup.bash
```

应看到 5 个包构建成功：

```text
zhirong_description
zhirong_gazebo
zhirong_vision
zhirong_tasks
zhirong_bringup
```

## 图形桌面

本机使用 xRDP/XFCE，ROS 图形程序默认显示到 `:10`。登录 Ubuntu
图形桌面后运行：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_full_system_gui.sh"
```

若换到原生 Ubuntu 桌面，正常的 `DISPLAY` 和 `XAUTHORITY` 会被保留；
无需使用 Windows 终端操作 Gazebo 或 RViz。

## 复现校验

完整回归：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/full_regression_test.sh"
```

输出日志：

```text
artifacts/validation/full_regression_latest.log
```

该命令会全量构建、运行 8 个单元测试，并依次验证健康状态、多点巡航、
动态避障、急停/恢复、视觉返航和失败恢复。
