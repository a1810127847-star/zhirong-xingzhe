# 智融行者 ROS2 仿真系统

本仓库是“智融行者”移动机器人仿真项目的代码仓库。当前阶段使用：

- Ubuntu 22.04（WSL2）
- ROS2 Humble
- Gazebo Classic 11
- 四轮滑移转向底盘

## 当前功能

- `zhirong_description`：四轮滑移转向底盘 URDF/Xacro 模型
- `zhirong_gazebo`：Gazebo 测试场景
- `zhirong_bringup`：仿真一键启动入口
- `zhirong_vision`：颜色与二维码视觉事件
- `zhirong_tasks`：任务队列、巡航、返航与失败恢复
- `zhirong_ppo`：Gymnasium 环境、PPO 训练与评估工具
- 车顶二维激光雷达：360°、10 Hz、720 点、0.12～12 m
- 前置 RGB-D 相机：彩色图、深度图、相机参数和带 RGB 点云
- RViz2：同时显示 RobotModel、TF、Odometry、LaserScan 和 RGB-D 点云
- `slam_toolbox`：使用二维雷达在线生成占据栅格地图
- Nav2：AMCL 定位、路径规划、局部控制和代价地图避障
- `/cmd_vel` 速度控制
- `/odom` 里程计
- `/scan` 激光扫描
- `/map` 二维地图
- `/camera/color/image_raw` 彩色图像
- `/camera/depth/image_raw` 深度图像
- `/camera/depth/points` 三维点云
- `odom -> base_footprint` TF

## Windows 裸机一键部署

给验收人员发送 `智融行者验收一键部署.zip`。对方完整解压后，只需双击：

```text
一键部署并打开验收.cmd
```

入口会自动申请管理员权限并依次完成：

1. 检查 64 位 Windows 版本和 WinGet。
2. 安装缺失的 Windows Git 与带 Tk 的 Python 3。
3. 安装 WSL2 与 Ubuntu 22.04；需要重启时登记一次性续装任务，登录后自动继续。
4. 从公开 GitHub 仓库 Clone `master` 分支；已有干净仓库只执行 `fast-forward` 更新。
5. 在 Ubuntu 中安装 ROS2 Humble、Gazebo、Nav2、SLAM 和 `package.xml` 依赖。
6. 建立 `$HOME/zhirong_xingzhe_ws` 并执行 `colcon build --symlink-install`。
7. 运行验收面板自检，打开面板并自动启动 Gazebo、RViz 和 Nav2。

默认 Windows 源码目录为 `文档\ZhirongXingzhe`，部署日志位于
`%LOCALAPPDATA%\ZhirongXingzhe\bootstrap.log`。安装过程可安全重跑；如果目标
仓库存在未提交修改，安装器会停止而不会覆盖。

可在项目根目录执行以下命令重新生成发送包：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\ros2_ws\tools\build_windows_deployment_bundle.ps1
```

部署完成后，日常重新打开只需双击 `打开项目验收面板.cmd`。右上角“跨机联调配置”
仍保留检测、Clone/更新、依赖安装、构建和 GitHub 发布的分步入口，供故障排查使用。

系统安装脚本 `ros2_ws/tools/bootstrap_machine.sh` 只接受 Ubuntu 22.04 amd64，
避免在错误系统上修改软件源。完整收件人说明见 `给验收人员的部署说明.txt`。

## 工作空间

Ubuntu 中的构建工作空间：

```text
$HOME/zhirong_xingzhe_ws
```

仓库中的六个功能包会链接到该工作空间的 `src` 目录，代码保留在 Windows Git
仓库中，构建产物保留在 Ubuntu 文件系统中。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/zhirong_xingzhe_ws
colcon build --symlink-install
source install/setup.bash
```

## 启动仿真

同时启动 Gazebo 和 RViz2：

```bash
ros2 launch zhirong_bringup simulation.launch.py rviz:=true
```

也可以在 WSL 中执行图形化一键启动脚本：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_simulation_gui.sh"
```

只启动 Gazebo：

```bash
ros2 launch zhirong_bringup simulation.launch.py
```

无图形界面启动：

```bash
ros2 launch zhirong_bringup simulation.launch.py gui:=false
```

## 二维激光雷达

雷达固定在车体中心上方 `0.145 m`，发布到 `/scan`。当前参数：

- 扫描范围：360°
- 扫描频率：10 Hz
- 每圈采样：720 点
- 测距范围：0.12～12 m
- 坐标系：`lidar_link`

打开带 RViz2 的仿真后，RViz 中的红色点就是雷达看到的障碍物。固定坐标系为
`odom`，机器人移动时可以同时观察车体、TF、里程计和扫描点。

仿真运行时可执行以下检查：

```bash
python3 "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_lidar.py"
```

## RGB-D 相机

相机安装在车体前部，朝机器人正前方。为了避免挡住激光雷达，相机外壳位于雷达扫描平面下方。
当前仿真参数：

- 分辨率：320×240
- 图像频率：15 Hz
- 水平视场角：70°
- 深度范围：0.15～8 m
- 光学坐标系：`camera_optical_link`

主要话题：

- `/camera/color/image_raw`：彩色图
- `/camera/color/camera_info`：彩色相机参数
- `/camera/depth/image_raw`：32 位浮点深度图
- `/camera/depth/camera_info`：深度相机参数
- `/camera/depth/points`：包含 RGB 颜色的 `PointCloud2`

RViz2 默认显示三维点云；显示列表中的 `RGB Color Image` 已配置但默认关闭，需要单独看
彩色画面时勾选它即可。仿真运行时可执行以下完整相机检查：

```bash
python3 "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_rgbd.py"
```

## SLAM 建图

启动 Gazebo、`slam_toolbox`、建图版 RViz2 和机器人模型：

```bash
ros2 launch zhirong_bringup mapping.launch.py
```

也可以直接使用图形化启动脚本：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_mapping_gui.sh"
```

随后打开松手即停键盘窗口，按住 `W/S/A/D` 驾驶机器人扫描环境。RViz2 使用俯视视角：

- 白色：已确认可以通行
- 黑色：墙体或障碍物
- 灰色：尚未扫描到
- 红点：当前二维雷达扫描

当前测试场景增加了四面围墙，便于观察地图边界和验证闭环。主要 SLAM 输出：

- `/map`：完整占据栅格地图
- `/map_updates`：增量地图更新
- `map -> odom`：SLAM 修正后的地图坐标关系

运行中的严格检查：

```bash
python3 "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_slam.py"
```

保存当前地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "/mnt/c/Users/legend/Documents/暑期项目/maps/zhirong_test_map"
```

已生成的地图位于：

- `maps/zhirong_test_map.pgm`
- `maps/zhirong_test_map.yaml`
- `maps/zhirong_test_map_preview.png`

## Nav2 自主导航

启动 Gazebo、地图服务器、AMCL、Nav2 和专用 RViz2 界面：

```bash
ros2 launch zhirong_bringup navigation.launch.py
```

也可以直接使用图形化启动脚本：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_navigation_gui.sh"
```

导航模式的操作：

1. 等待 RViz2 中地图、机器人和代价地图出现。
2. 点击顶部工具栏的 `Nav2 Goal`。
3. 在白色可通行区域按住鼠标左键并拖动一个箭头：
   - 箭头起点是目标位置。
   - 箭头方向是机器人到达后的朝向。
4. 机器人会自动规划路线、避开地图和雷达中的障碍物，并在目标点停车。
5. 需要中途停止时，点击右侧 `Navigation 2` 面板中的 `Cancel`。

AMCL 默认把仿真出生点设为地图 `(0, 0, 0)`。如果手动移动了机器人模型或定位明显偏移，
使用 RViz2 的 `2D Pose Estimate` 重新指定位置。

导航模式不要同时打开松手即停键盘窗口，因为键盘控制器和 Nav2 都会发布 `/cmd_vel`。

完整自动测试：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/navigation_smoke_test.sh"
```

关键输出：

- `/amcl_pose`：机器人在地图中的定位
- `/global_costmap/costmap`：全局规划代价地图
- `/local_costmap/costmap`：实时避障代价地图
- `/plan`：全局规划路线
- `/local_plan`：局部控制路线
- `/navigate_to_pose`：目标点导航动作

## 键盘控制机器人

保持仿真运行，再打开第二个 Ubuntu Terminal：

```bash
source /opt/ros/humble/setup.bash
source ~/zhirong_xingzhe_ws/install/setup.bash
ros2 run zhirong_bringup keyboard_hold_teleop.py
```

也可以直接打开专用的键盘控制窗口：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_keyboard_control_gui.sh"
```

按键前，先用鼠标点一下标题为 `Zhirong Hold-to-Run Control` 的窗口，
让键盘输入落在这个窗口里。

控制键：

- 按住 `W`：前进；松开立即停车
- 按住 `S`：倒车；松开立即停车
- 按住 `A`：向左转；松开停止转向
- 按住 `D`：向右转；松开停止转向
- 可以同时按 `W+A`、`W+D`、`S+A` 或 `S+D`
- `空格`：立即清除全部按键状态并停车
- 控制窗口失去焦点或关闭时自动停车

控制窗口有三个速度滑条，可设置前进、倒车和转向速度。默认值是：

- 前进：`0.35 m/s`
- 倒车：`0.25 m/s`
- 转向：`1.00 rad/s`

控制器最终发布到 `/cmd_vel`：

- `linear.x` 是油门，正数前进、负数倒车
- `angular.z` 是方向，正数左转、负数右转

## 手柄控制准备

该功能目前按用户要求暂停，当前运行控制器仍是松手即停键盘窗口。

当前系统已经安装：

- `joy`
- `teleop_twist_joy`

但 WSL 当前还没有 `/dev/input`，说明 USB 手柄尚未接入 Linux。
手柄连接到 WSL 后，可以先使用 Xbox 标准映射：

```bash
source /opt/ros/humble/setup.bash
source ~/zhirong_xingzhe_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox
```

默认 Xbox 映射使用左摇杆控制前后和左右；必须按住左扳机才允许输出，
右扳机为高速模式。实际轴号和按键号需要在手柄接入后根据型号验证。

## 直接发送控制指令

直行：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

左转：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.8}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

查看里程计：

```bash
ros2 topic echo /odom
```

## 完整系统（2026-07-29）

计划书定义的一级和二级仿真功能已经集成到统一入口：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_full_system_gui.sh"
```

新增功能：

- OpenCV 颜色识别和 pyzbar 二维码解码
- `NAV:HOME` 视觉触发返航
- 文本任务、任务队列、状态机、重试和失败恢复
- 多点巡航与一键返航
- Gazebo 动态障碍与 Nav2 实时绕行
- 随机远终点、行驶后随机生成 1–4 个障碍物的 3 轮鲁棒验收，支持 seed 重放
- 动态避障采用 `SmacPlanner2D` 平滑全局路径、独立弧线控制器与 3 Hz 重规划，自动判废持续原地打转
- 目标安全校验、方向感知 Collision Monitor 和 `/cmd_vel_safe`
- AMCL 转向稳定化与 Nav2 角速度/角加速度限幅
- 一次性视觉授权，防止连续误触发；二维码返航任务在到达识别站并停车后才授权
- 一键全回归与验收日志

完整回归：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/full_regression_test.sh"
```

转向定位专项测量（Gazebo 真值、`/odom`、AMCL 三路对照）：

```bash
python3 "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_turn_localization.py"
```

详细文档：

- `docs/INSTALL.md`
- `docs/ARCHITECTURE.md`
- `docs/INTERFACES.md`
- `docs/USER_GUIDE.md`
- `docs/PPO_EXPERIMENT_PLAN.md`
- `COMPLETION_CHECKLIST.md`

## PPO 研究扩展

PPO 使用独立无界面训练世界，不能与完整 Nav2 系统同时启动。先启动训练世界：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_ppo_world.sh"
```

另开 Ubuntu Terminal，检查空场环境：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/run_ppo_smoke.sh"
```

开始一轮可复现训练：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/run_ppo_training.sh" \
  --total-timesteps 1024 \
  --seed 42 \
  --curriculum empty
```

模型和曲线保存在 `artifacts/ppo/<运行编号>/`。PPO 直接发布 `/cmd_vel`，
但最终动作仍必须经过 Collision Monitor 的 `/cmd_vel_safe` 才能到达底盘。
训练默认每 `256` 环境步保存恢复检查点。当前 Reward V4 固定扇形正式结果为
`24/30`，随机空场最佳检查点为 `21/30`，两者均为 `0` 碰撞；随机空场仍未
达到 `80%` 晋级线，因此不进入单障碍正式训练，也不替换 Nav2 DWB。完整方案
见 `docs/PPO_EXPERIMENT_PLAN.md`，课程训练与固定 30 回合结果见
`docs/PPO_TRAINING_REPORT_2026-08-15.md`。

最终验收产物：

- 完整回归日志：`artifacts/validation/full_regression_latest.log`
- 演示视频：`artifacts/demo/zhirong_complete_demo_final.mp4`
- 最终视觉巡检：`blue_station → qr_station → home`
- 实测用时 `50.837 s`，返航误差 `0.050 m`
