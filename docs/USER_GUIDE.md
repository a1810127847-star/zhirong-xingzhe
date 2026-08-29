# 使用说明

## 一键启动

在 Ubuntu 图形桌面中运行：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_full_system_gui.sh"
```

它会启动：

- Gazebo
- RViz2
- AMCL + Nav2
- 颜色/二维码识别
- 任务队列与状态机
- Collision Monitor 安全速度过滤

系统启动后不会自动移动。视觉触发默认关闭。

## RViz 点到点导航

1. 等待地图、机器人、激光点和代价地图出现。
2. 点击顶部 `Nav2 Goal`。
3. 在白色可通行区域按住左键拖出方向箭头。
4. 机器人会规划、避障并在目标处停止。
5. 需要中止时使用 `Navigation 2` 面板中的 `Cancel`。

## 文本任务

另开一个 Ubuntu Terminal：

```bash
source /opt/ros/humble/setup.bash
source ~/zhirong_xingzhe_ws/install/setup.bash
```

多点巡航：

```bash
ros2 run zhirong_tasks task_cli patrol demo
```

前往命名站点：

```bash
ros2 run zhirong_tasks task_cli goto east
ros2 run zhirong_tasks task_cli goto qr_station
```

前往坐标：

```bash
ros2 run zhirong_tasks task_cli goto 0.8 0.5 1.57
```

优先返航：

```bash
ros2 run zhirong_tasks task_cli return_home
```

查看状态：

```bash
ros2 topic echo /tasks/status
```

## 视觉任务

授权一次视觉触发：

```bash
ros2 service call /tasks/arm_vision std_srvs/srv/SetBool "{data: true}"
```

也可使用：

```bash
ros2 run zhirong_tasks task_cli vision on
```

当前映射：

- 蓝色标志 → `blue_station`
- `NAV:HOME` 二维码 → 返航
- `NAV:PATROL` 二维码 → 默认巡航

第一个匹配事件触发后会自动撤权。下一次使用必须重新授权。

## 多点巡航路线

默认 `demo` 路线：

```text
east → northeast → north → home
```

巡航会先用 NavFn 一次生成连续路径，再由 `Regulated Pure Pursuit` 沿路径
通过各航点；不会在每个点停下并原地校正朝向。回到 `home` 前使用同向直线
交接，进入 `0.18 m` 范围后平滑停车。单点 `goto` 仍使用 DWB。

视觉演示路线：

```text
blue_station → qr_station → home
```

启动整条视觉演示路线：

```bash
ros2 run zhirong_tasks task_cli patrol vision
```

生成带状态、速度、定位和视觉框的 70 秒验收录像：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/run_final_vision_demo.sh"
```

站点和路线可在以下文件修改：

```text
ros2_ws/src/zhirong_tasks/config/tasks.yaml
```

## 手动键盘

松手即停键盘：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_keyboard_control_gui.sh"
```

按住 `W/S/A/D` 移动，松开即停，空格急停。键盘和 Nav2 不应同时控制，
避免两个控制源竞争。两者的速度都会经过 Collision Monitor。

## 目标与运动安全

在 RViz 点击目标后，Goal Guard 会先检查目标周围 `0.34 m`：

- 离障碍太近、位于未知区域或地图外：目标立即拒绝，小车不移动。
- 目标安全：转发给 Nav2 正常导航。
- 查看具体判定：`ros2 topic echo /goal_guard/status --once`。

运动过程中使用方向感知轨迹预测：

- 向障碍前进：自动减速直至停止。
- 原地转向扫掠范围安全：允许继续转向。
- 后方激光净空安全：允许低速后退脱困。
- 两段有限脱困仍失败：任务明确失败，不会无限尝试。

紧急取消任务：

```bash
ros2 service call /tasks/cancel std_srvs/srv/Trigger "{}"
```

清空待执行队列：

```bash
ros2 service call /tasks/clear std_srvs/srv/Trigger "{}"
```

## 停止系统

在标题为 `Zhirong Complete System` 的终端中按 `Ctrl+C`。Gazebo、RViz
和所有子节点会随 Launch 一起退出。

## PPO 实验模式

PPO 是独立研究模式。运行前先关闭标题为 `Zhirong Complete System` 的完整系统，
避免 Nav2 和 PPO 同时发布速度。

第一个 Ubuntu Terminal 启动无界面训练世界：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/start_ppo_world.sh"
```

第二个 Ubuntu Terminal 先做环境检查：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/run_ppo_smoke.sh" \
  --curriculum empty
```

训练：

```bash
bash "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/run_ppo_training.sh" \
  --total-timesteps 1024 \
  --seed 42 \
  --curriculum empty
```

单障碍课程把 `empty` 改为 `single`。训练产物位于
`artifacts/ppo/<运行编号>/`。停止时先结束训练命令，再在训练世界终端按
`Ctrl+C`；机器人会先发布零速度。
