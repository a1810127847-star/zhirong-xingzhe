# ROS 2 接口

## 传感器与底盘

| 接口 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2、键盘等控制源的原始速度 |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | 安全监控后的底盘实际输入 |
| `/odom` | `nav_msgs/Odometry` | 仿真里程计 |
| `/scan` | `sensor_msgs/LaserScan` | 360°、720 点、约 10 Hz |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 320×240 RGB 图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 32FC1 深度图 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 带 RGB 的点云 |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 坐标变换 |

## 视觉

| 接口 | 类型 | 说明 |
|---|---|---|
| `/vision/detections` | `std_msgs/String` | 每帧 JSON 检测结果 |
| `/vision/color` | `std_msgs/String` | 当前稳定颜色或 `none` |
| `/vision/qr` | `std_msgs/String` | 当前二维码文本或 `none` |
| `/vision/events` | `std_msgs/String` | 去抖后的颜色/QR 事件 JSON |
| `/vision/debug_image` | `sensor_msgs/Image` | 带框与标签的调试图像 |

视觉事件示例：

```json
{"type":"qr","value":"NAV:HOME","confidence":1.0,"stamp":40.2}
```

## 任务系统

| 接口 | 类型 | 说明 |
|---|---|---|
| `/tasks/command` | `std_msgs/String` | 文本或 JSON 任务命令 |
| `/tasks/status` | `std_msgs/String` | 状态、活动任务、队列、历史和错误 JSON |
| `/tasks/queue` | `std_msgs/String` | 当前待执行任务列表 JSON |
| `/tasks/start_patrol` | `std_srvs/Trigger` | 启动默认巡航 |
| `/tasks/return_home` | `std_srvs/Trigger` | 取消当前任务并优先返航 |
| `/tasks/cancel` | `std_srvs/Trigger` | 取消活动任务 |
| `/tasks/clear` | `std_srvs/Trigger` | 清空待执行队列 |
| `/tasks/arm_vision` | `std_srvs/SetBool` | 授权/撤销一次性视觉任务 |

文本命令：

```text
goto <station>
goto <x> <y> [yaw]
patrol [route]
return_home
cancel
clear
status
vision on
vision off
```

## Nav2 与安全

| 接口 | 类型 | 说明 |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Goal Guard 提供的公共安全单目标导航 |
| `/nav2_raw/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 内部 Nav2 动作，仅由 Goal Guard 调用 |
| `/goal_guard/status` | `std_msgs/String` | 最近一次目标接受、拒绝、成功或失败原因 JSON |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Nav2 原生航点接口 |
| `/plan` | `nav_msgs/Path` | 全局路径 |
| `/local_plan` | `nav_msgs/Path` | 局部轨迹 |
| `/safety/predicted_footprint` | `geometry_msgs/PolygonStamped` | 方向感知碰撞预测使用的车体轮廓 |

## Gazebo 测试接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/set_entity_state` | `gazebo_msgs/SetEntityState` | 测试时移动动态障碍 |
| `/model_states` | `gazebo_msgs/ModelStates` | 计算机器人—障碍实际距离 |
| `/get_model_list` | `gazebo_msgs/GetModelList` | 健康检查场景实体 |
