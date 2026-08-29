# 地图文件

该目录保存“智融行者”在 Gazebo 测试场景中生成的二维占据栅格地图。

- `zhirong_test_map.pgm`：地图像素数据
- `zhirong_test_map.yaml`：分辨率、原点和占用阈值

地图由 `slam_toolbox` 根据 `/scan`、`/odom` 和 TF 在线生成，再由
`nav2_map_server map_saver_cli` 保存。
