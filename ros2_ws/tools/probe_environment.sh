#!/usr/bin/env bash

echo "PROCESSES"
ps -eo pid,cmd \
  | grep -E 'ros2 launch|rviz2|gzserver|gzclient|slam_toolbox|nav2' \
  | grep -v grep \
  || true

echo "PYTHON_AND_OPENCV"
source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash 2>/dev/null || true
python3 - <<'PY'
import cv2
import cv_bridge
import importlib.util
import rclpy

print(f"CV2_VERSION={cv2.__version__}")
print(f"QR_DETECTOR={hasattr(cv2, 'QRCodeDetector')}")
print(f"QR_ENCODER={hasattr(cv2, 'QRCodeEncoder_create')}")
print(f"PYTHON_QRCODE={importlib.util.find_spec('qrcode') is not None}")
print(f"PYZBAR={importlib.util.find_spec('pyzbar') is not None}")
print("ROS_PYTHON_OK")
PY

command -v qrencode || true
ldconfig -p 2>/dev/null | grep libzbar || true

echo "OPTIONAL_ROS_PACKAGES"
ros2 pkg list \
  | grep -E '^nav2_collision_monitor$|^vision_msgs$|^zbar_ros$' \
  || true
