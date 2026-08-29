#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

project_root="/mnt/c/Users/legend/Documents/暑期项目"
video_path="${project_root}/artifacts/demo/zhirong_complete_demo_final.mp4"

python3 "${project_root}/ros2_ws/tools/record_demo_video.py" \
  --output "${video_path}" \
  --duration 70 \
  --fps 15 &
recorder_pid=$!

cleanup() {
  if kill -0 "${recorder_pid}" 2>/dev/null; then
    kill -TERM "${recorder_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 "${project_root}/ros2_ws/tools/validate_task_patrol.py" \
  --command "patrol vision" \
  --expected-count 3 \
  --expected-names "blue_station,qr_station,home" \
  --final-x 0 \
  --final-y 0 \
  --timeout 150

wait "${recorder_pid}"
trap - EXIT
echo "FINAL_VISION_DEMO_OK=${video_path}"
