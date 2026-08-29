#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
set -u

probe_dir="$(mktemp -d /tmp/zhirong_vision_probe.XXXXXX)"
vision_log="${probe_dir}/vision.log"

ros2 launch zhirong_vision vision.launch.py >"${vision_log}" 2>&1 &
vision_pid=$!

cleanup() {
  if kill -0 "${vision_pid}" 2>/dev/null; then
    kill -INT "${vision_pid}" 2>/dev/null || true
  fi
  wait "${vision_pid}" 2>/dev/null || true
}
trap cleanup EXIT

camera_ready=false
for _ in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -qx "/camera/color/image_raw"; then
    camera_ready=true
    break
  fi
  sleep 0.5
done

if [[ "${camera_ready}" != "true" ]]; then
  echo "ERROR: /camera/color/image_raw is not available."
  exit 1
fi

timeout 20s ros2 topic echo --once /vision/detections
timeout 20s ros2 topic echo --once /vision/color

if grep -Eq "Traceback|Image processing failed" "${vision_log}"; then
  echo "ERROR: Vision node reported an exception."
  tail -n 120 "${vision_log}"
  exit 1
fi

echo "VISION_LIVE_PROBE_OK"
echo "LOG_DIR=${probe_dir}"
