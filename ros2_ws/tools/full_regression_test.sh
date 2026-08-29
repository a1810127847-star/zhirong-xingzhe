#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

workspace="/home/lddcyfy/zhirong_xingzhe_ws"
project_root="/mnt/c/Users/legend/Documents/暑期项目"
report_dir="${project_root}/artifacts/validation"
report_file="${report_dir}/full_regression_latest.log"
runtime_dir="$(mktemp -d /tmp/zhirong_full_regression.XXXXXX)"
system_log="${runtime_dir}/system.log"

mkdir -p "${report_dir}"
exec > >(tee "${report_file}") 2>&1

echo "FULL_REGRESSION_STARTED=$(date --iso-8601=seconds)"
echo "RUNTIME_DIR=${runtime_dir}"

cd "${workspace}"
colcon build --symlink-install
source "${workspace}/install/setup.bash"
colcon test \
  --packages-select zhirong_bringup zhirong_tasks zhirong_vision \
  --event-handlers console_direct+
colcon test-result --verbose

setsid ros2 launch zhirong_bringup system.launch.py \
  gui:=false \
  navigation_rviz:=false \
  >"${system_log}" 2>&1 &
system_pid=$!

cleanup() {
  if kill -0 "${system_pid}" 2>/dev/null; then
    kill -INT -- "-${system_pid}" 2>/dev/null || true
  fi
  for _ in $(seq 1 60); do
    if ! kill -0 "${system_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "${system_pid}" 2>/dev/null; then
    kill -TERM -- "-${system_pid}" 2>/dev/null || true
  fi
  sleep 0.5
  kill -KILL -- "-${system_pid}" 2>/dev/null || true
  wait "${system_pid}" 2>/dev/null || true
}
trap cleanup EXIT

bash "${project_root}/ros2_ws/tools/full_system_health_check.sh"

timeout 210s python3 \
  "${project_root}/ros2_ws/tools/validate_task_patrol.py" \
  --command "patrol demo" \
  --expected-count 4 \
  --expected-names "east,northeast,north,home" \
  --final-x 0.0 \
  --final-y 0.0 \
  --timeout 180

timeout 180s python3 \
  "${project_root}/ros2_ws/tools/validate_dynamic_avoidance.py" \
  --timeout 130 \
  --obstacle-hold 8

timeout 60s python3 \
  "${project_root}/ros2_ws/tools/validate_collision_monitor.py"

timeout 180s python3 \
  "${project_root}/ros2_ws/tools/validate_vision_task_loop.py"

timeout 210s python3 \
  "${project_root}/ros2_ws/tools/validate_failure_recovery.py"

timeout 60s python3 \
  "${project_root}/ros2_ws/tools/validate_turn_localization.py"

bash "${project_root}/ros2_ws/tools/full_system_health_check.sh"

if grep -Eq \
  "process has died|Traceback|Failed to configure|Caught exception" \
  "${system_log}"; then
  echo "ERROR: The complete system log contains a fatal runtime failure."
  grep -E \
    "process has died|Traceback|Failed to configure|Caught exception" \
    "${system_log}"
  exit 1
fi

echo "FULL_REGRESSION_FINISHED=$(date --iso-8601=seconds)"
echo "FULL_REGRESSION_REPORT=${report_file}"
echo "FULL_REGRESSION_OK"
