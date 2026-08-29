#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
set -u

test_dir="$(mktemp -d /tmp/zhirong_task_patrol.XXXXXX)"
task_log="${test_dir}/task_manager.log"

setsid ros2 launch zhirong_tasks task_manager.launch.py \
  >"${task_log}" 2>&1 &
task_pid=$!

cleanup() {
  if kill -0 "${task_pid}" 2>/dev/null; then
    kill -INT -- "-${task_pid}" 2>/dev/null || true
  fi
  for _ in $(seq 1 40); do
    if ! kill -0 "${task_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "${task_pid}" 2>/dev/null; then
    kill -TERM -- "-${task_pid}" 2>/dev/null || true
  fi
  wait "${task_pid}" 2>/dev/null || true
}
trap cleanup EXIT

ready=false
for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx "/tasks/status"; then
    ready=true
    break
  fi
  sleep 0.5
done

if [[ "${ready}" != "true" ]]; then
  echo "ERROR: Task manager did not start."
  tail -n 160 "${task_log}"
  exit 1
fi

timeout 210s python3 \
  "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_task_patrol.py" \
  --timeout 180

if grep -Eq "Traceback|process has died" "${task_log}"; then
  echo "ERROR: Task manager reported a fatal exception."
  tail -n 160 "${task_log}"
  exit 1
fi

echo "TASK_PATROL_LIVE_TEST_OK"
echo "LOG_DIR=${test_dir}"
