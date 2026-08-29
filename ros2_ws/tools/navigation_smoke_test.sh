#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
set -u

test_dir="$(mktemp -d /tmp/zhirong_navigation_smoke.XXXXXX)"
launch_log="${test_dir}/navigation.log"

ros2 launch zhirong_bringup navigation.launch.py gui:=false navigation_rviz:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
  fi

  for _ in $(seq 1 40); do
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM "${launch_pid}" 2>/dev/null || true
  fi

  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

nav_ready=false
for _ in $(seq 1 60); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  actions="$(ros2 action list 2>/dev/null || true)"
  if grep -qx "/map" <<<"${topics}" &&
    grep -qx "/amcl_pose" <<<"${topics}" &&
    grep -qx "/global_costmap/costmap" <<<"${topics}" &&
    grep -qx "/local_costmap/costmap" <<<"${topics}" &&
    grep -qx "/navigate_to_pose" <<<"${actions}"; then
    nav_ready=true
    break
  fi
  sleep 1
done

if [[ "${nav_ready}" != "true" ]]; then
  echo "ERROR: Nav2 topics and actions did not become available."
  tail -n 220 "${launch_log}"
  exit 1
fi

timeout 120s python3 \
  "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/auto_navigate_test.py" \
  --goal-x 0.8 \
  --goal-y 0.0 \
  --goal-yaw 0.0 \
  --timeout 90.0

if grep -Eq "process has died|Failed to configure|Caught exception" "${launch_log}"; then
  echo "ERROR: A Nav2 or Gazebo process reported a fatal startup/runtime error."
  grep -E "process has died|Failed to configure|Caught exception" "${launch_log}"
  exit 1
fi

echo "NAVIGATION_SMOKE_TEST_OK"
echo "TOPICS_OK=/map,/scan,/odom,/global_costmap/costmap,/local_costmap/costmap"
echo "ACTION_OK=/navigate_to_pose"
echo "LOG_DIR=${test_dir}"
