#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
set -u

test_dir="$(mktemp -d /tmp/zhirong_smoke.XXXXXX)"
launch_log="${test_dir}/simulation.log"
tf_log="${test_dir}/tf.log"

ros2 launch zhirong_bringup simulation.launch.py gui:=false \
  >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
  fi

  for _ in $(seq 1 30); do
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM "${launch_pid}" 2>/dev/null || true
  fi

  for _ in $(seq 1 20); do
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -KILL "${launch_pid}" 2>/dev/null || true
  fi

  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sensor_topics=(
  /camera/color/camera_info
  /camera/color/image_raw
  /camera/depth/camera_info
  /camera/depth/image_raw
  /camera/depth/points
  /odom
  /scan
)

sensors_ready=false
for _ in $(seq 1 30); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  missing_topic=false
  for required_topic in "${sensor_topics[@]}"; do
    if ! grep -qx "${required_topic}" <<<"${topics}"; then
      missing_topic=true
      break
    fi
  done

  if [[ "${missing_topic}" == "false" ]]; then
    sensors_ready=true
    break
  fi
  sleep 1
done

if [[ "${sensors_ready}" != "true" ]]; then
  echo "ERROR: odometry, lidar, and RGB-D topics did not all become available."
  tail -n 120 "${launch_log}"
  exit 1
fi

python3 \
  "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_lidar.py"

python3 \
  "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_rgbd.py"

before_x="$(
  timeout 8s ros2 topic echo /odom --once --field pose.pose.position.x |
    sed -n "1p"
)"

timeout 12s ros2 topic pub \
  --rate 10 \
  --times 20 \
  --wait-matching-subscriptions 1 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.0}}"

timeout 8s ros2 topic pub \
  --once \
  --wait-matching-subscriptions 1 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

after_x="$(
  timeout 8s ros2 topic echo /odom --once --field pose.pose.position.x |
    sed -n "1p"
)"

python3 - "${before_x}" "${after_x}" <<'PY'
import sys

before = float(sys.argv[1])
after = float(sys.argv[2])
delta = after - before

print(f"ODOM_BEFORE_X={before:.4f}")
print(f"ODOM_AFTER_X={after:.4f}")
print(f"ODOM_DELTA_X={delta:.4f}")

if abs(delta) < 0.05:
    raise SystemExit("ERROR: Robot did not move far enough during the command test.")
PY

before_turn_z="$(
  timeout 8s ros2 topic echo /odom --once --field pose.pose.orientation.z |
    sed -n "1p"
)"

timeout 12s ros2 topic pub \
  --rate 10 \
  --times 20 \
  --wait-matching-subscriptions 1 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.8}}"

timeout 8s ros2 topic pub \
  --once \
  --wait-matching-subscriptions 1 \
  /cmd_vel \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

after_turn_z="$(
  timeout 8s ros2 topic echo /odom --once --field pose.pose.orientation.z |
    sed -n "1p"
)"

python3 - "${before_turn_z}" "${after_turn_z}" <<'PY'
import sys

before = float(sys.argv[1])
after = float(sys.argv[2])
delta = after - before

print(f"TURN_BEFORE_QZ={before:.4f}")
print(f"TURN_AFTER_QZ={after:.4f}")
print(f"TURN_DELTA_QZ={delta:.4f}")

if abs(delta) < 0.05:
    raise SystemExit("ERROR: Robot did not turn far enough during the steering test.")
PY

tf_status=0
timeout 6s ros2 run tf2_ros tf2_echo odom base_footprint \
  >"${tf_log}" 2>&1 || tf_status=$?

if ! grep -q "Translation:" "${tf_log}"; then
  echo "ERROR: odom -> base_footprint TF was not observed (status ${tf_status})."
  cat "${tf_log}"
  exit 1
fi

if ! grep -q "Successfully spawned entity" "${launch_log}"; then
  echo "ERROR: Gazebo did not report a successful robot spawn."
  tail -n 120 "${launch_log}"
  exit 1
fi

echo "SMOKE_TEST_OK"
echo "TOPICS_OK=/cmd_vel,/odom,/scan,/camera/color/image_raw,/camera/depth/image_raw,/camera/depth/points,/tf"
echo "LOG_DIR=${test_dir}"
