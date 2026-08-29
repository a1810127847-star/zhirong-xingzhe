#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
set -u

test_dir="$(mktemp -d /tmp/zhirong_mapping_smoke.XXXXXX)"
launch_log="${test_dir}/mapping.log"
before_log="${test_dir}/map_before.log"
after_log="${test_dir}/map_after.log"
map_output="/mnt/c/Users/legend/Documents/暑期项目/maps/zhirong_test_map"

ros2 launch zhirong_bringup simulation.launch.py gui:=false slam:=true \
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

  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

topics_ready=false
for _ in $(seq 1 40); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -qx "/map" <<<"${topics}" &&
    grep -qx "/odom" <<<"${topics}" &&
    grep -qx "/scan" <<<"${topics}"; then
    topics_ready=true
    break
  fi
  sleep 1
done

if [[ "${topics_ready}" != "true" ]]; then
  echo "ERROR: /map, /odom, and /scan did not all become available."
  tail -n 160 "${launch_log}"
  exit 1
fi

python3 \
  "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_slam.py" \
  | tee "${before_log}"

publish_twist() {
  local linear_x="$1"
  local angular_z="$2"
  local message_count="$3"

  timeout 15s ros2 topic pub \
    --rate 10 \
    --times "${message_count}" \
    --wait-matching-subscriptions 1 \
    /cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: ${linear_x}}, angular: {z: ${angular_z}}}" \
    >/dev/null
}

stop_robot() {
  timeout 8s ros2 topic pub \
    --once \
    --wait-matching-subscriptions 1 \
    /cmd_vel \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" \
    >/dev/null
  sleep 0.5
}

for _ in $(seq 1 4); do
  publish_twist 0.22 0.0 15
  stop_robot
  publish_twist 0.0 0.75 20
  stop_robot
done

sleep 3

python3 \
  "/mnt/c/Users/legend/Documents/暑期项目/ros2_ws/tools/validate_slam.py" \
  --skip-tf \
  | tee "${after_log}"

before_known="$(sed -n 's/^MAP_KNOWN_CELLS=//p' "${before_log}")"
after_known="$(sed -n 's/^MAP_KNOWN_CELLS=//p' "${after_log}")"

python3 - "${before_known}" "${after_known}" <<'PY'
import sys

before = int(sys.argv[1])
after = int(sys.argv[2])

print(f"MAP_KNOWN_BEFORE={before}")
print(f"MAP_KNOWN_AFTER={after}")

if after < before:
    raise SystemExit("ERROR: Known map area shrank after the mapping route.")
PY

ros2 run nav2_map_server map_saver_cli \
  -f "${map_output}" \
  --ros-args \
  -p save_map_timeout:=10.0 \
  -p free_thresh_default:=0.25 \
  -p occupied_thresh_default:=0.65

for expected_file in "${map_output}.pgm" "${map_output}.yaml"; do
  if [[ ! -s "${expected_file}" ]]; then
    echo "ERROR: Map file was not saved: ${expected_file}"
    exit 1
  fi
done

if ! grep -Fq "Registering sensor: [Custom Described Lidar]" "${launch_log}"; then
  echo "ERROR: slam_toolbox did not register the lidar."
  tail -n 160 "${launch_log}"
  exit 1
fi

echo "MAPPING_SMOKE_TEST_OK"
echo "MAP_FILES=${map_output}.pgm,${map_output}.yaml"
echo "LOG_DIR=${test_dir}"
