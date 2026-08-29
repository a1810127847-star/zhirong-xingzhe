#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash
set -u

required_nodes=(
  /amcl
  /collision_monitor
  /color_qr_detector
  /controller_server
  /gazebo
  /goal_guard
  /nav2_raw/bt_navigator
  /robot_state_publisher
  /task_manager
)
required_topics=(
  /camera/color/image_raw
  /cmd_vel_safe
  /goal_guard/status
  /scan
  /tasks/status
  /vision/detections
)

ready=false
for _ in $(seq 1 90); do
  nodes="$(ros2 node list 2>/dev/null || true)"
  topics="$(ros2 topic list 2>/dev/null || true)"
  actions="$(ros2 action list 2>/dev/null || true)"

  missing=false
  for node in "${required_nodes[@]}"; do
    if ! grep -qx "${node}" <<<"${nodes}"; then
      missing=true
    fi
  done
  for topic in "${required_topics[@]}"; do
    if ! grep -qx "${topic}" <<<"${topics}"; then
      missing=true
    fi
  done
  if ! grep -qx "/navigate_to_pose" <<<"${actions}"; then
    missing=true
  fi

  if [[ "${missing}" == "false" ]]; then
    ready=true
    break
  fi
  sleep 0.5
done

if [[ "${ready}" != "true" ]]; then
  echo "ERROR: Complete system did not become ready."
  echo "NODES=${nodes}"
  echo "TOPICS=${topics}"
  echo "ACTIONS=${actions}"
  exit 1
fi

collision_state=""
amcl_state=""
navigator_state=""
velocity_smoother_state=""
lifecycle_ready=false
for _ in $(seq 1 120); do
  collision_state="$(
    timeout 5s ros2 lifecycle get /collision_monitor 2>/dev/null || true
  )"
  amcl_state="$(
    timeout 5s ros2 lifecycle get /amcl 2>/dev/null || true
  )"
  navigator_state="$(
    timeout 5s ros2 lifecycle get \
      /nav2_raw/bt_navigator 2>/dev/null || true
  )"
  velocity_smoother_state="$(
    timeout 5s ros2 lifecycle get \
      /velocity_smoother 2>/dev/null || true
  )"
  if grep -q "active" <<<"${collision_state}" \
    && grep -q "active" <<<"${amcl_state}" \
    && grep -q "active" <<<"${navigator_state}" \
    && grep -q "active" <<<"${velocity_smoother_state}"; then
    lifecycle_ready=true
    break
  fi
  sleep 0.5
done
if [[ "${lifecycle_ready}" != "true" ]]; then
  echo "ERROR: Lifecycle nodes did not all become active."
  echo "COLLISION_MONITOR_STATE=${collision_state}"
  echo "AMCL_STATE=${amcl_state}"
  echo "BT_NAVIGATOR_STATE=${navigator_state}"
  echo "VELOCITY_SMOOTHER_STATE=${velocity_smoother_state}"
  exit 1
fi

if ! grep -q "active" <<<"${collision_state}"; then
  echo "ERROR: Collision Monitor is not active: ${collision_state}"
  exit 1
fi
if ! grep -q "active" <<<"${amcl_state}"; then
  echo "ERROR: AMCL is not active: ${amcl_state}"
  exit 1
fi
if ! grep -q "active" <<<"${navigator_state}"; then
  echo "ERROR: BT Navigator is not active: ${navigator_state}"
  exit 1
fi
if ! grep -q "active" <<<"${velocity_smoother_state}"; then
  echo "ERROR: Velocity Smoother is not active: ${velocity_smoother_state}"
  exit 1
fi

public_action_info="$(
  timeout 8s ros2 action info /navigate_to_pose
)"
raw_action_info="$(
  timeout 8s ros2 action info /nav2_raw/navigate_to_pose
)"
if ! grep -q "Action servers: 1" <<<"${public_action_info}" \
  || ! grep -q "/goal_guard" <<<"${public_action_info}"; then
  echo "ERROR: Public navigation action is not uniquely guarded."
  echo "${public_action_info}"
  exit 1
fi
if ! grep -q "Action servers: 1" <<<"${raw_action_info}" \
  || ! grep -q "/nav2_raw/bt_navigator" <<<"${raw_action_info}"; then
  echo "ERROR: Internal Nav2 action is not correctly isolated."
  echo "${raw_action_info}"
  exit 1
fi

topic_echo_with_retry() {
  local topic="$1"
  local output=""
  for _ in $(seq 1 3); do
    if output="$(timeout 12s ros2 topic echo --once "${topic}")"; then
      printf '%s\n' "${output}"
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: Timed out waiting for one message on ${topic}."
  return 1
}

topic_echo_with_retry /tasks/status
topic_echo_with_retry /vision/detections

model_names=""
model_list_ready=false
for _ in $(seq 1 3); do
  if model_names="$(
    timeout 12s ros2 service call \
      /get_model_list \
      gazebo_msgs/srv/GetModelList \
      '{}'
  )"; then
    model_list_ready=true
    break
  fi
  sleep 0.5
done
if [[ "${model_list_ready}" != "true" ]]; then
  echo "ERROR: Timed out calling Gazebo /get_model_list."
  exit 1
fi
if ! grep -q "dynamic_obstacle" <<<"${model_names}"; then
  echo "ERROR: dynamic_obstacle is missing from Gazebo."
  exit 1
fi
if ! grep -q "nav_home_qr_marker" <<<"${model_names}"; then
  echo "ERROR: nav_home_qr_marker is missing from Gazebo."
  exit 1
fi

echo "COLLISION_MONITOR_STATE=${collision_state}"
echo "AMCL_STATE=${amcl_state}"
echo "BT_NAVIGATOR_STATE=${navigator_state}"
echo "VELOCITY_SMOOTHER_STATE=${velocity_smoother_state}"
echo "FULL_SYSTEM_HEALTH_OK"
