#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_src="$(cd -- "${script_dir}/../src" && pwd)"
workspace="${ZHIRONG_WORKSPACE:-${HOME}/zhirong_xingzhe_ws}"

mkdir -p "${workspace}/src"

packages=(
  zhirong_description
  zhirong_gazebo
  zhirong_vision
  zhirong_tasks
  zhirong_bringup
  zhirong_ppo
)

for package in "${packages[@]}"; do
  source_path="${project_src}/${package}"
  target_path="${workspace}/src/${package}"

  if [[ -L "${target_path}" ]]; then
    existing="$(readlink -f "${target_path}")"
    expected="$(readlink -f "${source_path}")"
    if [[ "${existing}" != "${expected}" ]]; then
      echo "ERROR: ${target_path} points to ${existing}, expected ${expected}."
      exit 1
    fi
    continue
  fi

  if [[ -e "${target_path}" ]]; then
    echo "ERROR: ${target_path} already exists and is not a symbolic link."
    exit 1
  fi

  ln -s "${source_path}" "${target_path}"
done

cd "${workspace}"
colcon build --symlink-install

echo "WORKSPACE=${workspace}"
echo "SETUP_WORKSPACE_OK"
