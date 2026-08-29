#!/usr/bin/env bash
set -euo pipefail

mode="check"
project_source=""
ros_distro="humble"

usage() {
  cat <<'EOF'
Usage:
  bootstrap_machine.sh --mode check
  bootstrap_machine.sh --mode system
  bootstrap_machine.sh --mode dependencies --source /path/to/ros2_ws/src

Modes:
  check          Read-only environment report.
  system         Install ROS 2 Humble base tools. Must run as root on Ubuntu 22.04.
  dependencies   Install package.xml dependencies with rosdep. Must run as root.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      mode="${2:-}"
      shift 2
      ;;
    --source)
      project_source="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: /etc/os-release is unavailable; Ubuntu is required." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release

print_check() {
  echo "OS=${PRETTY_NAME:-unknown}"
  echo "ARCH=$(dpkg --print-architecture 2>/dev/null || uname -m)"
  if [[ -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    echo "ROS2=OK"
    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/${ros_distro}/setup.bash"
    set -u
  else
    echo "ROS2=MISSING"
  fi
  command -v colcon >/dev/null && echo "COLCON=OK" || echo "COLCON=MISSING"
  command -v rosdep >/dev/null && echo "ROSDEP=OK" || echo "ROSDEP=MISSING"
  command -v gazebo >/dev/null && echo "GAZEBO=OK" || echo "GAZEBO=MISSING"
  command -v rviz2 >/dev/null && echo "RVIZ=OK" || echo "RVIZ=MISSING"
  echo "BOOTSTRAP_CHECK_OK"
}

require_root() {
  if [[ ${EUID} -ne 0 ]]; then
    echo "ERROR: mode '${mode}' must run as root." >&2
    exit 1
  fi
}

require_jammy_amd64() {
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "jammy" ]]; then
    echo "ERROR: this project is pinned to Ubuntu 22.04 (jammy); detected ${PRETTY_NAME:-unknown}." >&2
    exit 1
  fi
  if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
    echo "ERROR: Gazebo Classic on Ubuntu 22.04 is supported here only on amd64." >&2
    exit 1
  fi
}

install_system() {
  require_root
  require_jammy_amd64

  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y ca-certificates curl git locales software-properties-common
  locale-gen en_US en_US.UTF-8
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  add-apt-repository universe -y

  if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    ros_apt_version="$(
      curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' \
        | head -n 1
    )"
    if [[ -z "${ros_apt_version}" ]]; then
      echo "ERROR: unable to determine the latest ros2-apt-source release." >&2
      exit 1
    fi
    ros_apt_deb="/tmp/ros2-apt-source.deb"
    curl -fsSL -o "${ros_apt_deb}" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${VERSION_CODENAME}_all.deb"
    dpkg -i "${ros_apt_deb}"
  fi

  apt-get update
  apt-get install -y \
    "ros-${ros_distro}-desktop" \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-tk

  echo "SYSTEM_SETUP_OK"
}

install_dependencies() {
  require_root
  require_jammy_amd64
  if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
    echo "ERROR: ROS 2 Humble is missing; run --mode system first." >&2
    exit 1
  fi
  if [[ -z "${project_source}" || ! -d "${project_source}" ]]; then
    echo "ERROR: --source must point to an existing ros2_ws/src directory." >&2
    exit 1
  fi

  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${ros_distro}/setup.bash"
  set -u
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
  fi
  rosdep update
  rosdep install \
    --from-paths "${project_source}" \
    --ignore-src \
    --rosdistro "${ros_distro}" \
    -r -y

  echo "PROJECT_DEPENDENCIES_OK"
}

case "${mode}" in
  check)
    print_check
    ;;
  system)
    install_system
    ;;
  dependencies)
    install_dependencies
    ;;
  *)
    echo "ERROR: unsupported mode: ${mode}" >&2
    usage >&2
    exit 2
    ;;
esac
