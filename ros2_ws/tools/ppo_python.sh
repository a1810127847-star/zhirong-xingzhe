#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/lddcyfy/zhirong_xingzhe_ws/install/setup.bash

exec /home/lddcyfy/zhirong_ppo_venv/bin/python "$@"
