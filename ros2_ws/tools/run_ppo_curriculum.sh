#!/usr/bin/env bash
set -e

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <base-model> [empty-extra-steps] [single-steps]"
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
base_model="$1"
empty_steps="${2:-4096}"
single_steps="${3:-4096}"
stamp="$(date +%Y%m%d_%H%M%S)"
empty_run="${stamp}_empty_resume_seed42"
single_run="${stamp}_single_transfer_seed43"

if [[ "${base_model}" != /* ]]; then
  base_model="${project_root}/${base_model}"
fi

bash "${script_dir}/run_ppo_training.sh" \
  --resume-model "${base_model}" \
  --run-name "${empty_run}" \
  --total-timesteps "${empty_steps}" \
  --n-steps 256 \
  --batch-size 64 \
  --eval-episodes 5 \
  --seed 42 \
  --curriculum empty \
  --action-duration 0.10 \
  --max-steps 120

empty_model="${project_root}/artifacts/ppo/${empty_run}/ppo_zhirong.zip"

bash "${script_dir}/run_ppo_training.sh" \
  --resume-model "${empty_model}" \
  --run-name "${single_run}" \
  --total-timesteps "${single_steps}" \
  --n-steps 256 \
  --batch-size 64 \
  --eval-episodes 5 \
  --seed 43 \
  --curriculum single \
  --action-duration 0.10 \
  --max-steps 120

echo "PPO_CURRICULUM_EMPTY_RUN=${project_root}/artifacts/ppo/${empty_run}"
echo "PPO_CURRICULUM_SINGLE_RUN=${project_root}/artifacts/ppo/${single_run}"
echo "PPO_CURRICULUM_OK"
