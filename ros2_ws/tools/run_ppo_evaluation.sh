#!/usr/bin/env bash
set -e

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <model-path-relative-to-project-or-absolute> [evaluation options...]"
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"
model_path="$1"
shift

if [[ "${model_path}" != /* ]]; then
  model_path="${project_root}/${model_path}"
fi

cd "${project_root}"
exec "${script_dir}/ppo_python.sh" -m zhirong_ppo.evaluate \
  "${model_path}" \
  "$@"
