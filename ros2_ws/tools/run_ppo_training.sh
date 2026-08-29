#!/usr/bin/env bash
set -e

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/../.." && pwd)"

cd "${project_root}"
exec "${script_dir}/ppo_python.sh" -m zhirong_ppo.train \
  --artifact-root "${project_root}/artifacts/ppo" \
  "$@"
