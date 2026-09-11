#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

command -v gz >/dev/null || { echo "Missing command: gz" >&2; exit 2; }
export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/simulation/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export SDF_PATH="${PROJECT_ROOT}/simulation/models${SDF_PATH:+:${SDF_PATH}}"
if [[ -d "${PROJECT_ROOT}/third_party/ardupilot_gazebo/build" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${PROJECT_ROOT}/third_party/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PROJECT_ROOT}/simulation/plugins/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_PARTITION="icarus_sdf_validation_$$"
validation_logs="$(mktemp -d "${PROJECT_ROOT}/logs/sdf_validation_XXXXXX")"

mapfile -d '' sdf_files < <(find "${PROJECT_ROOT}/simulation" -type f -name '*.sdf' -print0 | sort -z)
if (( ${#sdf_files[@]} == 0 )); then
  echo "No SDF files found" >&2
  exit 1
fi

for sdf_file in "${sdf_files[@]}"; do
  if [[ "${sdf_file}" == */worlds/* ]]; then
    validation_log="${validation_logs}/$(basename "${sdf_file}").log"
    timeout --kill-after=5s 25s gz sim -s -r --iterations 1 -v 2 "${sdf_file}" >"${validation_log}" 2>&1
    if rg -q '\[Err\]|Error Code|Failed to load|You tried to access' "${validation_log}"; then
      echo "Gazebo reported an error: ${validation_log}" >&2
      exit 1
    fi
  else
    gz sdf -k "${sdf_file}"
  fi
done

echo "Validated ${#sdf_files[@]} SDF file(s)"
