#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ARDUPILOT_DIR="${PROJECT_ROOT}/third_party/ardupilot"
GAZEBO_DIR="${PROJECT_ROOT}/third_party/ardupilot_gazebo"
PYTHON_BIN="${ARDUPILOT_DIR}/.venv/bin/python"
SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${PROJECT_ROOT}/logs/simulation/iris_smoke/${RUN_ID}"
SITL_STATE_DIR="${LOG_DIR}/sitl_state"
mkdir -p "${SITL_STATE_DIR}"

GAZEBO_PID=""
SITL_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${SITL_PID}" ]] && kill -0 "${SITL_PID}" 2>/dev/null; then
    kill -TERM -- "-${SITL_PID}" 2>/dev/null || true
  fi
  if [[ -n "${GAZEBO_PID}" ]] && kill -0 "${GAZEBO_PID}" 2>/dev/null; then
    kill -TERM -- "-${GAZEBO_PID}" 2>/dev/null || true
  fi
  wait "${SITL_PID}" 2>/dev/null || true
  wait "${GAZEBO_PID}" 2>/dev/null || true
  echo "Iris smoke-test artifacts: ${LOG_DIR}"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

for required in gz "${PYTHON_BIN}" "${SIM_VEHICLE}" \
  "${ARDUPILOT_DIR}/build/sitl/bin/arducopter" \
  "${GAZEBO_DIR}/build/libArduPilotPlugin.so"; do
  if [[ "${required}" == "gz" ]]; then
    command -v gz >/dev/null || { echo "Missing command: gz" >&2; exit 2; }
  elif [[ ! -e "${required}" ]]; then
    echo "Missing dependency: ${required}" >&2
    exit 2
  fi
done

export GZ_SIM_SYSTEM_PLUGIN_PATH="${GAZEBO_DIR}/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${GAZEBO_DIR}/models:${GAZEBO_DIR}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

setsid gz sim -s -r -v 4 iris_runway.sdf \
  >"${LOG_DIR}/gazebo.log" 2>&1 &
GAZEBO_PID=$!

sleep 2
if ! kill -0 "${GAZEBO_PID}" 2>/dev/null; then
  echo "Gazebo exited during startup" >&2
  exit 1
fi

(
  cd "${ARDUPILOT_DIR}"
  exec setsid "${PYTHON_BIN}" "${SIM_VEHICLE}" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --no-rebuild \
    --wipe-eeprom \
    --use-dir "${SITL_STATE_DIR}" \
    --add-param-file "${ARDUPILOT_DIR}/Tools/autotest/default_params/gazebo-iris.parm" \
    --no-mavproxy
) >"${LOG_DIR}/sitl.log" 2>&1 &
SITL_PID=$!

"${PYTHON_BIN}" "${SCRIPT_DIR}/iris_smoke_controller.py" \
  --result "${LOG_DIR}/result.json" \
  >"${LOG_DIR}/controller.log" 2>&1

echo "PASS: official Iris completed takeoff, hover, landing, and disarm"
