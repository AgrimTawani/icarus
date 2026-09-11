#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
world="$project_root/simulation/worlds/mark4_rigid_body_drop.sdf"
world_name="mark4_rigid_body_drop"
topic="/world/$world_name/dynamic_pose/info"
run_id="$(date -u +%Y%m%dT%H%M%SZ)"
log_dir="$project_root/logs/simulation/mark4_drop_$run_id"
mkdir -p "$log_dir"

export GZ_SIM_RESOURCE_PATH="$project_root/simulation/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_PARTITION="icarus_drop_${run_id}_$$"

sim_pid=""
cleanup() {
  if [[ -n "$sim_pid" ]]; then
    for signal in INT TERM KILL; do
      kill -"$signal" -- "-$sim_pid" 2>/dev/null || break
      for _ in {1..20}; do
        kill -0 -- "-$sim_pid" 2>/dev/null || break
        sleep 0.1
      done
    done
    wait "$sim_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid gz sim -s -r -v 2 "$world" >"$log_dir/gazebo.log" 2>&1 &
sim_pid=$!

for _ in $(seq 1 40); do
  if gz topic -l 2>/dev/null | grep -Fxq "$topic"; then
    break
  fi
  sleep 0.25
done

if ! gz topic -l 2>/dev/null | grep -Fxq "$topic"; then
  echo "FAIL: Gazebo pose topic did not appear. See $log_dir/gazebo.log" >&2
  exit 1
fi

# Allow enough simulation time for the 194 mm drop and contact settling.
sleep 4
timeout 5 gz topic -e -n 1 -t "$topic" >"$log_dir/final_pose.txt"

python3 - "$log_dir/final_pose.txt" <<'PY'
import math
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()
match = re.search(
    r'name: "icarus_mark4_v2_10".*?position \{(.*?)\}.*?orientation \{(.*?)\}',
    text,
    re.S,
)
if not match:
    raise SystemExit("FAIL: Mark4 model pose was not present in Gazebo output")

def field(block: str, name: str, default: float = 0.0) -> float:
    found = re.search(rf"(?:^|\n)\s*{name}:\s*([-+0-9.eE]+)", block)
    return float(found.group(1)) if found else default

position, orientation = match.groups()
z = field(position, "z")
qx = field(orientation, "x")
qy = field(orientation, "y")
qz = field(orientation, "z")
qw = field(orientation, "w", 1.0)
tilt_deg = math.degrees(2.0 * math.acos(min(1.0, math.sqrt(qz*qz + qw*qw))))

if not 0.150 <= z <= 0.162:
    raise SystemExit(f"FAIL: unexpected settled base height {z:.6f} m")
if tilt_deg > 1.0:
    raise SystemExit(f"FAIL: vehicle settled with {tilt_deg:.3f} deg tilt")

print(f"PASS: settled base height={z:.6f} m, tilt={tilt_deg:.3f} deg")
PY

echo "Logs: $log_dir"
