#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_DIR="${PROJECT_DIR:-${G1_DIR}/unitree_g1_primitives}"
XR_TELEOP_REPO="${XR_TELEOP_REPO:-${G1_DIR}/xr_teleoperate}"
export XR_TELEOP_REPO

echo "== G1 Primitives Preflight Check =="
echo

echo "[1/10] Environment"
echo "PWD before check: $(pwd)"
echo "Python: $(command -v python || true)"
echo

echo "[2/10] Project paths"
test -d "$PROJECT_DIR" && echo "OK  project dir: $PROJECT_DIR" || echo "ERR project dir missing: $PROJECT_DIR"
test -d "$XR_TELEOP_REPO" && echo "OK  xr_teleoperate dir: $XR_TELEOP_REPO" || echo "ERR xr_teleoperate dir missing: $XR_TELEOP_REPO"
test -d "$XR_TELEOP_REPO/teleop/robot_control" && echo "OK  robot_control dir present" || echo "ERR robot_control dir missing"
test -f "$XR_TELEOP_REPO/assets/g1/g1_body29_hand14.urdf" && echo "OK  URDF present" || echo "ERR URDF missing"
echo

echo "[3/10] Core Python deps"
python - <<'PY'
mods = ["numpy", "scipy", "pinocchio"]
ok = True
for m in mods:
    try:
        __import__(m)
        print(f"OK  {m}")
    except Exception as e:
        ok = False
        print(f"ERR {m}: {type(e).__name__}: {e}")
raise SystemExit(0 if ok else 1)
PY
echo

echo "[4/10] XR/SDK deps"
python - <<'PY'
mods = ["casadi", "meshcat", "unitree_sdk2py"]
ok = True
for m in mods:
    try:
        __import__(m)
        print(f"OK  {m}")
    except Exception as e:
        ok = False
        print(f"ERR {m}: {type(e).__name__}: {e}")
raise SystemExit(0 if ok else 1)
PY
echo

echo "[5/10] g1_primitives import"
python - <<'PY'
from g1_primitives import G1Primitives, MotionType
print("OK  g1_primitives import")
print("OK  MotionType:", MotionType.GRAB)
PY
echo

echo "[6/10] xr_teleoperate controller imports"
python - <<'PY'
import sys
import os
sys.path.insert(0, os.environ.get("XR_TELEOP_REPO", "") + "/teleop/robot_control")
import robot_arm
import robot_arm_ik
print("OK  robot_arm import")
print("OK  robot_arm_ik import")
PY
echo

echo "[7/10] CycloneDDS and domain"
if [ -f "$HOME/.cyclonedds.xml" ]; then
  grep -n "NetworkInterfaceAddress" "$HOME/.cyclonedds.xml" || true
else
  echo "WARN ~/.cyclonedds.xml not found"
fi
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
echo

echo "[8/10] Network interfaces"
if command -v ip >/dev/null 2>&1; then
  ip -br addr || true
else
  echo "WARN ip command not found"
fi
echo

echo "[9/10] Script dry-run"
cd "$PROJECT_DIR" || exit 1
python examples/basic_usage.py --dry-run
echo

echo "[10/10] Manual safety checklist"
echo "1) Robot powered on"
echo "2) E-stop within reach"
echo "3) Workspace clear"
echo "4) Robot in controllable state"
echo "5) Input points verified safe"
echo
echo "Preflight check complete."
