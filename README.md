# inner_projects_g1_motion_primitives

This repository packages the Unitree G1 motion primitive workspace and the local dependencies needed by the examples.

## What It Does

```text
ROS2 / YOLO target input
        ↓
G1 primitives
        ↓
IK and arm controller
        ↓
Inspire hand / Unitree hand control
        ↓
Unitree G1 grasp, release, hold, and arm movement routines
```

Core capabilities:

- G1 arm and hand primitive actions: grab, release, hold position, and dual-arm movement.
- Example grasp flows using `unitree_g1_primitives/examples`.
- Bundled `xr_teleoperate` controller dependency used by the G1 arm/IK code.
- Bundled Inspire hand DFX controller code used by the local G1 configuration.
- YOLO helper scripts and smaller segmentation weights under `Reshape/`.

## Repository Layout

- `unitree_g1_primitives/`: editable Python package for the G1 primitive API and examples.
- `unitree_g1_primitives/preflight_check.sh`: local environment, dependency, controller import, and dry-run check.
- `xr_teleoperate/`: vendored controller and G1 asset dependency used by the primitive examples.
- `Reshape/`: YOLO/Open3D helper scripts and included segmentation weights.
- `run_primitives_check.sh`: top-level wrapper around the primitive preflight check.
- `run_ros_yolo_env.sh`: helper for launching the ROS/YOLO environment.

## Current Checks

`run_primitives_check.sh` runs the package preflight inside the `g1-primitives` conda environment by default:

```bash
cd /home/louisxx/inner_project_repos/inner_projects_g1_motion_primitives
./run_primitives_check.sh
```

The preflight checks:

```text
1. Project and xr_teleoperate paths
2. G1 URDF/assets availability
3. Core Python dependencies: numpy, scipy, pinocchio
4. XR/SDK dependencies: casadi, meshcat, unitree_sdk2py
5. g1_primitives package import
6. xr_teleoperate arm controller and IK imports
7. CycloneDDS/domain hints
8. Network interface visibility
9. examples/basic_usage.py --dry-run
10. Manual robot safety checklist
```

Useful overrides:

```bash
G1_PRIMITIVES_ENV=g1-primitives ./run_primitives_check.sh
PROJECT_DIR=/path/to/unitree_g1_primitives XR_TELEOP_REPO=/path/to/xr_teleoperate \
  ./unitree_g1_primitives/preflight_check.sh
```

## Verification

Before upload, this repository copy passed:

```bash
./run_primitives_check.sh
```

See `RELEASE_VERIFICATION.md` and `UPLOAD_NOTES.md` for packaging notes.
