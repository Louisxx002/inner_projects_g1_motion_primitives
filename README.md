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

## Verification

Before upload, this repository copy passed:

```bash
./run_primitives_check.sh
```

See `RELEASE_VERIFICATION.md` and `UPLOAD_NOTES.md` for packaging notes.
