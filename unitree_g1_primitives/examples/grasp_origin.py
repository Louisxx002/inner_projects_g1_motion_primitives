#!/usr/bin/env python3
"""
Real-robot autonomous template for G1 Primitives.

This script runs an autonomous, no-teleoperation pick-and-place sequence.
Use --dry-run to validate configuration and dependencies before sending commands.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
import traceback
import types
from multiprocessing import Array


import numpy as np
import pinocchio as pin

from pathlib import Path

# 把项目根目录加进去
ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR))

from g1_primitives import G1Primitives
from inspire_hand_dfx.dfx_action import InspireHandManager

# -----------------------------------------------------------------------------
# Fill this section for your setup
# -----------------------------------------------------------------------------
XR_TELEOP_REPO = Path(os.environ.get("XR_TELEOP_REPO", str(ROOT_DIR.parent / "xr_teleoperate")))
ROBOT_CONTROL_DIR = XR_TELEOP_REPO / "teleop" / "robot_control"

NETWORK_INTERFACE = "eno1"  # from ~/.cyclonedds.xml
DOMAIN_ID = 0  # real robot mode in xr_teleoperate defaults to domain 0

# Conservative initial wrist poses (meters, robot base frame)
INIT_LEFT_POS = np.array([0.25, 0.4, 0.30], dtype=float)
INIT_RIGHT_POS = np.array([0.25, -0.4, 0.30], dtype=float)

# Autonomous pick-and-place targets (left hand)
PICK_POS = np.array([0.30, 0.12, 0.05], dtype=float)
PLACE_POS = np.array([0.40, -0.10, 0.06], dtype=float)
APPROACH_OFFSET_Z = 0.15


class SimpleDex3Controller:
    """Minimal Dex3 joint-space publisher for non-teleop scripted control."""

    def __init__(self, left_hand_array: Array, right_hand_array: Array, fps: float = 100.0):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

        self.left_hand_array = left_hand_array
        self.right_hand_array = right_hand_array
        self.fps = fps
        self.running = True

        self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.left_pub.Init()
        self.right_pub.Init()

        self.left_msg = unitree_hg_msg_dds__HandCmd_()
        self.right_msg = unitree_hg_msg_dds__HandCmd_()
        self._init_messages()

        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()

    @staticmethod
    def _motor_mode(motor_id: int, status: int = 0x01, timeout: int = 0) -> int:
        return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)

    def _init_messages(self) -> None:
        left_ids = list(range(7))
        right_ids = list(range(7))
        for motor_id in left_ids:
            self.left_msg.motor_cmd[motor_id].mode = self._motor_mode(motor_id)
            self.left_msg.motor_cmd[motor_id].q = 0.0
            self.left_msg.motor_cmd[motor_id].dq = 0.0
            self.left_msg.motor_cmd[motor_id].tau = 0.0
            self.left_msg.motor_cmd[motor_id].kp = 1.5
            self.left_msg.motor_cmd[motor_id].kd = 0.2
        for motor_id in right_ids:
            self.right_msg.motor_cmd[motor_id].mode = self._motor_mode(motor_id)
            self.right_msg.motor_cmd[motor_id].q = 0.0
            self.right_msg.motor_cmd[motor_id].dq = 0.0
            self.right_msg.motor_cmd[motor_id].tau = 0.0
            self.right_msg.motor_cmd[motor_id].kp = 1.5
            self.right_msg.motor_cmd[motor_id].kd = 0.2

    def _publish_loop(self) -> None:
        period = 1.0 / self.fps
        while self.running:
            with self.left_hand_array.get_lock():
                left_q = np.array(self.left_hand_array[:], dtype=float)
            with self.right_hand_array.get_lock():
                right_q = np.array(self.right_hand_array[:], dtype=float)

            for idx in range(7):
                self.left_msg.motor_cmd[idx].q = float(left_q[idx])
                self.right_msg.motor_cmd[idx].q = float(right_q[idx])

            self.left_pub.Write(self.left_msg)
            self.right_pub.Write(self.right_msg)
            time.sleep(period)

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=1.0)


def _ensure_robot_control_on_path() -> None:
    if not ROBOT_CONTROL_DIR.exists():
        raise FileNotFoundError(
            f"robot_control path not found: {ROBOT_CONTROL_DIR}\n"
            "Clone unitree xr_teleoperate and update XR_TELEOP_REPO in this file."
        )
    sys.path.insert(0, str(ROBOT_CONTROL_DIR))


def _ensure_logging_mp_shim() -> None:
    if "logging_mp" in sys.modules:
        return
    try:
        import logging_mp  # noqa: F401
        return
    except Exception:
        pass

    shim = types.ModuleType("logging_mp")
    shim.getLogger = logging.getLogger
    shim.basicConfig = logging.basicConfig
    shim.INFO = logging.INFO
    shim.DEBUG = logging.DEBUG
    shim.WARNING = logging.WARNING
    shim.ERROR = logging.ERROR
    shim.CRITICAL = logging.CRITICAL
    sys.modules["logging_mp"] = shim


def _import_unitree_controllers():
    _ensure_robot_control_on_path()
    _ensure_logging_mp_shim()
    try:
        from robot_arm_ik import G1_29_ArmIK
        try:
            from robot_arm import G1_29_ArmController
        except Exception:
            from robot_arm_close import G1_29_ArmController
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from teleop.utils.motion_switcher import MotionSwitcher
    except Exception as exc:
        raise ImportError(
            "Failed to import Unitree controllers from teleop/robot_control. "
            f"Root cause: {type(exc).__name__}: {exc}\n"
            f"Traceback:\n{traceback.format_exc()}"
        ) from exc

    return G1_29_ArmController, G1_29_ArmIK, ChannelFactoryInitialize, MotionSwitcher


def validate_dry_run() -> None:
    _import_unitree_controllers()
    urdf_primary = XR_TELEOP_REPO / "assets" / "g1" / "g1_body29_hand14.urdf"
    model_dir_primary = XR_TELEOP_REPO / "assets" / "g1"
    urdf_legacy = XR_TELEOP_REPO / "teleop" / "assets" / "g1" / "g1_body29_hand14.urdf"
    model_dir_legacy = XR_TELEOP_REPO / "teleop" / "assets" / "g1"

    print(f"XR_TELEOP_REPO: {XR_TELEOP_REPO}")
    print(f"ROBOT_CONTROL_DIR: {ROBOT_CONTROL_DIR}")
    print(f"NETWORK_INTERFACE: {NETWORK_INTERFACE}")
    print(f"DOMAIN_ID: {DOMAIN_ID}")
    print(f"URDF exists (primary): {urdf_primary.exists()} -> {urdf_primary}")
    print(f"Model dir exists (primary): {model_dir_primary.exists()} -> {model_dir_primary}")
    print(f"URDF exists (legacy): {urdf_legacy.exists()} -> {urdf_legacy}")
    print(f"Model dir exists (legacy): {model_dir_legacy.exists()} -> {model_dir_legacy}")


def _parse_point_input(raw: str, default: np.ndarray) -> np.ndarray:
    text = raw.strip()
    if not text:
        return default.copy()
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("Expected exactly 3 values: x y z")
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)


def prompt_points(default_pick: np.ndarray, default_place: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    print("Enter target points before motion starts.")
    print(f"Pick default : {default_pick.tolist()}")
    print(f"Place default: {default_place.tolist()}")
    print("Press Enter to accept a default value.")

    while True:
        try:
            pick_raw = input("Pick point [x y z]: ")
            place_raw = input("Place point [x y z]: ")
            pick_pos = _parse_point_input(pick_raw, default_pick)
            place_pos = _parse_point_input(place_raw, default_place)
            return pick_pos, place_pos
        except ValueError as exc:
            print(f"Input error: {exc}")
            print("Please enter points like: 0.30 0.12 0.05")


def setup_real_robot() -> G1Primitives:
    G1_29_ArmController, G1_29_ArmIK, ChannelFactoryInitialize, MotionSwitcher = _import_unitree_controllers()

    # robot_arm_ik.py uses relative paths like "../assets/..."
    os.chdir(str(XR_TELEOP_REPO / "teleop"))

    left_hand_array = Array("d", 7)
    right_hand_array = Array("d", 7)

    ChannelFactoryInitialize(DOMAIN_ID, networkInterface=NETWORK_INTERFACE)
    try:
        motion_switcher = MotionSwitcher()
        status, result = motion_switcher.Enter_Debug_Mode()
        print(f"Enter debug mode status: {status}, result: {result}")
    except Exception as exc:
        print(f"[WARN] Failed to enter debug mode automatically: {exc}")
    # 
    try:
        arm_controller = G1_29_ArmController(motion_mode=False, simulation_mode=False)
    except TypeError:
        arm_controller = G1_29_ArmController(
            network_interface=NETWORK_INTERFACE,
            domain_id=DOMAIN_ID,
        )

    ik_solver = G1_29_ArmIK(Unit_Test=False, Visualization=False)

    hand_controller = SimpleDex3Controller(
        left_hand_array=left_hand_array,
        right_hand_array=right_hand_array,
    )

    wrist_positions = {
        "left": pin.SE3(pin.Quaternion(1, 0, 0, 0), INIT_LEFT_POS),
        "right": pin.SE3(pin.Quaternion(1, 0, 0, 0), INIT_RIGHT_POS),
    }
    last_hand_sol_tauff = np.zeros(14, dtype=float)
    hand_state = {"left": "open", "right": "open"}

    return G1Primitives(
        arm_controller=arm_controller,
        hand_controller=hand_controller,
        ik_solver=ik_solver,
        left_hand_array=left_hand_array,
        right_hand_array=right_hand_array,
        wrist_positions=wrist_positions,
        last_hand_sol_tauff=last_hand_sol_tauff,
        hand_state=hand_state,
    )


def run_autonomous_pick_and_place(
    primitives: G1Primitives,
    pick_pos: np.ndarray = PICK_POS,
    place_pos: np.ndarray = PLACE_POS,
    approach_offset_z: float = APPROACH_OFFSET_Z,
    speed_scale: float = 1.0,
    verbose: bool = True,
) -> None:
    pre_pick = pick_pos.copy()
    pre_pick[2] += approach_offset_z

    pre_place = place_pos.copy()
    pre_place[2] += approach_offset_z   

    hand = InspireHandManager()
    # hand.prepare_grasp()
    hand.release()
    # print("Step 1/9: ensure left hand open")
    # primitives.release(hand="left", duration=0.8 * speed_scale, verbose=verbose)

    print("Step 2/9: move above pick point")
    primitives.dual_arm_movement(
        left_hand_pos=pre_pick.tolist(),
        duration=3.0 * speed_scale,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=verbose,
    )

    print("Step 3/9: move down to pick")
    primitives.dual_arm_movement(
        left_hand_pos=pick_pos.tolist(),
        duration=2.5 * speed_scale,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=verbose,
    )

    hand.execute_grasp(strength=0.3)
    # time.sleep(1000)
    # print("Step 4/9: close left hand (grasp)")
    # primitives.grab(hand="left", duration=1.2 * speed_scale, verbose=verbose)

    # print("Step 5/9: lift object")
    # primitives.dual_arm_movement(
    #     left_hand_pos=pre_pick.tolist(),
    #     duration=2.5 * speed_scale,
    #     left_angle_deg=0.0,
    #     right_angle_deg=0.0,
    #     verbose=verbose,
    # )

    print("Step 6/9: move above place point")
    primitives.dual_arm_movement(
        left_hand_pos=pre_place.tolist(),
        duration=3.2 * speed_scale,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=verbose,
    )

    print("Step 7/9: move down to place")
    primitives.dual_arm_movement(
        left_hand_pos=place_pos.tolist(),
        duration=2.5 * speed_scale,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=verbose,
    )
    hand.release()
    
    # print("Step 8/9: open left hand (release)")
    # primitives.release(hand="left", duration=0.8 * speed_scale, verbose=verbose)

    # print("Step 9/9: retreat upward")
    # primitives.dual_arm_movement(
    #     left_hand_pos=pre_place.tolist(),
    #     duration=2.8 * speed_scale,
    #     left_angle_deg=0.0,
    #     right_angle_deg=0.0,
    #     verbose=verbose,
    # )


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 autonomous primitive demo")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only validate imports and configuration; do not send motion commands.",
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Use shorter durations (for simulation/bench only).",
    )
    parser.add_argument(
        "--pick",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=PICK_POS.tolist(),
        help=f"Pick point in meters, default: {PICK_POS.tolist()}",
    )
    parser.add_argument(
        "--place",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=PLACE_POS.tolist(),
        help=f"Place point in meters, default: {PLACE_POS.tolist()}",
    )
    parser.add_argument(
        "--approach-z",
        type=float,
        default=APPROACH_OFFSET_Z,
        help=f"Approach height above pick/place in meters, default: {APPROACH_OFFSET_Z}",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Do not prompt before motion; use --pick/--place values directly.",
    )
    args = parser.parse_args()

    print("Safety checklist (autonomous run):")
    print("1) Robot in safe workspace, no people in arm range")
    print("2) Hardware E-stop reachable")
    print("3) Low-level controllers already running")
    print("4) NETWORK_INTERFACE and DOMAIN_ID are correct")
    time.sleep(1.0)

    if args.dry_run:
        validate_dry_run()
        print("Dry-run mode complete.")
        return

    default_pick = np.array(args.pick, dtype=float)
    default_place = np.array(args.place, dtype=float)
    if args.no_prompt:
        pick_pos = default_pick
        place_pos = default_place
    else:
        pick_pos, place_pos = prompt_points(default_pick, default_place)

    print(f"Using pick point : {pick_pos.tolist()}")
    print(f"Using place point: {place_pos.tolist()}")

    primitives = setup_real_robot()
    print("Controller setup OK.")
    time.sleep(10)
    speed_scale = 0.65 if args.fast else 1.0
    approach_offset_z = 0.08 if args.fast else args.approach_z
    run_autonomous_pick_and_place(
        primitives,
        pick_pos=pick_pos,
        place_pos=place_pos,
        approach_offset_z=approach_offset_z,
        speed_scale=speed_scale,
        verbose=True,
    )
    print("Autonomous pick-and-place sequence complete.")


if __name__ == "__main__":
    main()
"""
简化代码
WJK
"""
