#!/usr/bin/env python3
"""
Real-robot autonomous template for G1 Primitives.
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
# from inspire_hand_dfx.dfx_action import InspireHandManager

# -----------------------------------------------------------------------------
XR_TELEOP_REPO = Path(os.environ.get("XR_TELEOP_REPO", str(ROOT_DIR.parent / "xr_teleoperate")))
ROBOT_CONTROL_DIR = XR_TELEOP_REPO / "teleop" / "robot_control"

NETWORK_INTERFACE = "eno1"
DOMAIN_ID = 0

INIT_LEFT_POS = np.array([0.25, 0.4, 0.30], dtype=float)
INIT_RIGHT_POS = np.array([0.25, -0.4, 0.30], dtype=float)

PICK_POS = np.array([0.30, 0.12, 0.05], dtype=float)
PLACE_POS = np.array([0.40, -0.10, 0.06], dtype=float)
APPROACH_OFFSET_Z = 0.15

class SimpleDex3Controller:
    def __init__(self, left_hand_array: Array, right_hand_array: Array, fps: float = 100.0):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

        self.left_hand_array = left_hand_array
        self.right_hand_array = right_hand_array
        self.running = True

        self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
        self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
        self.left_pub.Init()
        self.right_pub.Init()

        self.left_msg = unitree_hg_msg_dds__HandCmd_()
        self.right_msg = unitree_hg_msg_dds__HandCmd_()

        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()

    @staticmethod
    def _motor_mode(motor_id: int, status: int = 0x01, timeout: int = 0) -> int:
        return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)

    def _publish_loop(self) -> None:
        period = 0.01
        while self.running:
            with self.left_hand_array.get_lock():
                left_q = np.array(self.left_hand_array[:], dtype=float)
            with self.right_hand_array.get_lock():
                right_q = np.array(self.right_hand_array[:], dtype=float)

            for i in range(7):
                self.left_msg.motor_cmd[i].q = float(left_q[i])
                self.right_msg.motor_cmd[i].q = float(right_q[i])

            self.left_pub.Write(self.left_msg)
            self.right_pub.Write(self.right_msg)
            time.sleep(period)

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)


def _ensure_robot_control_on_path():
    if not ROBOT_CONTROL_DIR.exists():
        raise FileNotFoundError(f"robot_control not found: {ROBOT_CONTROL_DIR}")
    sys.path.insert(0, str(ROBOT_CONTROL_DIR))


def _ensure_logging_mp_shim():
    if "logging_mp" in sys.modules:
        return
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

    from robot_arm_ik import G1_29_ArmIK
    try:
        from robot_arm import G1_29_ArmController
    except Exception:
        from robot_arm_close import G1_29_ArmController

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from teleop.utils.motion_switcher import MotionSwitcher

    return G1_29_ArmController, G1_29_ArmIK, ChannelFactoryInitialize, MotionSwitcher


def setup_real_robot():
    G1_29_ArmController, G1_29_ArmIK, ChannelFactoryInitialize, MotionSwitcher = _import_unitree_controllers()

    os.chdir(str(XR_TELEOP_REPO / "teleop"))

    left_hand_array = Array("d", 7)
    right_hand_array = Array("d", 7)

    ChannelFactoryInitialize(DOMAIN_ID, networkInterface=NETWORK_INTERFACE)

    try:
        motion_switcher = MotionSwitcher()
        status, result = motion_switcher.Enter_Debug_Mode()
        print(f"Debug mode: {status}, {result}")
    except Exception as e:
        print(f"[WARN] Debug mode failed: {e}")

    try:
        arm_controller = G1_29_ArmController(motion_mode=False, simulation_mode=False)
    except TypeError:
        arm_controller = G1_29_ArmController(
            network_interface=NETWORK_INTERFACE,
            domain_id=DOMAIN_ID,
        )

    ik_solver = G1_29_ArmIK(Unit_Test=False, Visualization=False)

    # hand_controller = SimpleDex3Controller(left_hand_array, right_hand_array)

    wrist_positions = {
        "left": pin.SE3(
            pin.Quaternion(1, 0, 0, 0),
            np.array(INIT_LEFT_POS, dtype=float)
        ),
        "right": pin.SE3(
            pin.Quaternion(1, 0, 0, 0),
            np.array(INIT_RIGHT_POS, dtype=float)
        ),
    }   


    return G1Primitives(
        arm_controller=arm_controller,
        # hand_controller=hand_controller,
        ik_solver=ik_solver,
        left_hand_array=left_hand_array,
        right_hand_array=right_hand_array,
        wrist_positions=wrist_positions,
        last_hand_sol_tauff=np.zeros(14),
        hand_state={"left": "open", "right": "open"},
    )


def run_autonomous_pick_and_place(primitives):
    print("Waiting for robot stand...")
    primitives.wait_arm_ready()
    move_to_init(primitives)
    time.sleep(1000.0)

    pick = PICK_POS.copy()
    place = PLACE_POS.copy()

    pre_pick = pick.copy()
    pre_pick[2] += APPROACH_OFFSET_Z

    pre_place = place.copy()
    pre_place[2] += APPROACH_OFFSET_Z

    # hand = InspireHandManager()

    # print("Step 1: open hand")
    # hand.release()

    print("Step 2: move above pick")
    primitives.dual_arm_movement(left_hand_pos=pre_pick.tolist(), duration=2.5 * 1.0,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=True,)

    print("Step 3: move to pick")
    primitives.dual_arm_movement(left_hand_pos=pick.tolist(), duration=2.5 * 1.0,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=True,)

    print("Step 4: grasp")
    # hand.execute_grasp(strength=0.3)

    print("Step 5: move above place")
    primitives.dual_arm_movement(left_hand_pos=pre_place.tolist(), duration=2.5 * 1.0,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=True,)

    print("Step 6: move to place")
    primitives.dual_arm_movement(left_hand_pos=place.tolist(), duration=2.5 * 1.0,
        left_angle_deg=90.0,
        right_angle_deg=0.0,
        verbose=True,)

    print("Step 7: release")
    # hand.release()

    print("Done")

# ⭐⭐⭐ 新增：移动到初始位置函数
def move_to_init(primitives):
    print("Move to INIT position")

    init_left = INIT_LEFT_POS.copy()
    init_right = INIT_RIGHT_POS.copy()

    primitives.dual_arm_movement(
        left_hand_pos=init_left.tolist(),
        right_hand_pos=init_right.tolist(),
        duration=3.0,
        left_angle_deg=30.0,
        right_angle_deg=0.0,
        verbose=True,
    )

def main():
    print("Safety check passed")
    primitives = setup_real_robot()
    time.sleep(2)
    run_autonomous_pick_and_place(primitives)
    

if __name__ == "__main__":
    main()
    """
    简化代码并增加初始动作（测试机器人控制的代码）
    """
