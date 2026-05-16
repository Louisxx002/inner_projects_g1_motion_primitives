#!/usr/bin/env python3
"""
Real-robot autonomous template for G1 Primitives.
Now supports ROS2 /grasp topic input.
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

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# -----------------------------------------------------------------------------
# 项目路径
ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(ROOT_DIR))

from g1_primitives import G1Primitives
from inspire_hand_dfx.dfx_action import InspireHandManager

# -----------------------------------------------------------------------------
XR_TELEOP_REPO = Path(os.environ.get("XR_TELEOP_REPO", str(ROOT_DIR.parent / "xr_teleoperate")))
ROBOT_CONTROL_DIR = XR_TELEOP_REPO / "teleop" / "robot_control"

NETWORK_INTERFACE = "eno1"
DOMAIN_ID = 0

INIT_LEFT_POS = np.array([0.25, 0.4, 0.050], dtype=float)
INIT_RIGHT_POS = np.array([0.25, -0.4, 0.05], dtype=float)

PLACE_POS = np.array([0.20, 0.4, 0.03], dtype=float)
APPROACH_OFFSET_Z = 0.2
APPROACH_OFFSET_Y = 0.1

# =============================================================================
# ✅ ROS2 订阅 grasp 点
# =============================================================================


class GraspSubscriber(Node):
    def __init__(self):
        super().__init__("grasp_subscriber")
        self.sub = self.create_subscription(
            PointStamped,
            "/grasp",
            self.callback,
            10
        )
        self.grasp_point = None

    def callback(self, msg: PointStamped):
        self.grasp_point = np.array([
            msg.point.x,
            msg.point.y,
            0.0003 #人为写死Z轴高度，根据实际桌子高度修改，ik精度不够
        ], dtype=float)

        self.get_logger().info(
            f"Received grasp point: {self.grasp_point}"
        )


def get_grasp_point():
    rclpy.init()
    node = GraspSubscriber()

    print("Waiting for /grasp topic...")
    grasp = None

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.grasp_point is not None:
            grasp = node.grasp_point.copy()
            break

    node.destroy_node()
    rclpy.shutdown()
    return grasp


# =============================================================================
# 原有控制代码（保持不变）
# =============================================================================
class DummyDex3Controller:
    """
    Fake hand controller for simulation / debugging.
    Keeps API compatible but does nothing.
    """

    def __init__(self, left_hand_array=None, right_hand_array=None, fps: float = 100.0):
        self.left_hand_array = left_hand_array
        self.right_hand_array = right_hand_array
        print("[DummyDex3Controller] Initialized (no real hardware control)")

    def close(self):
        print("[DummyDex3Controller] Closed")

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
        motion_switcher.Enter_Debug_Mode()
    except Exception as e:
        print(f"[WARN] Debug mode failed: {e}")

    arm_controller = G1_29_ArmController(motion_mode=False, simulation_mode=False)
    ik_solver = G1_29_ArmIK(Unit_Test=False, Visualization=False)
    # hand_controller = DummyDex3Controller(left_hand_array, right_hand_array)

    wrist_positions = {
        "left": pin.SE3(pin.Quaternion(1, 0, 0, 0), INIT_LEFT_POS),
        "right": pin.SE3(pin.Quaternion(1, 0, 0, 0), INIT_RIGHT_POS),
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


# =============================================================================
# 主逻辑
# =============================================================================
def move_to_init(primitives):
    primitives.dual_arm_movement(
        left_hand_pos=INIT_LEFT_POS.tolist(),
        right_hand_pos=INIT_RIGHT_POS.tolist(),
        duration=3.0,
        left_angle_deg=0.0,
        right_angle_deg=0.0,verbose=True,
    )


def run_autonomous_pick_and_place(primitives):
    print("Waiting for robot stand...")
    primitives.wait_arm_ready()
    move_to_init(primitives)
    time.sleep(2.0) 
    hand = InspireHandManager()#DFX灵巧手代码，型号不同需要重新写代码
    # hand.release()
    # ✅ 从 ROS2 获取抓取点
    pick = get_grasp_point()

    pick_offset = pick.copy()
    pick_offset[0] -= 0.135#x轴增加人为偏移空出手的和手腕之间的距离
    pick_offset[1] += 0.04
    print(f"Raw grasp: {pick}")
    print(f"Offset grasp: {pick_offset}")

    place = PLACE_POS.copy()

    pre_pick = pick_offset.copy()
    pre_pick[2] += APPROACH_OFFSET_Z

    pre_place = place.copy()
    pre_place[2] += 0.3

    lift1 = INIT_LEFT_POS.copy()
    lift1[2] += 0.25
    lift = pick_offset.copy()
    lift[2] += 0.3
    print("Lift up 1")
    primitives.dual_arm_movement(
        left_hand_pos=lift1.tolist(),
        duration=2.5,
        left_angle_deg=0.0,
        right_angle_deg=0.0,verbose=True,
    )
    print("Move above pick")
    primitives.dual_arm_movement(
        left_hand_pos=pre_pick.tolist(),
        duration=2.5,
        left_angle_deg=90.0,
        right_angle_deg=0.0,verbose=True,
    )

    print("Move to pick")
    primitives.dual_arm_movement(
        left_hand_pos=pick_offset.tolist(),
        duration=2.5,
        left_angle_deg=80.0,
        right_angle_deg=0.0,verbose=True,
    )
    hand.execute_grasp(strength=0.33,hand="left")


    
    print("Lift up")
    primitives.dual_arm_movement(
        left_hand_pos=lift.tolist(),
        duration=2.5,
        left_angle_deg=0.0,
        right_angle_deg=0.0,verbose=True,
    )


    print("Move above place")
    primitives.dual_arm_movement(
        left_hand_pos=pre_place.tolist(),
        duration=2.5,
        left_angle_deg=90.0,
        right_angle_deg=0.0,verbose=True,
    )

    print("Move to place")
    primitives.dual_arm_movement(
        left_hand_pos=place.tolist(),
        duration=2.5,
        left_angle_deg=90.0,
        right_angle_deg=0.0,verbose=True,
    )
    hand.release()
    print("Done")


def main():
    print("Safety check passed")
    primitives = setup_real_robot()
    time.sleep(2)
    run_autonomous_pick_and_place(primitives)


if __name__ == "__main__":
    main()
    """
    *订阅/grasp话题获得目标点并执行
    """
