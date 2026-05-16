import time
import numpy as np
from multiprocessing import Array
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire_hand_dfx.dfx_controller import Inspire_Controller_DFX 


class InspireHandManager:
    def __init__(self, fps=100.0):
        # ChannelFactoryInitialize(0)

        # 共享数组 (1.0 为全开)
        self.action_array = Array('d', [1.0] * 12)

        # 右手
        self.R_FINGERS = [0, 1, 2, 3]
        self.R_THUMB_BEND = 4
        self.R_THUMB_ROT = 5

        # 左手
        self.L_FINGERS = [6, 7, 8, 9]
        self.L_THUMB_BEND = 10
        self.L_THUMB_ROT = 11

        # 控制器
        self.ctrl = Inspire_Controller_DFX(
            target_action_array=self.action_array, fps=fps
        )
        print("[HandManager] 控制器已启动")

    # =========================
    # 工具函数：选择控制哪只手
    # =========================
    def _get_active_indices(self, hand):
        if hand == "right":
            fingers = self.R_FINGERS
            thumb_bend = [self.R_THUMB_BEND]
            thumb_rot = [self.R_THUMB_ROT]

        elif hand == "left":
            fingers = self.L_FINGERS
            thumb_bend = [self.L_THUMB_BEND]
            thumb_rot = [self.L_THUMB_ROT]

        elif hand == "both":
            fingers = self.R_FINGERS + self.L_FINGERS
            thumb_bend = [self.R_THUMB_BEND, self.L_THUMB_BEND]
            thumb_rot = [self.R_THUMB_ROT, self.L_THUMB_ROT]

        else:
            raise ValueError("hand must be 'left', 'right', or 'both'")

        return fingers, thumb_bend, thumb_rot

    # =========================
    # 基础控制
    # =========================
    def set_all_open(self):
        for i in range(12):
            self.action_array[i] = 1.0

    def release(self):
        self.set_all_open()
        print("[HandManager] 已释放")

    def set_custom_action(self, index, value):
        if 0 <= index < 12:
            self.action_array[index] = np.clip(value, 0.0, 1.0)

    # =========================
    # 平滑运动
    # =========================
    def _smooth_move(self, target, duration=1.0, steps=50):
        start = list(self.action_array)

        for i in range(steps):
            alpha = (i + 1) / steps
            for j in range(len(self.action_array)):
                self.action_array[j] = (
                    start[j] * (1 - alpha) + target[j] * alpha
                )
            time.sleep(duration / steps)

    # =========================
    # 主抓取函数（支持左/右/双手）
    # =========================
    def execute_grasp(self, strength=0.25, hand="both"):
        print(f"[HandManager] 开始抓取 | 手: {hand} | 力度: {strength}")

        fingers, thumb_bend, thumb_rot = self._get_active_indices(hand)

        # -------------------------
        # Step 1: 张开
        # -------------------------
        open_target = [1.0] * len(self.action_array)
        self._smooth_move(open_target, duration=0.6)

        # -------------------------
        # Step 2: 同步预抓（关键）
        # -------------------------
        thumb_angle = 45  # 35可调
        value = 1.0 - thumb_angle / 90.0

        target = list(self.action_array)

        # 拇指旋转
        for idx in thumb_rot:
            target[idx] = value

        # 四指收紧
        for idx in fingers:
            target[idx] = 0.6

        self._smooth_move(target, duration=0.8)

        # -------------------------
        # Step 3: 正式抓取
        # -------------------------
        target = list(self.action_array)

        for idx in fingers:
            target[idx] = strength

        for idx in thumb_bend:
            target[idx] = 0.58
        target[9] = 0.2#小拇指
        self._smooth_move(target, duration=1.0)

        print(f"[HandManager] 抓取完成 | 手: {hand}")

    # =========================
    # 停止
    # =========================
    def stop(self):
        self.set_all_open()
        time.sleep(0.5)
        self.ctrl.stop()
        print("[HandManager] 控制器已关闭")


# =========================
# 测试入口
# =========================
if __name__ == "__main__":
    hand = InspireHandManager()

    time.sleep(1)

    # 测试不同模式
    hand.execute_grasp(0.3, hand="right")
    time.sleep(2)

    hand.release()
    time.sleep(2)

    hand.execute_grasp(0.3, hand="left")
    time.sleep(2)

    hand.release()
    time.sleep(2)

    hand.execute_grasp(0.3, hand="both")
    time.sleep(2)

    hand.stop()
    """
    (*)封装成函数
    """