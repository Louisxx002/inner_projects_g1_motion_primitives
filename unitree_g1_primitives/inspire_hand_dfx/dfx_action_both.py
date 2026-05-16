import time
import numpy as np
from multiprocessing import Array
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire_hand_dfx.dfx_controller import Inspire_Controller_DFX 

class InspireHandManager:
    def __init__(self, fps=100.0):
        # 1. 初始化通信
        # ChannelFactoryInitialize(0)
        
        # 2. 定义共享数组 (1.0 为全开)
        self.action_array = Array('d', [1.0] * 12)
        
        # 3. 索引常量定义
        self.R_FINGERS = [0, 1, 2, 3]
        self.R_THUMB_BEND = 4
        self.R_THUMB_ROT = 5
        
        self.L_FINGERS = [6, 7, 8, 9]
        self.L_THUMB_BEND = 10
        self.L_THUMB_ROT = 11

        # 4. 启动底层控制器
        self.ctrl = Inspire_Controller_DFX(target_action_array=self.action_array, fps=fps)
        print("[HandManager] 控制器已启动")

    def set_all_open(self):
        """所有手指完全伸直"""
        for i in range(12):
            self.action_array[i] = 1.0

    def prepare_grasp(self):
        """大拇指旋转到对掌位置，为抓取做准备"""
        self.action_array[self.R_THUMB_ROT] = 0.0
        self.action_array[self.L_THUMB_ROT] = 0.0
        print("[HandManager] 大拇指已旋转，准备抓取")

    def _smooth_move(self, target, duration=1.0, steps=50):
        """
        内部函数：平滑插值，避免电机冲击
        """
        start = self.action_array[:]
        for i in range(steps):
            alpha = (i + 1) / steps
            for j in range(len(self.action_array)):
                self.action_array[j] = start[j] * (1 - alpha) + target[j] * alpha
            time.sleep(duration / steps)


    def execute_grasp(self, strength=0.25):
        """
        执行完整抓取动作（推荐版本）

        :param strength: 抓取位置（0.0最紧，1.0最松）
        """

        print(f"[HandManager] 开始抓取，目标力度: {strength}")

        # =========================
        # Step 1: 确保张开
        # =========================
        open_target = [1.0] * len(self.action_array)
        self._smooth_move(open_target, duration=0.6)

        # =========================
        # Step 2: 拇指对掌（关键）
        # =========================
        # thumb_angle = 35  # 想要的角度
        # value = 1.0 - thumb_angle / 90.0
        # target = self.action_array[:]
        # target[self.R_THUMB_ROT] = value
        # target[self.L_THUMB_ROT] = value
        # self._smooth_move(target, duration=0.5)

        # # =========================
        # # Step 3: 预抓（轻微闭合）
        # # =========================
        # target = self.action_array[:]
        # for idx in self.R_FINGERS + self.L_FINGERS:
        #     target[idx] = 0.6
        # self._smooth_move(target, duration=0.6)
        
        # =========================
        # Step 2: 同步预抓（拇指 + 四指一起动）
        # =========================
        thumb_angle = 45
        value = 1.0 - thumb_angle / 90.0

        target = self.action_array[:]

        # 拇指旋转
        target[self.R_THUMB_ROT] = value
        target[self.L_THUMB_ROT] = value

        # 四指开始收紧
        for idx in self.R_FINGERS + self.L_FINGERS:
            target[idx] = 0.6

        self._smooth_move(target, duration=0.8)

        # =========================
        # Step 4: 正式抓取
        # =========================
        target = self.action_array[:]

        for idx in self.R_FINGERS + self.L_FINGERS:
            target[idx] = strength

        target[self.R_THUMB_BEND] = strength
        target[self.L_THUMB_BEND] = strength

        self._smooth_move(target, duration=1.0)

        print(f"[HandManager] 抓取完成，力度: {strength}")

    def release(self):
        """释放并张开手掌"""
        self.set_all_open()
        print("[HandManager] 已释放")

    def set_custom_action(self, index, value):
        """手动设置某个电机的数值"""
        if 0 <= index < 12:
            self.action_array[index] = np.clip(value, 0.0, 1.0)

    def stop(self):
        """安全停止"""
        self.set_all_open()
        time.sleep(0.5)
        self.ctrl.stop()
        print("[HandManager] 控制器已关闭")


"""
封装成函数：控制DFX灵巧手，同时控制左右
"""