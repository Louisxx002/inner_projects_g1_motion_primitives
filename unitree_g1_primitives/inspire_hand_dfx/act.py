import time
from multiprocessing import Array
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from dfx_controller import Inspire_Controller_DFX 


def smooth_move(action_array, target, duration=1.0, steps=50):
    """
    平滑插值运动（避免电机冲击）
    """
    start = action_array[:]
    for i in range(steps):
        alpha = (i + 1) / steps
        for j in range(len(action_array)):
            action_array[j] = start[j] * (1 - alpha) + target[j] * alpha
        time.sleep(duration / steps)


if __name__ == "__main__":
    ChannelFactoryInitialize(0)

    # -----------------------------
    # 1. 初始化 (1.0 = 全张开)
    # -----------------------------
    action_array = Array('d', [1.0] * 12)

    # -----------------------------
    # 2. 索引定义
    # -----------------------------
    R_FINGERS = [0, 1, 2, 3]
    R_THUMB_BEND = 4
    R_THUMB_ROT = 5

    L_FINGERS = [6, 7, 8, 9]
    L_THUMB_BEND = 10
    L_THUMB_ROT = 11

    # -----------------------------
    # 3. 启动控制器
    # -----------------------------
    ctrl = Inspire_Controller_DFX(
        target_action_array=action_array,
        fps=100.0
    )

    print("开始抓取循环（平滑控制版）...")

    try:
        while True:

            # =========================
            # Step 1: 张开手
            # =========================
            print("Step 1: Open hand")
            smooth_move(action_array, [1.0] * 12, duration=1.0)

            # =========================
            # Step 2: 拇指对掌（关键！）
            # =========================
            print("Step 2: Thumb rotate")
            target = action_array[:]

            target[R_THUMB_ROT] = 0.0
            target[L_THUMB_ROT] = 0.0

            smooth_move(action_array, target, duration=0.6)

            # =========================
            # Step 3: 预收拢（轻微弯曲）
            # =========================
            print("Step 3: Pre-grasp")
            target = action_array[:]

            for idx in R_FINGERS + L_FINGERS:
                target[idx] = 0.6  # 先轻微弯

            smooth_move(action_array, target, duration=0.8)

            # =========================
            # Step 4: 正式抓取
            # =========================
            print("Step 4: GRASP")
            target = action_array[:]

            grip_strength = 0.25  # 👈 可调（越小越紧）

            for idx in R_FINGERS + L_FINGERS:
                target[idx] = grip_strength

            target[R_THUMB_BEND] = grip_strength
            target[L_THUMB_BEND] = grip_strength

            smooth_move(action_array, target, duration=1.2)

            # =========================
            # Step 5: 保持抓取
            # =========================
            print("Step 5: Hold object")
            time.sleep(2.0)

            # =========================
            # Step 6: 松开
            # =========================
            print("Step 6: Release")
            smooth_move(action_array, [1.0] * 12, duration=1.0)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("安全退出：张开手")
        smooth_move(action_array, [1.0] * 12, duration=0.5)
        ctrl.stop()
        """
        灵巧手控制测试代码
        """