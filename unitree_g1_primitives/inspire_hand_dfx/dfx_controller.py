from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Array

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

# 常量定义
Inspire_Num_Motors = 6
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"

# 关节索引定义（用于解析状态）
class JointIndex(IntEnum):
    # 右手
    R_PINKY = 0
    R_RING = 1
    R_MIDDLE = 2
    R_INDEX = 3
    R_THUMB_BEND = 4
    R_THUMB_ROT = 5
    # 左手
    L_PINKY = 6
    L_RING = 7
    L_MIDDLE = 8
    L_INDEX = 9
    L_THUMB_BEND = 10
    L_THUMB_ROT = 11

class Inspire_Controller_DFX:
    """
    简化版的灵巧手控制器
    不再处理人手关键点，直接接收 [0.0, 1.0] 的电机指令
    """
    def __init__(self, target_action_array, state_feedback_array=None, fps=100.0):
        logger_mp.info("Initialize Simplified Inspire_Controller_DFX...")
        
        self.fps = fps
        # target_action_array 是外部传入的共享内存数组，长度为 12 (6左 + 6右)
        # 数值范围约定为 0.0 (全关) 到 1.0 (全开)
        self.target_action_array = target_action_array
        # state_feedback_array 用于向外写回当前手的实际角度
        self.state_feedback_array = state_feedback_array

        # 1. 初始化 DDS 发布者 (发送指令)
        self.HandCmd_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmd_publisher.Init()

        # 2. 初始化 DDS 订阅者 (接收状态)
        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # 3. 内部状态缓存
        self.left_hand_state = np.zeros(Inspire_Num_Motors)
        self.right_hand_state = np.zeros(Inspire_Num_Motors)

        # 4. 启动状态订阅线程
        self.subscribe_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # 5. 等待 DDS 连接（检查是否收到状态数据）
        logger_mp.info("Waiting for DDS state feedback...")
        while True:
            if np.any(self.left_hand_state) or np.any(self.right_hand_state):
                break
            time.sleep(0.1)
        
        # 6. 启动控制循环线程
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        logger_mp.info("Inspire_Controller_DFX initialized and running.")

    def _subscribe_hand_state(self):
        """后台线程：不断读取手的实时状态"""
        while True:
            hand_msg = self.HandState_subscriber.Read()
            if hand_msg is not None:
                # 提取左手 6 个电机状态
                for i in range(Inspire_Num_Motors):
                    # 根据 JointIndex 偏移量读取
                    self.left_hand_state[i] = hand_msg.states[i + 6].q
                    self.right_hand_state[i] = hand_msg.states[i].q
                
                # 如果有外部反馈数组，则同步过去
                if self.state_feedback_array is not None:
                    self.state_feedback_array[:6] = self.left_hand_state
                    self.state_feedback_array[6:] = self.right_hand_state
            time.sleep(0.005)

    def _control_loop(self):
        """核心控制循环：从共享数组读取目标值并发送指令"""
        self.running = True
        
        # 预先构造消息模板
        hand_msg = MotorCmds_()
        hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(12)]

        while self.running:
            start_time = time.time()

            # 1. 从共享内存获取最新的目标动作 [0.0 - 1.0]
            # 假设 target_action_array 的顺序是 [左手6个, 右手6个]
            current_targets = np.array(self.target_action_array[:])
            
            left_targets = current_targets[:6]
            right_targets = current_targets[6:]

            # 2. 填充消息
            # 注意：DFX 协议中 0.0 是关，1.0 是开
            for i in range(Inspire_Num_Motors):
                # 左手索引 6-11
                hand_msg.cmds[i + 6].q = float(np.clip(left_targets[i], 0.0, 1.0))
                # 右手索引 0-5
                hand_msg.cmds[i].q = float(np.clip(right_targets[i], 0.0, 1.0))

            # 3. 发送
            self.HandCmd_publisher.Write(hand_msg)

            # 4. 频率控制
            elapsed = time.time() - start_time
            sleep_time = max(0, (1.0 / self.fps) - elapsed)
            time.sleep(sleep_time)

    def stop(self):
        self.running = False
        logger_mp.info("Controller stopped.")
        """
        (*)DFX灵巧手控制代码controller
        """