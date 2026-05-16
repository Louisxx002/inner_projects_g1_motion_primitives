from dex_retargeting import RetargetingConfig
from pathlib import Path
import yaml
from enum import Enum
import logging_mp
import os

logger_mp = logging_mp.getLogger(__name__)


class HandType(Enum):
    INSPIRE_HAND = "../assets/inspire_hand/inspire_hand.yml"
    INSPIRE_HAND_Unit_Test = "../../assets/inspire_hand/inspire_hand.yml"
    UNITREE_DEX3 = "../assets/unitree_hand/unitree_dex3.yml"
    UNITREE_DEX3_Unit_Test = "../../assets/unitree_hand/unitree_dex3.yml"
    BRAINCO_HAND = "../assets/brainco_hand/brainco.yml"
    BRAINCO_HAND_Unit_Test = "../../assets/brainco_hand/brainco.yml"


class HandRetargeting:
    def __init__(self, hand_type: HandType):

    
        # ✅ 正确 project root
        # ================================
        ROOT = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../..")
        )

        # Fallback for unusual launch paths.
        if not ROOT.endswith("xr_teleoperate"):
            ROOT = os.environ.get("XR_TELEOP_REPO", ROOT)

        ASSETS_DIR = os.path.join(ROOT, "assets", "inspire_hand")
        # ================================
        # ✅ 2. 强制修正 URDF 路径
        # ================================
        if hand_type in [
            HandType.INSPIRE_HAND,
            HandType.INSPIRE_HAND_Unit_Test,
            HandType.UNITREE_DEX3,
            HandType.UNITREE_DEX3_Unit_Test,
            HandType.BRAINCO_HAND,
            HandType.BRAINCO_HAND_Unit_Test,
        ]:
            # 👉 指向 assets 根目录，而不是 inspire_hand 子目录
            RetargetingConfig.set_default_urdf_dir(os.path.join(ROOT, "assets"))

        # ================================
        # 3. YAML配置文件路径（绝对路径）
        # ================================
        config_file_path = os.path.join(ROOT, "assets", hand_type.value)

        try:
            with open(config_file_path, 'r') as f:
                self.cfg = yaml.safe_load(f)

            if 'left' not in self.cfg or 'right' not in self.cfg:
                raise ValueError("Configuration file must contain 'left' and 'right' keys.")

            left_retargeting_config = RetargetingConfig.from_dict(self.cfg['left'])
            right_retargeting_config = RetargetingConfig.from_dict(self.cfg['right'])

            self.left_retargeting = left_retargeting_config.build()
            self.right_retargeting = right_retargeting_config.build()

            self.left_retargeting_joint_names = self.left_retargeting.joint_names
            self.right_retargeting_joint_names = self.right_retargeting.joint_names
            self.left_indices = self.left_retargeting.optimizer.target_link_human_indices
            self.right_indices = self.right_retargeting.optimizer.target_link_human_indices

            # ================================
            # 4. Inspire 手映射
            # ================================
            if hand_type in [
                HandType.INSPIRE_HAND,
                HandType.INSPIRE_HAND_Unit_Test
            ]:
                self.left_inspire_api_joint_names = [
                    'L_pinky_proximal_joint',
                    'L_ring_proximal_joint',
                    'L_middle_proximal_joint',
                    'L_index_proximal_joint',
                    'L_thumb_proximal_pitch_joint',
                    'L_thumb_proximal_yaw_joint'
                ]

                self.right_inspire_api_joint_names = [
                    'R_pinky_proximal_joint',
                    'R_ring_proximal_joint',
                    'R_middle_proximal_joint',
                    'R_index_proximal_joint',
                    'R_thumb_proximal_pitch_joint',
                    'R_thumb_proximal_yaw_joint'
                ]

                self.left_dex_retargeting_to_hardware = [
                    self.left_retargeting_joint_names.index(name)
                    for name in self.left_inspire_api_joint_names
                ]

                self.right_dex_retargeting_to_hardware = [
                    self.right_retargeting_joint_names.index(name)
                    for name in self.right_inspire_api_joint_names
                ]

            elif hand_type in [
                HandType.UNITREE_DEX3,
                HandType.UNITREE_DEX3_Unit_Test
            ]:
                self.left_dex3_api_joint_names = [
                    'left_hand_thumb_0_joint', 'left_hand_thumb_1_joint', 'left_hand_thumb_2_joint',
                    'left_hand_middle_0_joint', 'left_hand_middle_1_joint',
                    'left_hand_index_0_joint', 'left_hand_index_1_joint'
                ]

                self.right_dex3_api_joint_names = [
                    'right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint',
                    'right_hand_middle_0_joint', 'right_hand_middle_1_joint',
                    'right_hand_index_0_joint', 'right_hand_index_1_joint'
                ]

                self.left_dex_retargeting_to_hardware = [
                    self.left_retargeting_joint_names.index(name)
                    for name in self.left_dex3_api_joint_names
                ]

                self.right_dex_retargeting_to_hardware = [
                    self.right_retargeting_joint_names.index(name)
                    for name in self.right_dex3_api_joint_names
                ]

            elif hand_type in [
                HandType.BRAINCO_HAND,
                HandType.BRAINCO_HAND_Unit_Test
            ]:
                self.left_brainco_api_joint_names = [
                    'left_thumb_metacarpal_joint', 'left_thumb_proximal_joint',
                    'left_index_proximal_joint', 'left_middle_proximal_joint',
                    'left_ring_proximal_joint', 'left_pinky_proximal_joint'
                ]

                self.right_brainco_api_joint_names = [
                    'right_thumb_metacarpal_joint', 'right_thumb_proximal_joint',
                    'right_index_proximal_joint', 'right_middle_proximal_joint',
                    'right_ring_proximal_joint', 'right_pinky_proximal_joint'
                ]

                self.left_dex_retargeting_to_hardware = [
                    self.left_retargeting_joint_names.index(name)
                    for name in self.left_brainco_api_joint_names
                ]

                self.right_dex_retargeting_to_hardware = [
                    self.right_retargeting_joint_names.index(name)
                    for name in self.right_brainco_api_joint_names
                ]

        except FileNotFoundError:
            logger_mp.warning(f"Configuration file not found: {config_file_path}")
            raise

        except yaml.YAMLError as e:
            logger_mp.warning(f"YAML error while reading {config_file_path}: {e}")
            raise

        except Exception as e:
            logger_mp.error(f"An error occurred: {e}")
            raise
