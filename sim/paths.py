"""Shared paths and robot constants. Single source of truth for the repo."""
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
ASSETS_DIR = REPO_ROOT / "assets" / "so101"
SCENE_XML = ASSETS_DIR / "scene.xml"
ROBOT_URDF = ASSETS_DIR / "so101_new_calib.urdf"
OUTPUTS_DIR = REPO_ROOT / "outputs"

# The five arm joints, base to wrist. The gripper joint is handled separately.
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_JOINT = "gripper"

# End-effector reference frame. Same frame exists in both model files:
#   MJCF site "gripperframe"  <->  URDF link "gripper_frame_link"
EE_SITE = "gripperframe"
EE_URDF_LINK = "gripper_frame_link"
URDF_BASE_LINK = "base_link"
