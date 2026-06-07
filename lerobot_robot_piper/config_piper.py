from dataclasses import dataclass, field
from pathlib import Path
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("piper_follower")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
  
  id: str | None = "piper_follower_robot"
  calibration_dir: Path | None = None
  can_interface: str = "can0"
  port: str = "/dev/ttyUSB0"
  bitrate: int = 1_000_000
  
  joint_names: list[str] = field(default_factory=lambda: [f"joint_{i+1}" for i in range(6)])
  joint_signs: list[int] = field(default_factory=lambda: [-1, 1, 1, -1, 1, -1])
  joint_aliases: dict[str, str] = field(default_factory=lambda: {
    "shoulder_pan":  "joint_1",
    "shoulder_lift": "joint_2",
    "elbow_flex":    "joint_3",
    "elbow_roll":    "joint_4",
    "wrist_flex":    "joint_5",
    "wrist_roll":    "joint_6",
    })
  
  include_gripper: bool = True
  gripper_id: int = 2
  gripper_baudrate: int = 1_000_000
  cameras: dict[str, CameraConfig] = field(default_factory=dict)
  use_degrees: bool = True
  enable_timeout: float = 5.0
