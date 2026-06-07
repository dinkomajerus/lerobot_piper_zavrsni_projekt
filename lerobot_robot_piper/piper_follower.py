from typing import Any
import logging

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot

from .config_piper import PiperFollowerConfig
from .piper_sdk_interface import PiperSDKInterface

from .motors.ax12a import AX12ABus

logger = logging.getLogger(__name__)


class PiperFollower(Robot):
    config_class = PiperFollowerConfig
    name = "piper_follower"

    # Use **kwargs to catch __parent__ or any other factory parameters dynamically
    def __init__(self, config: PiperFollowerConfig, *args, **kwargs):
        super().__init__(config, *args, **kwargs)
        self.config = config
        # ...

        if config.include_gripper:
            self.gripper_bus = AX12ABus(
                port=config.port,
                motors={"gripper": config.gripper_id},
                baudrate=config.gripper_baudrate,
        )
        else:       
            self.gripper_bus = None

        self._iface: PiperSDKInterface | None = None
        self.cameras = make_cameras_from_configs(config.cameras) if config.cameras else {}

    @property
    def is_connected(self) -> bool:
        return (
            self._iface is not None
            and getattr(self._iface, "piper", None) is not None
            and all(cam.is_connected for cam in self.cameras.values())
            and (self.gripper_bus is None or self.gripper_bus.is_connected)
        )

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in self.config.joint_names}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {k: (c.height, c.width, 3) for k, c in self.cameras.items()}

    @property
    def observation_features(self) -> dict:
        ft = {**self._motors_ft, **self._cameras_ft}
        if self.config.include_gripper:
            ft["gripper.pos"] = float
        return ft

    @property
    def action_features(self) -> dict:
        ft = {f"{alias}.pos": float for alias in self.config.joint_aliases}
        if self.config.include_gripper:
            ft["gripper.pos"] = float
        return ft

    def connect(self, calibrate: bool = True) -> None:
        if self._iface is None:
            self._iface = PiperSDKInterface(
                port=self.config.can_interface,
                enable_timeout=self.config.enable_timeout,
            )
        for cam in self.cameras.values():
            cam.connect()
        if self.gripper_bus is not None:
            self.gripper_bus.connect()
            print("GRIPPER CONNECTED")
        if calibrate and self.gripper_bus is not None and not self.gripper_bus.is_calibrated:
            self.calibrate()
            print("GRIPPER TORQUE STATUS AFTER CALIBRATE")
        self.configure()
        print("CONNECT DONE")

    def disconnect(self) -> None:
        if self._iface is not None:
            self._iface = None
        for cam in self.cameras.values():
            cam.disconnect()
        if self.gripper_bus is not None:
            print("DISCONNECT CALLED")
            self.gripper_bus.disconnect()

    @property
    def is_calibrated(self) -> bool:
        if self.gripper_bus is not None:
            return self.gripper_bus.is_calibrated
        return True

    def calibrate(self) -> None:
        if self.gripper_bus is None:
            return
        print("Calibrating AX-12A gripper...")
        self.gripper_bus.disable_torque()
        #input("Move gripper to MIDDLE of its range, then press ENTER...")
        #self.gripper_bus.set_midpoint()
        #self.gripper_bus.record_ranges_of_motion()
        self.gripper_bus.enter_values()
        self.gripper_bus.enable_torque()
        print("Gripper calibration complete.")

    def configure(self) -> None:
        pass

    def _apply_signs(self, joints_deg: list[float]) -> list[float]:
        signs = self.config.joint_signs
        return [d * s for d, s in zip(joints_deg, signs, strict=True)]

    def _get_hw_limits(self) -> tuple[list[float], list[float]]:
        if self._iface is None:
            raise RuntimeError("Piper SDK interface not available")
        min_pos = getattr(self._iface, "min_pos", None)
        max_pos = getattr(self._iface, "max_pos", None)
        if not isinstance(min_pos, list) or not isinstance(max_pos, list) or len(min_pos) < 7 or len(max_pos) < 7:
            raise RuntimeError("Piper SDK limits unavailable")
        return min_pos, max_pos

    def _get_oriented_limits(self) -> tuple[list[float], list[float]]:
        min_pos, max_pos = self._get_hw_limits()
        oriented_min: list[float] = []
        oriented_max: list[float] = []
        for idx, sign in enumerate(self.config.joint_signs):
            hw_min = min_pos[idx]
            hw_max = max_pos[idx]
            if sign >= 0:
                oriented_min.append(hw_min)
                oriented_max.append(hw_max)
            else:
                oriented_min.append(-hw_max)
                oriented_max.append(-hw_min)
        return oriented_min, oriented_max

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected or self._iface is None:
            raise ConnectionError(f"{self} is not connected.")
        status = self._iface.get_status_deg()

        if not self.config.use_degrees:
            oriented_min, oriented_max = self._get_oriented_limits()

            def deg_to_pct(deg: float, idx: int) -> float:
                rng_min = oriented_min[idx]
                rng_max = oriented_max[idx]
                if rng_max <= rng_min:
                    return 0.0
                pct = (deg - rng_min) / (rng_max - rng_min) * 200.0 - 100.0
                return max(-100.0, min(100.0, pct))
        else:
            def deg_to_pct(deg: float, idx: int) -> float:
                return deg

        obs = {}
        for i, name in enumerate(self.config.joint_names, start=1):
            deg = status[f"joint_{i}.pos"] * self.config.joint_signs[i - 1]
            obs[f"{name}.pos"] = deg if self.config.use_degrees else deg_to_pct(deg, i - 1)

        if self.config.include_gripper and self.gripper_bus is not None:
            gripper = self.gripper_bus.sync_read("Present_Position", ["gripper"])
            obs["gripper.pos"] = -gripper["gripper"]

        for alias, target in self.config.joint_aliases.items():
            target_key = f"{target}.pos"
            alias_key = f"{alias}.pos"
            if target_key in obs and alias_key not in obs:
                obs[alias_key] = obs[target_key]

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected or self._iface is None:
            raise ConnectionError(f"{self} is not connected.")
        try:
            obs = self.get_observation()
        except Exception:
            obs = {f"{name}.pos": 0.0 for name in self.config.joint_names}
            if self.config.include_gripper:
                obs["gripper.pos"] = 0.0

        hw_min, hw_max = self._get_hw_limits()

        if self.config.use_degrees:
            def to_oriented_deg(value: float, idx: int) -> float:
                return value
        else:
            oriented_min, oriented_max = self._get_oriented_limits()

            def to_oriented_deg(value: float, idx: int) -> float:
                p = max(-100.0, min(100.0, value))
                p01 = (p + 100.0) / 200.0
                rng_min = oriented_min[idx]
                rng_max = oriented_max[idx]
                if rng_max <= rng_min:
                    return rng_min
                return rng_min + p01 * (rng_max - rng_min)

        name_to_idx = {name: idx for idx, name in enumerate(self.config.joint_names)}
        oriented_deg: dict[str, float] = {}

        for name, idx in name_to_idx.items():
            key = f"{name}.pos"
            raw = action.get(key, obs.get(key, 0.0))
            try:
                val = float(raw)
            except Exception:
                logger.warning("Invalid value for %s: %r, falling back to observation/default", key, raw)
                val = float(obs.get(key, 0.0))
            oriented_deg[name] = to_oriented_deg(val, idx)

        for alias, target in self.config.joint_aliases.items():
            alias_key = f"{alias}.pos"
            if alias_key not in action or target not in oriented_deg:
                continue
            idx = name_to_idx[target]
            raw = action[alias_key]
            try:
                val = float(raw)
            except Exception:
                logger.warning("Invalid value for %s: %r, ignoring alias", alias_key, raw)
                continue
            oriented_deg[target] = to_oriented_deg(val, idx)


        joints_hw_deg = []
        for name, idx in name_to_idx.items():
            deg_oriented = oriented_deg[name]
            deg_hw = deg_oriented * self.config.joint_signs[idx]
            deg_hw = max(hw_min[idx], min(hw_max[idx], deg_hw))
            joints_hw_deg.append(deg_hw)

        if self.config.include_gripper and self.gripper_bus is not None:
            g_raw = action.get("gripper.pos", obs.get("gripper.pos", None))
            if g_raw is not None:
                try:
                    g_inverted = -float(g_raw)
                    self.gripper_bus.sync_write(
                        "Goal_Position", {"gripper": float(g_raw)}
                    )
                except Exception as e:
                    logger.warning("Invalid gripper.pos value %r, ignoring: %s", g_raw, e)

        try:
            self._iface.set_joint_positions_deg(joints_hw_deg)
        except Exception as e:
            logger.exception("Failed to send joint positions: %s", e)
            raise
        
        return action
