# Piper SDK interface for LeRobot integration

import time
import logging
from typing import Any

log = logging.getLogger(__name__)

try:
    from piper_sdk import C_PiperInterface_V2
except Exception:
    C_PiperInterface_V2 = None
    log.debug("piper_sdk not available at import time; use `pip install piper_sdk` if you need hardware access")


class PiperSDKInterface:
    def __init__(self, port: str = "can0", enable_timeout: float = 5.0):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed. Install with `pip install piper_sdk`.")
        try:
            self.piper = C_PiperInterface_V2(port)
        except Exception as e:
            log.error("Failed to initialize Piper SDK: %s. Did you activate the CAN interface?", e)
            self.piper = None
            raise RuntimeError("Failed to initialize Piper SDK") from e

        try:
            self.piper.ConnectPort()
            time.sleep(0.1)
        except Exception as e:
            log.error("ConnectPort failed: %s", e)
            raise

        try:
            status = self.piper.GetArmStatus().arm_status
            log.debug("Initial arm motion_status=%s ctrl_mode=%s", getattr(status, "motion_status", None), getattr(status, "ctrl_mode", None))
            if status.motion_status != 0:
                self.piper.EmergencyStop(0x02)
            if status.ctrl_mode == 2:
                log.warning("Arm is in teaching mode (ctrl_mode==2). Attempting resume.")
                self.piper.EmergencyStop(0x02)
                time.sleep(0.5)
                self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                time.sleep(0.5)
        except Exception as e:
            log.debug("Unable to read arm status: %s", e)

        start = time.time()
        while True:
            try:
                ok = self.piper.EnablePiper()
            except Exception:
                ok = False
            if ok:
                break
            if time.time() - start > enable_timeout:
                raise TimeoutError(f"EnablePiper timed out after {enable_timeout} seconds")
            time.sleep(0.01)

        try:
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        except Exception as e:
            log.warning("MotionCtrl_2 failed: %s", e)

        try:
            time.sleep(0.2)
            js = self.piper.GetArmJointMsgs().joint_state
            self.piper.JointCtrl(
                js.joint_1, js.joint_2, js.joint_3,
                js.joint_4, js.joint_5, js.joint_6
            )
        except Exception as e:
            log.warning("Could not send hold-position command: %s", e)

        try:
            angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
            self.min_pos = [pos.min_angle_limit / 10.0 for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [0.0]
            self.max_pos = [pos.max_angle_limit / 10.0 for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [10.0]
        except Exception as e:
            log.warning("Could not read joint limits: %s", e)
            self.min_pos = [-180.0] * 6 + [0.0]
            self.max_pos = [180.0] * 6 + [10.0]

        FALLBACK_MIN = [-150.0, 0.0, -170.0, -100.0, -70.0, -180.0, 0.0]
        FALLBACK_MAX = [ 150.0, 180.0, 170.0,  100.0,  70.0,  180.0, 10.0]
        for i in range(6):
            if self.max_pos[i] == 0.0 or self.max_pos[i] <= self.min_pos[i]:
                log.warning("Joint %d max_pos invalid (%s), using fallback %s", i+1, self.max_pos[i], FALLBACK_MAX[i])
                self.max_pos[i] = FALLBACK_MAX[i]
            if self.min_pos[i] == 0.0 and FALLBACK_MIN[i] < 0.0:
                log.warning("Joint %d min_pos likely wrong (%s), using fallback %s", i+1, self.min_pos[i], FALLBACK_MIN[i])
                self.min_pos[i] = FALLBACK_MIN[i]

        try:
            time.sleep(0.2)
            js = self.piper.GetArmJointMsgs().joint_state
            self.piper.JointCtrl(
                js.joint_1, js.joint_2, js.joint_3,
                js.joint_4, js.joint_5, js.joint_6
            )
            log.info("Arm holding position: j1=%s j2=%s j3=%s j4=%s j5=%s j6=%s",
                     js.joint_1, js.joint_2, js.joint_3,
                     js.joint_4, js.joint_5, js.joint_6)
        except Exception as e:
            log.warning("Could not send hold-position command: %s", e)

    def set_joint_positions(self, positions):
        if not isinstance(positions, (list, tuple)) or len(positions) < 7:
            raise ValueError("positions must be a sequence of length >=7")

        scaled_angles = []
        for i in range(6):
            p = positions[i]
            try:
                p = float(p)
            except Exception:
                p = 0.0
            p = max(-100.0, min(100.0, p))
            minv = self.min_pos[i]
            maxv = self.max_pos[i]
            angle = minv + (p + 100.0) / 200.0 * (maxv - minv)
            scaled_angles.append(int(round(angle * 1000.0)))

        g = positions[6]
        try:
            g = float(g)
        except Exception:
            g = 0.0
        g = max(0.0, min(100.0, g))
        g_mm = self.min_pos[6] + (self.max_pos[6] - self.min_pos[6]) * (g / 100.0)
        g_int = int(round(g_mm * 10000.0))

        try:
            self.piper.JointCtrl(*scaled_angles)
            self.piper.GripperCtrl(g_int, 1000, 0x01, 0)
        except Exception as e:
            log.exception("Failed to send joint/gripper via JointCtrl/GripperCtrl: %s", e)
            raise

    def get_status_deg(self) -> dict[str, float]:
        js = self.piper.GetArmJointMsgs().joint_state
        g = self.piper.GetArmGripperMsgs()
        out = {
            "joint_1.pos": js.joint_1 / 1000.0,
            "joint_2.pos": js.joint_2 / 1000.0,
            "joint_3.pos": js.joint_3 / 1000.0,
            "joint_4.pos": js.joint_4 / 1000.0,
            "joint_5.pos": js.joint_5 / 1000.0,
            "joint_6.pos": js.joint_6 / 1000.0,
        }
        try:
            out["gripper.pos"] = g.gripper_state.grippers_angle / 10000.0
        except Exception:
            pass
        return out

    def set_joint_positions_deg(self, joints_deg: list[float], gripper_mm: float | None = None) -> None:
        j_ints = [int(round(d * 1000.0)) for d in joints_deg]
        try:
            status = self.piper.GetArmStatus().arm_status
        except Exception:
            pass
        try:
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)  # must be sent every cycle
            self.piper.JointCtrl(*j_ints)
            if gripper_mm is not None:
                self.piper.GripperCtrl(int(round(gripper_mm * 10000.0)), 1000, 0x01, 0)
        except Exception as e:
            log.exception("set_joint_positions_deg failed: %s", e)
            raise

    def get_status(self) -> dict[str, Any]:
        joint_status = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()

        joint_state = joint_status.joint_state
        obs_dict = {
            "joint_0.pos": joint_state.joint_1,
            "joint_1.pos": joint_state.joint_2,
            "joint_2.pos": joint_state.joint_3,
            "joint_3.pos": joint_state.joint_4,
            "joint_4.pos": joint_state.joint_5,
            "joint_5.pos": joint_state.joint_6,
        }
        obs_dict.update(
            {
                "joint_6.pos": gripper.gripper_state.grippers_angle,
            }
        )

        return obs_dict

    def disconnect(self):
        try:
            js = self.piper.GetArmJointMsgs().joint_state
            self.piper.JointCtrl(
                js.joint_1, js.joint_2, js.joint_3,
                js.joint_4, js.joint_5, js.joint_6
            )
        except Exception:
            log.debug("Disconnect: cleanup failed or piper already disconnected")
