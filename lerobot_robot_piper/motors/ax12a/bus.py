import threading
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from .tables import (
    AX12A_CONTROL_TABLE,
    AX12A_BAUDRATE_TABLE,
    AX12A_MIN_POSITION,
    AX12A_MAX_POSITION,
    AX12A_MODEL_NUMBER,
)

PROTOCOL_VERSION = 1.0

MIDPOINT_VALUE = 500
MIN_VALUE = 150
MAX_VALUE = 900


class AX12ABus:
    def __init__(self, port: str, motors: dict, baudrate: int = 1_000_000):
        print(f"AX12ABus __init__ called — port={port}", flush=True)
        if baudrate not in AX12A_BAUDRATE_TABLE:
            raise ValueError(
                f"Baudrate {baudrate} not supported. "
                f"Choose from {list(AX12A_BAUDRATE_TABLE.keys())}"
            )
        self.port     = port
        self.motors   = motors
        self.baudrate = baudrate

        self._connected  = False
        self._calibrated = False

        self._cal_min = {name: AX12A_MIN_POSITION for name in motors}
        self._cal_max = {name: AX12A_MAX_POSITION for name in motors}

    def connect(self):
        self.port_handler   = PortHandler(self.port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")

        for name, motor_id in self.motors.items():
            model, result, error = self.packet_handler.ping(
                self.port_handler, motor_id
            )
            if result != COMM_SUCCESS:
                raise RuntimeError(
                    f"Motor '{name}' (id={motor_id}) not found on {self.port}. "
                    f"Check wiring and ID."
                )
            if model != AX12A_MODEL_NUMBER:
                raise RuntimeError(
                    f"Motor '{name}' returned model {model}, "
                    f"expected AX-12A ({AX12A_MODEL_NUMBER})."
                )

        self._connected = True
        
        for name, motor_id in self.motors.items():
            cw  = self._read("CW_Angle_Limit",  motor_id)
            ccw = self._read("CCW_Angle_Limit", motor_id)
            if cw == 0 and ccw == 0:
                print(f"Motor '{name}' is in wheel mode — switching to joint mode")
                self._write("CW_Angle_Limit",  motor_id, 0)
                self._write("CCW_Angle_Limit", motor_id, 1023)

        self.enable_torque()
        self.set_compliance_margin(cw=0, ccw=0)
        self.set_compliance_slope(cw=32, ccw=32)
        print(f"AX-12A connected on {self.port} — motors: {self.motors}")

    def disconnect(self):
        if self._connected:
            self.disable_torque()
            self.port_handler.closePort()
            self._connected = False
            print("AX-12A disconnected.")

    @property
    def is_connected(self):
        return self._connected

    @property
    def is_calibrated(self):
        return self._calibrated

    def _write(self, register: str, motor_id: int, value: int):
        if register not in AX12A_CONTROL_TABLE:
            raise KeyError(f"Unknown register: '{register}'")

        address, size = AX12A_CONTROL_TABLE[register]

        if size == 1:
            result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, address, value
            )
        elif size == 2:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, address, value
            )
        else:
            raise ValueError(f"Unsupported size {size} for '{register}'")

        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Write failed — motor {motor_id}, register '{register}': "
                f"{self.packet_handler.getTxRxResult(result)}"
            )

    def _read(self, register: str, motor_id: int) -> int:
        if register not in AX12A_CONTROL_TABLE:
            raise KeyError(f"Unknown register: '{register}'")

        address, size = AX12A_CONTROL_TABLE[register]

        if size == 1:
            value, result, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, address
            )
        elif size == 2:
            value, result, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, address
            )
        else:
            raise ValueError(f"Unsupported size {size} for '{register}'")

        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"Read failed — motor {motor_id}, register '{register}': "
                f"{self.packet_handler.getTxRxResult(result)}"
            )

        return value

    def _normalize(self, name: str, raw: int) -> float:
        cal_min = self._cal_min[name]
        cal_max = self._cal_max[name]
        if cal_max == cal_min:
            return 0.0 
        normalized = (raw - cal_min) / (cal_max - cal_min)
        return (normalized * 200.0) - 100.0

    def _denormalize(self, name: str, value: float) -> int:
        cal_min = self._cal_min[name]
        cal_max = self._cal_max[name]
        normalized = (value + 100.0) / 200.0
        raw = int(normalized * (cal_max - cal_min) + cal_min)
        return max(cal_min, min(cal_max, raw))

    def sync_read(self, register: str, motor_names: list) -> dict:
        result = {}
        for name in motor_names:
            if name not in self.motors:
                raise KeyError(f"Unknown motor: '{name}'")
            raw = self._read(register, self.motors[name])
            result[name] = (
                self._normalize(name, raw)
                if register == "Present_Position"
                else raw
            )
        return result

    def sync_write(self, register: str, values: dict):
        for name, value in values.items():
            if name not in self.motors:
                raise KeyError(f"Unknown motor: '{name}'")
            raw = (
                self._denormalize(name, value)
                if register == "Goal_Position"
                else int(value)
            )
            self._write(register, self.motors[name], raw)
            if register == "Goal_Position":
                current = self._read("Present_Position", self.motors[name])

    def disable_torque(self):
        for name, motor_id in self.motors.items():
            self._write("Torque_Enable", motor_id, 0)
        print("Torque disabled.")

    def enable_torque(self):
        for name, motor_id in self.motors.items():
            self._write("Torque_Enable", motor_id, 1)
        print("Torque enabled.")

    def is_moving(self) -> dict:
        return {
            name: bool(self._read("Moving", motor_id))
            for name, motor_id in self.motors.items()
        }

    def read_temperature(self) -> dict:
        return {
            name: self._read("Present_Temperature", motor_id)
            for name, motor_id in self.motors.items()
        }

    def set_compliance_slope(self, cw: int = 32, ccw: int = 32):
        for name, motor_id in self.motors.items():
            self._write("CW_Compliance_Slope",  motor_id, cw)
            self._write("CCW_Compliance_Slope", motor_id, ccw)

    def set_compliance_margin(self, cw: int = 1, ccw: int = 1):
        for name, motor_id in self.motors.items():
            self._write("CW_Compliance_Margin",  motor_id, cw)
            self._write("CCW_Compliance_Margin", motor_id, ccw)

    def set_midpoint(self):
        self._midpoints = {}
        for name, motor_id in self.motors.items():
            self._midpoints[name] = self._read("Present_Position", motor_id)
        print(f"Midpoints recorded: {self._midpoints}")

    def record_ranges_of_motion(self):
        for name in self.motors:
            self._cal_min[name] = AX12A_MAX_POSITION
            self._cal_max[name] = AX12A_MIN_POSITION

        stop_event = threading.Event()

        def record_loop():
            while not stop_event.is_set():
                for name, motor_id in self.motors.items():
                    try:
                        pos = self._read("Present_Position", motor_id)
                        print(f"record_loop read: {pos}", flush=True)
                        self._cal_min[name] = min(self._cal_min[name], pos)
                        self._cal_max[name] = max(self._cal_max[name], pos)
                    except RuntimeError as e:
                        print(f"record_loop error: {e}", flush=True)

        t = threading.Thread(target=record_loop, daemon=True)
        t.start()
        input("Slowly move gripper through FULL range, then press ENTER...")
        stop_event.set()
        t.join()

        for name in self.motors:
            span = self._cal_max[name] - self._cal_min[name]
            if span < 10:
                print(
                    f"WARNING: '{name}' range too small ({span} steps). "
                    f"Did you move it enough?"
                )

        self._calibrated = True
        print(f"Calibration done — min: {self._cal_min}, max: {self._cal_max}")

    def enter_values(self):
        self._midpoints = {}
        for name in self.motors:
            raw = input("Enter midpoint value: ").strip()
            self._midpoints[name] = int(raw) if raw else MIDPOINT_VALUE

            raw = input("Enter minimal value: ").strip()
            self._cal_min[name] = int(raw) if raw else MIN_VALUE

            raw = input("Enter maximum value: ").strip()
            self._cal_max[name] = int(raw) if raw else MAX_VALUE

        self._calibrated = True
