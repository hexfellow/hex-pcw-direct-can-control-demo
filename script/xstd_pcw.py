import struct
import logging
from dataclasses import dataclass
from typing import List
from enum import Enum
import can

logger = logging.getLogger(__name__)

class NMTState(Enum):
    INITIALISING = 0
    RESET_APPLICATION = 1
    RESET_COMMUNICATION = 2
    PRE_OPERATIONAL = 3
    OPERATIONAL = 4
    STOPPED = 5

def set_nmt_state(node_id: int, target_state: NMTState, bus) -> None:
    if not (0 <= node_id <= 127):
        raise ValueError("Invalid node ID")
    state_code_map = {
        NMTState.OPERATIONAL: 0x01,
        NMTState.STOPPED: 0x02,
        NMTState.PRE_OPERATIONAL: 0x80,
        NMTState.RESET_APPLICATION: 0x81,
        NMTState.RESET_COMMUNICATION: 0x82,
    }
    if target_state not in state_code_map:
        raise ValueError("Invalid target NMT state")
    state_code = state_code_map[target_state]
    msg = can.Message(
        arbitration_id=0x000,
        data=[state_code, node_id],
        is_extended_id=False
    )
    bus.send(msg)

@dataclass
class MitData:
    kp: float
    kd: float
    p: float
    v: float
    t_ff: float

    @classmethod
    def torque(cls, t: float) -> 'MitData':
        return cls(kp=0.0, kd=0.0, p=0.0, v=0.0, t_ff=t)

    def to_bytes(self, mapping_config: 'MitMappingConfig') -> bytes:
        p = max(min(self.p, mapping_config.p_max), mapping_config.p_min)
        v = max(min(self.v, mapping_config.v_max), mapping_config.v_min)
        kp = max(min(self.kp, mapping_config.kp_max), mapping_config.kp_min)
        kd = max(min(self.kd, mapping_config.kd_max), mapping_config.kd_min)
        t_ff = max(min(self.t_ff, mapping_config.t_max), mapping_config.t_min)

        pos = XstdPcw.float_to_uint(p, mapping_config.p_min, mapping_config.p_max, 16)
        vel = XstdPcw.float_to_uint(v, mapping_config.v_min, mapping_config.v_max, 12)
        kp_val = XstdPcw.float_to_uint(kp, mapping_config.kp_min, mapping_config.kp_max, 12)
        kd_val = XstdPcw.float_to_uint(kd, mapping_config.kd_min, mapping_config.kd_max, 12)
        t_ff_val = XstdPcw.float_to_uint(t_ff, mapping_config.t_min, mapping_config.t_max, 12)

        return bytes([
            (pos >> 8) & 0xFF,
            pos & 0xFF,
            (vel >> 4) & 0xFF,
            ((vel & 0x0F) << 4 | (kp_val >> 8)) & 0xFF,
            kp_val & 0xFF,
            (kd_val >> 4) & 0xFF,
            ((kd_val & 0x0F) << 4 | (t_ff_val >> 8)) & 0xFF,
            t_ff_val & 0xFF,
        ])

@dataclass
class MitMappingConfig:
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0
    p_min: float = -3.1415926
    p_max: float = 3.1415926
    v_min: float = -50.0
    v_max: float = 50.0
    t_min: float = -10.0
    t_max: float = 10.0

class MotorControlMode:
    LOCK = 0
    POSITION = 1
    SPEED = 2
    TORQUE = 3
    MIT = 4

    def __init__(self, mode_type: int, value=None):
        self.mode_type = mode_type
        self.value = value

    @classmethod
    def lock(cls) -> 'MotorControlMode':
        return cls(cls.LOCK)

    @classmethod
    def position(cls, pos: int) -> 'MotorControlMode':
        return cls(cls.POSITION, pos)

    @classmethod
    def speed(cls, speed: float) -> 'MotorControlMode':
        return cls(cls.SPEED, speed)

    @classmethod
    def torque(cls, torque: float) -> 'MotorControlMode':
        return cls(cls.TORQUE, torque)

    @classmethod
    def mit(cls, mit_data: MitData) -> 'MotorControlMode':
        return cls(cls.MIT, mit_data)

class XstdPcw:

    def __init__(self, canopen_id: int):
        self.canopen_id = canopen_id
        self.read_control_modes = [MotorControlMode.lock(), MotorControlMode.lock()]
        self.target_control_modes = [MotorControlMode.lock(), MotorControlMode.lock()]
        self.encoder_resolutions = [0xFFFF, 0xFFFF]
        self.speed_mapping_maxs = [50.0, 50.0]
        self.speed_mapping_mins = [-50.0, -50.0]
        self.torque_mapping_maxs = [10.0, 10.0]
        self.torque_mapping_mins = [-10.0, -10.0]
        self.motor_temp_zero = [0, 0]
        self.driver_temp_zero = [0, 0]

    def get_canopen_id(self) -> int:
        return self.canopen_id

    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        span = x_max - x_min
        offset = x_min
        scale = (1 << bits) - 1
        return int((x - offset) * scale / span)

    @staticmethod
    def float_to_i16(x: float, x_min: float, x_max: float) -> int:
        x = max(min(x, x_max), x_min)
        scale = 65534.0  # 32767 * 2
        span = x_max - x_min
        return int(((x - x_min) * scale / span) - 32767.0)

    @staticmethod
    def i16_to_float(x: int, x_min: float, x_max: float) -> float:
        if x == -32768:
            x = -32767
        scale = 65534.0  # 32767 * 2
        span = x_max - x_min
        return x_min + (x + 32767.0) * span / scale

    def set_control_mode(self, modes: List[MotorControlMode]):
        self.target_control_modes = modes

    def process_can_msg(self, msg) -> bool:
        if msg.arbitration_id & 0b0111_0000000 == 0b1110_0000000:
            # HEARTBEAT
            if len(msg.data) < 1:
                return False
            state = msg.data[0]
            if state != 0x05:
                logger.warning("NMT State is not Operational anymore; You should check if anything goes wrong")
        
        elif msg.arbitration_id & 0b0111_0000000 == 0b0011_0000000:
            # TPDO1
            if len(msg.data) != 24:
                logger.error(f"Received message with unexpected length: {len(msg.data)}")
                return False

            m0_pos = struct.unpack('<i', msg.data[0:4])[0]
            m1_pos = struct.unpack('<i', msg.data[4:8])[0]
            m0_speed = struct.unpack('<H', msg.data[8:10])[0]
            m1_speed = struct.unpack('<H', msg.data[10:12])[0]
            m0_torque = struct.unpack('<H', msg.data[12:14])[0]
            m1_torque = struct.unpack('<H', msg.data[14:16])[0]

            m0_speed = self.i16_to_float(m0_speed, self.speed_mapping_mins[0], self.speed_mapping_maxs[0])
            m1_speed = self.i16_to_float(m1_speed, self.speed_mapping_mins[1], self.speed_mapping_maxs[1])
            m0_torque = self.i16_to_float(m0_torque, self.torque_mapping_mins[0], self.torque_mapping_maxs[0])
            m1_torque = self.i16_to_float(m1_torque, self.torque_mapping_mins[1], self.torque_mapping_maxs[1])

            self.read_control_modes[0] = MotorControlMode(msg.data[16])
            self.read_control_modes[1] = MotorControlMode(msg.data[17])

            m0_error_code = msg.data[18]
            m1_error_code = msg.data[19]
            m0_mt_temp = msg.data[20] + self.motor_temp_zero[0]
            m1_mt_temp = msg.data[21] + self.motor_temp_zero[1]
            m0_driver_temp = msg.data[22] + self.driver_temp_zero[0]
            m1_driver_temp = msg.data[23] + self.driver_temp_zero[1]

            logger.info(
                f"Motor1: Mode {self.read_control_modes[0].mode_type}, "
                f"Position {m0_pos}, Speed {m0_speed}, Torque {m0_torque}, "
                f"Error Code {m0_error_code}, Motor Temp {m0_mt_temp}, Driver Temp {m0_driver_temp}"
            )
            logger.info(
                f"Motor2: Mode {self.read_control_modes[1].mode_type}, "
                f"Position {m1_pos}, Speed {m1_speed}, Torque {m1_torque}, "
                f"Error Code {m1_error_code}, Motor Temp {m1_mt_temp}, Driver Temp {m1_driver_temp}"
            )
        return True

    @staticmethod
    def generate_control_frame(pcws: List['XstdPcw']):
        data = bytearray()
        for pcw in pcws:
            data.append(pcw.target_control_modes[0].mode_type)
            data.append(pcw.target_control_modes[1].mode_type)
        for pcw in pcws:
            for i, mode in enumerate(pcw.target_control_modes):
                if mode.mode_type == MotorControlMode.POSITION:
                    data.extend(struct.pack('<i', mode.value))
                elif mode.mode_type == MotorControlMode.SPEED:
                    speed = XstdPcw.float_to_i16(
                        mode.value,
                        pcw.speed_mapping_mins[i],
                        pcw.speed_mapping_maxs[i],
                    )
                    data.extend(struct.pack('<H', speed))
                elif mode.mode_type == MotorControlMode.TORQUE:
                    torque = XstdPcw.float_to_i16(
                        mode.value,
                        pcw.torque_mapping_mins[i],
                        pcw.torque_mapping_maxs[i],
                    )
                    data.extend(struct.pack('<H', torque))
                elif mode.mode_type == MotorControlMode.MIT:
                    data.extend(mode.value.to_bytes(MitMappingConfig()))
        logger.info(f"Sending control frame: {data.hex()}")
        return can.Message(
            arbitration_id=0x181,
            data=data,
            is_extended_id=False,
            is_fd=len(data) > 8
        ) 