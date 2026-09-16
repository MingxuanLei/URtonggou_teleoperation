"""
Python port of DMMotor.cs.

The packing/unpacking rules are kept consistent with the original C# code:
- MIT mode uses compact fixed-point encoding in 8 bytes.
- PV mode uses two little-endian float32 values: position, velocity.
- PVT mode uses little-endian float32 position + uint16 velocity + uint16 current.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Optional, Sequence


@dataclass
class MITCommand:
    position_set: float = 0.0
    velocity_set: float = 0.0
    torque_set: float = 0.0
    kp_set: float = 0.0
    kd_set: float = 0.0


@dataclass
class PVCommand:
    position_set: float = 0.0
    velocity_lim: float = 0.0


@dataclass
class PVTCommand:
    position_set: float = 0.0
    velocity_lim: float = 0.0
    torque_lim: float = 0.0


class DMMotor:
    PARAM_SET_ID = 0x7FF

    enable_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    disable_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    clear_error_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB])
    set_zero_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    def __init__(self, motorID: int, motorType: int, mode: str):
        self._id = int(motorID)
        self._master_id = self._id + 0x010

        self.max_position = 0.0
        self.max_velocity = 0.0
        self.max_torque = 0.0
        self.kp_max = 500.0
        self.kd_max = 5.0

        self._position = 0.0
        self._velocity = 0.0
        self._torque = 0.0
        self._ERR = 0
        self.tem_mos = 0
        self.tem_rotor = 0
        self._enable = False

        self._mit_command = bytearray(8)
        self._pv_command = bytearray(8)
        self._pvt_command = bytearray(8)

        self.recv_num = 0
        self.set_mit_command = bytearray([self._id & 0xFF, 0x00, 0x55, 10, 0x01, 0x00, 0x00, 0x00])
        self.set_pv_command = bytearray([self._id & 0xFF, 0x00, 0x55, 10, 0x02, 0x00, 0x00, 0x00])
        self.set_pvt_command = bytearray([self._id & 0xFF, 0x00, 0x55, 10, 0x04, 0x00, 0x00, 0x00])
        self.get_mode_command = bytearray([self._id & 0xFF, 0x00, 0x33, 10, 0x00, 0x00, 0x00, 0x00])

        self.MIT = MITCommand()
        self.PV = PVCommand()
        self.PVT = PVTCommand()
        self.angle_lim = [-math.pi, math.pi]

        if motorType == 4340:
            self.max_position, self.max_velocity, self.max_torque = 12.5, 10.0, 28.0
        elif motorType == 4310:
            self.max_position, self.max_velocity, self.max_torque = 12.5, 30.0, 10.0
        elif motorType == 3507:
            self.max_position, self.max_velocity, self.max_torque = 12.5, 50.0, 5.0
        else:
            self.max_position, self.max_velocity, self.max_torque = 12.5, 10.0, 28.0

        mode_map = {"MIT": 1, "PV": 2, "PVT": 4}
        self._mode = mode_map.get(mode.upper(), 2)
        self.set_empty_command()

    @property
    def ERRCODE(self) -> str:
        return {
            0: "失能",
            1: "使能",
            0x08: "超压",
            0x09: "欠压",
            0x0A: "过流",
            0x0B: "MOS过热",
            0x0C: "线圈过热",
            0x0D: "通讯丢失",
            0x0E: "过载",
        }.get(self._ERR, "??")

    @property
    def Position(self) -> float:
        return self._position

    @property
    def Velocity(self) -> float:
        return self._velocity

    @property
    def Torque(self) -> float:
        return self._torque

    @property
    def Command(self) -> bytes:
        if self._mode == 1:
            return bytes(self._mit_command)
        if self._mode == 2:
            return bytes(self._pv_command)
        if self._mode == 4:
            return bytes(self._pvt_command)
        if self._mode == -1:
            return bytes(self.set_mit_command)
        if self._mode == -2:
            return bytes(self.set_pv_command)
        if self._mode == -4:
            return bytes(self.set_pvt_command)
        return self.disable_command

    @property
    def ID_OFFSET(self) -> int:
        if self._mode == 1:
            return self._id
        if self._mode == 2:
            return self._id + 0x100
        if self._mode == 4:
            return self._id + 0x300
        if self._mode in (-1, -2, -4):
            return self.PARAM_SET_ID
        return self._id

    @property
    def ID(self) -> int:
        return self._id

    @property
    def ID_MASTER(self) -> int:
        return self._master_id

    @property
    def Enable(self) -> bool:
        return self._enable

    @property
    def Mode(self) -> int:
        return self._mode

    @property
    def ModeName(self) -> str:
        return {1: "MIT", 2: "PV", 4: "PVT"}.get(self._mode, "??")

    @staticmethod
    def _clip(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(x)))

    def _convert_to_candata_MIT(self, Position: float, Velocity: float, Torque: float, KP: float, KD: float) -> bytes:
        Position = self._clip(Position, -self.max_position, self.max_position)
        Velocity = self._clip(Velocity, -self.max_velocity, self.max_velocity)
        Torque = self._clip(Torque, -self.max_torque, self.max_torque)
        KP = self._clip(KP, 0.0, self.kp_max)
        KD = self._clip(KD, 0.0, self.kd_max)

        data = bytearray(8)

        pos = int((Position + self.max_position) * 65535 / (2 * self.max_position))
        vel = int((Velocity + self.max_velocity) * 4095 / (2 * self.max_velocity))
        kp = int(KP * 4095 / self.kp_max)
        kd = int(KD * 4095 / self.kd_max)
        tor = int((Torque + self.max_torque) * 4095 / (2 * self.max_torque))

        pos = max(0, min(65535, pos))
        vel = max(0, min(4095, vel))
        kp = max(0, min(4095, kp))
        kd = max(0, min(4095, kd))
        tor = max(0, min(4095, tor))

        data[0] = (pos >> 8) & 0xFF
        data[1] = pos & 0xFF
        data[2] = (vel >> 4) & 0xFF
        data[3] = ((vel & 0x0F) << 4) | ((kp >> 8) & 0x0F)
        data[4] = kp & 0xFF
        data[5] = (kd >> 4) & 0xFF
        data[6] = ((kd & 0x0F) << 4) | ((tor >> 8) & 0x0F)
        data[7] = tor & 0xFF

        return bytes(data)

    @staticmethod
    def _convert_to_candata_PV(Position: float, Velocity: float) -> bytes:
        return struct.pack("<ff", float(Position), float(Velocity))

    @staticmethod
    def _convert_to_candata_PVT(Position: float, Velocity: int, Current: int) -> bytes:
        current = min(int(Current) & 0xFFFF, 5000)
        velocity = min(int(Velocity) & 0xFFFF, 5000)
        position = min(float(Position), 2.35)
        return struct.pack("<fHH", position, velocity, current)

    @classmethod
    def _convert_to_candata_PVT_from_torque(cls, Position: float, Velocity: int, Torque: float) -> bytes:
        current = int(abs(float(Torque)) * 1800) & 0xFFFF
        return cls._convert_to_candata_PVT(Position, Velocity, current)

    def read_motor(self, CANDATA: Optional[Sequence[int]]) -> bool:
        if CANDATA is None:
            return False
        data = list(CANDATA[:8])
        if len(data) < 8:
            return False

        if self._mode < 0 and self.get_motor_mode(data):
            return True

        self.recv_num += 1
        if (data[0] & 0x0F) != self._id:
            return False

        self._ERR = (0xF0 & data[0]) >> 4
        self._position = ((data[1] << 8) | data[2]) / 65536.0 * self.max_position * 2.0 - self.max_position
        self._velocity = ((((data[3] << 4) | (data[4] >> 4)) - 2048) / 4096.0) * self.max_velocity * 2.0
        self._torque = (((((data[4] & 0x0F) << 8) | data[5]) - 2048) / 4096.0) * self.max_torque * 2.0
        self.tem_mos = int(data[6])
        self.tem_rotor = int(data[7])

        if self._ERR == 0:
            self._enable = False
        elif self._ERR == 1:
            self._enable = True
        return True

    def get_motor_mode(self, CANDATA: Optional[Sequence[int]]) -> bool:
        if CANDATA is None:
            return False
        data = list(CANDATA[:8])
        if len(data) < 8:
            data.extend([0] * (8 - len(data)))
        if (data[2] in (0x33, 0x55)) and data[3] == 10:
            self._mode = int(data[4])
            return True
        return False

    def set(self) -> None:
        if self._mode == 1:
            self._mit_command = bytearray(self._convert_to_candata_MIT(self.MIT.position_set, self.MIT.velocity_set, self.MIT.torque_set, self.MIT.kp_set, self.MIT.kd_set))
        elif self._mode == 2:
            self._pv_command = bytearray(self._convert_to_candata_PV(self.PV.position_set, self.PV.velocity_lim))
        elif self._mode == 4:
            self._pvt_command = bytearray(self._convert_to_candata_PVT_from_torque(self.PVT.position_set, int(self.PVT.velocity_lim * 100), self.PVT.torque_lim))
        else:
            self.set_empty_command()

    def set_empty_command_PV(self) -> None:
        self._pv_command = bytearray(self._convert_to_candata_PV(0.0, 0.0))
        self.PV = PVCommand()

    def set_empty_command_MIT(self) -> None:
        self._mit_command = bytearray(self._convert_to_candata_MIT(0.0, 0.0, 0.0, 0.0, 0.0))
        self.MIT = MITCommand()

    def set_empty_command_PVT(self) -> None:
        self._pvt_command = bytearray(self._convert_to_candata_PVT(0.0, 0, 0))
        self.PVT = PVTCommand()

    def set_empty_command(self) -> None:
        self.set_empty_command_PV()
        self.set_empty_command_MIT()
        self.set_empty_command_PVT()

    def set_mode(self, mode: int) -> None:
        self._mode = -int(mode)
        self.set_empty_command()
