"""
Python port of USBCANFD.cs.

Responsibilities of this module
-------------------------------
1. Open / initialize / start the ZLG USB-CANFD device.
2. Encode and transmit CAN/CANFD frames.
3. Receive motor feedback.
4. Continuously transmit cached commands.
5. Switch the operating mode of arm motors 1~6.

Safety boundary
---------------
This module does NOT define robot mechanical soft limits.  The old
``motor.angle_lim`` values from the C# project are intentionally not used as
PV mechanical limits because their coordinate meaning conflicts with the C#
Home/PV targets.

Mechanical/software motion limits are enforced by the controller/UI layer.
This module only exposes the motor protocol range through
``motor.max_position`` and rejects nothing based on physical geometry.
"""

from __future__ import annotations

from ctypes import c_int, create_string_buffer
import threading
import time
from typing import Optional, Sequence

from DMMotor import DMMotor
from zlgcan import (
    ZCAN,
    ZCAN_STATUS_OK,
    ZCAN_TYPE_CAN,
    ZCAN_TYPE_CANFD,
    ZCAN_USBCANFD_MINI,
    ZCAN_CHANNEL_INIT_CONFIG,
    ZCAN_Transmit_Data,
    ZCAN_TransmitFD_Data,
)


class USBCANFD:
    MOTOR_NUM = 6
    TOOL_NUM = 1
    SLAVE_NUM = MOTOR_NUM + TOOL_NUM
    TYPE_CANFD = 1

    MODE_MIT = 1
    MODE_PV = 2
    MODE_PVT = 4

    def __init__(self, device_index: int = 0, channel_index: int = 0):
        self._zcan = ZCAN()
        self.device_handle_ = 0
        self.channel_handle_ = 0
        self.kDeviceType = ZCAN_USBCANFD_MINI

        self.device_index_ = int(device_index)
        self.channel_index_ = int(channel_index)

        self.tool_update = 50
        self.is_open = False
        self.is_updating = False

        self.can_param_time = 200  # ms
        self.can_param = [0.0] * 7
        self.send_suc_num = 0
        self.send_err_num = 0
        self.recv_num = 0
        self.system_update = 0

        # Arm motors 1~3: DM4340, arm motors 4~6: DM4310.
        self.motors = (
            [DMMotor(i + 1, 4340, "PV") for i in range(3)]
            + [DMMotor(i + 1, 4310, "PV") for i in range(3, 6)]
        )

        # IMPORTANT:
        # Do NOT install the old C# angle_lim values as Python PV safety
        # limits.  Their coordinate definition is ambiguous and conflicts
        # with the original C# Home target.  PV software limits are defined
        # explicitly in GUIyemian_URtonggou.py in DH coordinates.

        # Tool motor 7 remains an independent actuator.  Its default mode is
        # PVT, as in the previous Python port.
        self.tools = [DMMotor(7, 3507, "PVT")]

        self.canfd_queue = [
            self._new_canfd_frame(0, b"\x00" * 8)
            for _ in range(self.SLAVE_NUM)
        ]
        for i in range(self.MOTOR_NUM):
            self.canfd_queue[i] = self._new_canfd_frame(
                self.motors[i].ID_OFFSET,
                self.motors[i].Command,
                flags=0x11,
            )

        self.canfd_send_data = self._new_canfd_frame(
            0, b"\x00" * 8, flags=0x01
        )

        # motor_mode refers ONLY to arm motors 1~6.
        self.motor_mode = [m.Mode for m in self.motors]
        self.motor_lock = [False] * self.MOTOR_NUM
        self.mode_switch_flag = 0

        self.can_recv_trd: Optional[threading.Thread] = None
        self.can_send_trd: Optional[threading.Thread] = None
        self.can_param_update_trd: Optional[threading.Thread] = None
        self._lock = threading.RLock()

    @property
    def IsOpen(self) -> bool:
        return self.is_open

    @property
    def IsUpdating(self) -> bool:
        return self.is_updating

    @property
    def CanParam(self) -> list[float]:
        with self._lock:
            return list(self.can_param)

    @property
    def Mode(self) -> int:
        if self.motor_mode and all(v == self.MODE_MIT for v in self.motor_mode):
            return self.MODE_MIT
        if self.motor_mode and all(v == self.MODE_PV for v in self.motor_mode):
            return self.MODE_PV
        if self.motor_mode and all(v == self.MODE_PVT for v in self.motor_mode):
            return self.MODE_PVT
        return 0

    @staticmethod
    def _copy_data_to_ctypes(dst, src: Sequence[int], max_len: int) -> None:
        n = min(len(src), max_len)
        for i in range(max_len):
            dst[i] = int(src[i]) & 0xFF if i < n else 0

    @classmethod
    def _new_canfd_frame(
        cls,
        can_id: int,
        data: Sequence[int],
        flags: int = 0x11,
    ) -> ZCAN_TransmitFD_Data:
        frame = ZCAN_TransmitFD_Data()
        frame.transmit_type = 1
        frame.frame.can_id = int(can_id)
        frame.frame.len = 8
        frame.frame.flags = flags & 0xFF
        frame.frame._res0 = 1
        frame.frame._res1 = 0
        cls._copy_data_to_ctypes(frame.frame.data, data, 64)
        return frame

    @classmethod
    def _new_can_frame(
        cls,
        can_id: int,
        data: Sequence[int],
    ) -> ZCAN_Transmit_Data:
        frame = ZCAN_Transmit_Data()
        frame.transmit_type = 1
        frame.frame.can_id = cls.MakeCanId(can_id, 0, 0, 0)
        frame.frame.can_dlc = 8
        cls._copy_data_to_ctypes(frame.frame.data, data, 8)
        return frame

    def _set_value(self, path: str, value: str | int | bytes) -> bool:
        raw = value if isinstance(value, bytes) else str(value).encode("ascii")
        buf = create_string_buffer(raw)
        return self._zcan.ZCAN_SetValue(self.device_handle_, path, buf) == 1

    def open_device(self) -> bool:
        self.device_handle_ = self._zcan.OpenDevice(
            self.kDeviceType, self.device_index_, 0
        )
        if int(self.device_handle_) == 0:
            print(f"无法打开设备: device_index={self.device_index_}")
            return False

        info = self._zcan.GetDeviceInf(self.device_handle_)
        if info is not None:
            print(f"设备打开成功: device_index={self.device_index_}")
            print(f"  Serial  = {info.serial}")
            print(f"  HW Type = {info.hw_type}")
            print(f"  CAN Num = {info.can_num}")
        else:
            print(
                f"设备打开成功: device_index={self.device_index_}，"
                "但读取设备信息失败"
            )

        self.is_open = True
        return True

    def close_device(self) -> bool:
        self.stop_can()
        if self.is_open:
            if self._zcan.CloseDevice(self.device_handle_) == ZCAN_STATUS_OK:
                self.is_open = False
        return not self.is_open

    def init_device(self) -> bool:
        if not self.setCANFDStandard(0):
            print("设置CANFD标准失败")
            return False

        if not self.setCustomBaudrate(
            "1.0Mbps(75%),5.0Mbps(75%),(60,00000E2B,00800001)"
        ):
            print("设置波特率失败")
            return False

        config = ZCAN_CHANNEL_INIT_CONFIG()
        config.can_type = self.TYPE_CANFD
        config.config.canfd.mode = 0

        self.channel_handle_ = self._zcan.InitCAN(
            self.device_handle_, self.channel_index_, config
        )
        if int(self.channel_handle_) == 0:
            print("初始化CAN失败")
            return False

        if not self.setResistanceEnable(True):
            print("使能终端电阻失败")
            return False

        if not self.setFilter():
            print("滤波设置失败")
            return False

        if self._zcan.ClearBuffer(self.channel_handle_) != ZCAN_STATUS_OK:
            print("清空缓冲区失败")
            return False

        self._set_value(f"{self.channel_index_}/set_device_tx_echo", "0")
        return True

    def start_device(self) -> bool:
        if self._zcan.StartCAN(self.channel_handle_) != ZCAN_STATUS_OK:
            print("启动CAN失败")
            return False
        return True

    def canfd_send(self, can_id: int, data: Sequence[int]) -> bool:
        self.canfd_send_data = self._new_canfd_frame(
            can_id, data, flags=0x01
        )
        try:
            result = self._zcan.TransmitFD(
                self.channel_handle_, self.canfd_send_data, 1
            )
        except Exception:
            return False
        return result == 1

    def can_send(self, can_id: int, data: Sequence[int]) -> bool:
        can_data = self._new_can_frame(can_id, data)
        try:
            result = self._zcan.Transmit(self.channel_handle_, can_data, 1)
        except Exception:
            return False
        return result == 1

    @staticmethod
    def _mode_command(motor: DMMotor, mode: int) -> Sequence[int]:
        if mode == USBCANFD.MODE_MIT:
            return motor.set_mit_command
        if mode == USBCANFD.MODE_PV:
            return motor.set_pv_command
        if mode == USBCANFD.MODE_PVT:
            return motor.set_pvt_command
        raise ValueError(f"unsupported motor mode: {mode}")

    def _fill_send_queue(self) -> int:
        # Mode switching applies ONLY to arm motors 1~6.
        if self.mode_switch_flag in (
            self.MODE_MIT,
            self.MODE_PV,
            self.MODE_PVT,
        ):
            for i, motor in enumerate(self.motors):
                cmd = self._mode_command(motor, self.mode_switch_flag)
                self.canfd_queue[i] = self._new_canfd_frame(
                    motor.PARAM_SET_ID, cmd, flags=0x11
                )
        else:
            for i, motor in enumerate(self.motors):
                self.canfd_queue[i] = self._new_canfd_frame(
                    motor.ID_OFFSET, motor.Command, flags=0x11
                )

        # Tool remains independent and keeps its own current mode.
        for i, tool in enumerate(self.tools):
            self.canfd_queue[self.MOTOR_NUM + i] = self._new_canfd_frame(
                tool.ID_OFFSET, tool.Command, flags=0x11
            )

        return self.SLAVE_NUM

    def _canfd_queue_send_thread(self) -> None:
        self.setQueueSend()
        self.clearQueueSend()

        frame = 0
        self.system_update = 0
        FrameArray = ZCAN_TransmitFD_Data * self.SLAVE_NUM

        while self.is_updating:
            frame += 1
            self._fill_send_queue()

            if self.mode_switch_flag != 0:
                # During arm-mode switching only send the six arm config
                # frames.  The tool is not part of the arm-mode handshake.
                count = self.MOTOR_NUM
            else:
                # Preserve the old behavior: tool update is slower than arms.
                count = (
                    self.SLAVE_NUM
                    if frame % self.tool_update == 0
                    else self.MOTOR_NUM
                )

            frames = FrameArray(*self.canfd_queue)

            try:
                ret = self._zcan.TransmitFD(
                    self.channel_handle_, frames, count
                )
            except Exception:
                ret = 0

            with self._lock:
                self.send_err_num += max(0, count - int(ret))
                self.send_suc_num += int(ret)
                self.system_update = frame

            time.sleep(0)

    def _can_param_update_thread(self) -> None:
        while self.is_updating:
            with self._lock:
                recv_before = self.recv_num
                send_suc_before = self.send_suc_num
                send_err_before = self.send_err_num
                system_before = self.system_update

            time.sleep(self.can_param_time / 1000.0)

            with self._lock:
                recv_in_time = self.recv_num - recv_before
                send_suc_in_time = self.send_suc_num - send_suc_before
                send_err_in_time = self.send_err_num - send_err_before
                system_in_time = self.system_update - system_before

                self.can_param[0] = (
                    recv_in_time / self.can_param_time * 1000.0
                )
                self.can_param[1] = (
                    (recv_in_time + send_suc_in_time + send_err_in_time)
                    * 49.4
                    / 1000.0
                    / self.can_param_time
                    * 100.0
                )
                self.can_param[2] = float(self.send_suc_num)
                self.can_param[3] = float(self.send_err_num)
                self.can_param[4] = float(self.recv_num)
                self.can_param[5] = float(self.system_update)
                self.can_param[6] = (
                    system_in_time / self.can_param_time * 1000.0
                )

        with self._lock:
            self.send_suc_num = 0
            self.send_err_num = 0
            self.recv_num = 0

    def start_can_thread(self, type: int) -> None:
        if self.is_updating:
            return

        self.is_updating = True
        recv_target = (
            self._can_receive_thread
            if type == 0
            else self._canfd_receive_thread
        )

        self.can_recv_trd = threading.Thread(
            target=recv_target,
            name=(
                "can_receive_thread"
                if type == 0
                else "canfd_receive_thread"
            ),
            daemon=True,
        )
        self.can_send_trd = threading.Thread(
            target=self._canfd_queue_send_thread,
            name="canfd_queue_send_thread",
            daemon=True,
        )
        self.can_param_update_trd = threading.Thread(
            target=self._can_param_update_thread,
            name="can_param_update_thread",
            daemon=True,
        )

        self.can_recv_trd.start()
        self.can_send_trd.start()
        self.can_param_update_trd.start()

    def _can_receive_thread(self) -> None:
        while self.is_updating:
            length = self._zcan.GetReceiveNum(
                self.channel_handle_, ZCAN_TYPE_CAN
            )
            if length > 0:
                can_data, ret = self._zcan.Receive(
                    self.channel_handle_,
                    min(int(length), 100),
                    c_int(50),
                )
                for i in range(int(ret)):
                    self._can_data_proc(can_data[i])
            else:
                time.sleep(0.0005)

    def _canfd_receive_thread(self) -> None:
        while self.is_updating:
            length = self._zcan.GetReceiveNum(
                self.channel_handle_, ZCAN_TYPE_CANFD
            )
            if length > 0:
                canfd_data, ret = self._zcan.ReceiveFD(
                    self.channel_handle_,
                    min(int(length), 100),
                    c_int(50),
                )
                for i in range(int(ret)):
                    self._canfd_data_proc(canfd_data[i])
            else:
                time.sleep(0.0005)

    @staticmethod
    def _frame_data8(frame) -> list[int]:
        return [int(frame.data[i]) for i in range(8)]

    def _canfd_data_proc(self, canfd_data) -> bool:
        with self._lock:
            self.recv_num += 1

        data = self._frame_data8(canfd_data.frame)
        can_id = int(canfd_data.frame.can_id)

        # Arm motor mode-switch acknowledgements only.
        if self.mode_switch_flag != 0:
            motor_index = (data[0] & 0x0F) - 1
            if 0 <= motor_index < self.MOTOR_NUM:
                if self.motors[motor_index].get_motor_mode(data):
                    self.motor_mode[motor_index] = (
                        self.motors[motor_index].Mode
                    )

                    if all(
                        v == self.mode_switch_flag
                        for v in self.motor_mode
                    ):
                        self.mode_switch_flag = 0
                        print("1~6号机械臂电机模式切换完成")
            return True

        if 0x11 <= can_id <= 0x16:
            motor_index = (data[0] & 0x0F) - 1
            if 0 <= motor_index < self.MOTOR_NUM:
                self.motors[motor_index].read_motor(data)
                return True

        if can_id == 0x17 and self.tools:
            self.tools[0].read_motor(data)
            return True

        if can_id == 0x31:
            return True

        return False

    def _can_data_proc(self, can_data) -> None:
        can_id = int(can_data.frame.can_id)
        data = [int(can_data.frame.data[i]) for i in range(8)]

        if 0x10 <= can_id <= 0x16:
            motor_index = (data[0] & 0x0F) - 1
            if 0 <= motor_index < self.MOTOR_NUM:
                self.motors[motor_index].read_motor(data)

    def stop_can(self) -> None:
        self.is_updating = False
        for th in (
            self.can_recv_trd,
            self.can_send_trd,
            self.can_param_update_trd,
        ):
            if th is not None and th.is_alive():
                th.join(timeout=1.0)

    def _wait_for_rec(self, timeout_ms: float) -> Optional[bytes]:
        deadline = time.perf_counter() + timeout_ms / 1000.0

        while time.perf_counter() <= deadline:
            length = self._zcan.GetReceiveNum(
                self.channel_handle_, ZCAN_TYPE_CAN
            )
            if length > 0:
                can_data, ret = self._zcan.Receive(
                    self.channel_handle_, 100, c_int(50)
                )
                if ret > 0:
                    data = bytes(
                        int(can_data[0].frame.data[i]) & 0xFF
                        for i in range(8)
                    )
                    return data + bytes(
                        [int(can_data[0].frame.can_id) & 0xFF]
                    )

            length = self._zcan.GetReceiveNum(
                self.channel_handle_, ZCAN_TYPE_CANFD
            )
            if length > 0:
                canfd_data, ret = self._zcan.ReceiveFD(
                    self.channel_handle_, 100, c_int(50)
                )
                if ret > 0:
                    data = bytes(
                        int(canfd_data[0].frame.data[i]) & 0xFF
                        for i in range(8)
                    )
                    return data + bytes(
                        [int(canfd_data[0].frame.can_id) & 0xFF]
                    )

            time.sleep(0.0005)

        return None

    def send_wait(
        self,
        type: int,
        motor_id: int,
        send_data: Sequence[int],
        timeout: float,
    ) -> Optional[bytes]:
        self.stop_can()
        self.clearRecvBuffer()

        if type == 0:
            self.can_send(motor_id, send_data)
        else:
            self.canfd_send(motor_id, send_data)

        return self._wait_for_rec(timeout)

    def enable_all(self) -> bool:
        self.delayms(5)
        self.stop_can()

        for motor in self.motors + self.tools:
            data = self.send_wait(
                1, motor.ID, DMMotor.clear_error_command, 50
            )
            if not motor.read_motor(data):
                print(f"电机 {motor.ID} 清错无有效回复")
                return False

            data = self.send_wait(
                1, motor.ID, DMMotor.enable_command, 50
            )
            if not motor.read_motor(data):
                print(f"电机 {motor.ID} 使能无有效回复")
                return False

            if not motor.Enable:
                print(
                    f"电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}"
                )
                return False

        return True

    def disable_all(self) -> None:
        self.stop_can()

        for motor in self.motors + self.tools:
            data = self.send_wait(
                1, motor.ID, DMMotor.disable_command, 20
            )
            motor.read_motor(data)

    def set_zero(self, id: int) -> None:
        self.stop_can()

        data = self.send_wait(1, id, DMMotor.set_zero_command, 20)

        if id > self.MOTOR_NUM:
            tool = self.tools[id - self.MOTOR_NUM - 1]
            tool.read_motor(data)
            tool.set_empty_command()
            data = self.send_wait(1, id, tool.Command, 20)
            tool.read_motor(data)
        else:
            motor = self.motors[id - 1]
            motor.read_motor(data)
            motor.set_empty_command()
            data = self.send_wait(1, id, motor.Command, 20)
            motor.read_motor(data)

    def get_status_all(self) -> bool:
        for i in range(self.MOTOR_NUM):
            self.motor_mode[i] = 0

        for i, motor in enumerate(self.motors):
            data = self.send_wait(
                1, 0x7FF, motor.get_mode_command, 20
            )
            if motor.get_motor_mode(data):
                self.motor_mode[i] = motor.Mode
            else:
                return False

        if not all(v == self.motor_mode[0] for v in self.motor_mode):
            return False

        for motor in self.motors:
            motor.set_empty_command()
            data = self.send_wait(1, motor.ID, motor.Command, 20)
            if data is None:
                return False
            motor.read_motor(data)

        # Tool mode/status is independent; do not require it to equal arm mode.
        for tool in self.tools:
            data = self.send_wait(
                1, 0x7FF, tool.get_mode_command, 20
            )
            tool.get_motor_mode(data)

            tool.set_empty_command()
            data = self.send_wait(1, tool.ID, tool.Command, 20)
            if data is not None:
                tool.read_motor(data)

        return True

    def set_mode_all(self, mode: int) -> bool:
        """Switch arm motors 1~6 to MIT/PV/PVT while leaving tool mode alone."""
        if mode not in (
            self.MODE_MIT,
            self.MODE_PV,
            self.MODE_PVT,
        ):
            return False

        self.stop_can()

        for i, motor in enumerate(self.motors):
            cmd = self._mode_command(motor, mode)
            data = self.send_wait(1, 0x7FF, cmd, 100)

            if data is None or len(data) < 8:
                print(f"电机 {motor.ID} 切换模式无有效回复")
                return False

            if not motor.get_motor_mode(data):
                print(f"电机 {motor.ID} 模式回复解析失败")
                return False

            if motor.Mode != mode:
                print(
                    f"电机 {motor.ID} 模式切换失败，"
                    f"当前={motor.Mode}，目标={mode}"
                )
                return False

            self.motor_mode[i] = motor.Mode

        return True

    @staticmethod
    def delayms(time_ms: float) -> float:
        if time_ms == 0:
            return 0.0

        start = time.perf_counter()
        deadline = start + time_ms / 1000.0
        while time.perf_counter() < deadline:
            pass
        return (time.perf_counter() - start) * 1000.0

    def clearRecvBuffer(self) -> bool:
        return (
            self._zcan.ClearBuffer(self.channel_handle_)
            == ZCAN_STATUS_OK
        )

    def setQueueSend(self) -> None:
        self._set_value(
            f"{self.channel_index_}/set_send_mode", "1"
        )

    def clearQueueSend(self) -> None:
        self._set_value(
            f"{self.channel_index_}/clear_delay_send_queue", "0"
        )

    @staticmethod
    def MakeCanId(id: int, eff: int, rtr: int, err: int) -> int:
        return (
            int(id)
            | ((1 if eff else 0) << 31)
            | ((1 if rtr else 0) << 30)
            | ((1 if err else 0) << 29)
        )

    def setCANFDStandard(self, canfd_standard: int) -> bool:
        return self._set_value(
            f"{self.channel_index_}/canfd_standard",
            str(canfd_standard),
        )

    def setFdBaudrate(self, abaud: int, dbaud: int) -> bool:
        if not self._set_value(
            f"{self.channel_index_}/canfd_abit_baud_rate",
            str(abaud),
        ):
            return False

        return self._set_value(
            f"{self.channel_index_}/canfd_dbit_baud_rate",
            str(dbaud),
        )

    def setResistanceEnable(self, enable: bool) -> bool:
        return self._set_value(
            f"{self.channel_index_}/initenal_resistance",
            "1" if enable else "0",
        )

    def setCustomBaudrate(self, ABIT: str) -> bool:
        return self._set_value(
            f"{self.channel_index_}/baud_rate_custom",
            ABIT,
        )

    def setFilter(self) -> bool:
        return self._set_value(
            f"{self.channel_index_}/filter_clear", "0"
        )
