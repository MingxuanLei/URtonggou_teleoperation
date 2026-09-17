"""
UR5e 同构遥操作、示教记录与示教回放的控制逻辑模块。

本文件只负责设备通信、主端电机控制、UR5e RTDE、夹爪控制、遥操作、
示教记录和示教回放，不创建 GUI。界面由 GUI_takeover_stable.py 提供。

功能：
1. 主端 UR 同构小机械臂 1~6 轴 PV 准备对齐、MIT 重力补偿遥操作。
2. 主端第7电机 0~1 rad 映射 CTAG2F90D 夹爪 0~1 开度。
3. UR5e 关节角 1:1 遥操作。
4. 遥操作期间记录 UR5e actual_q 与夹爪 actual_open，保存为 NPZ。
5. 示教回放：UR5e 与主端共享记录轨迹；UR5e 用 servoJ，主端用 MIT 位置跟随。
6. 回放期间第7电机以 MIT 跟随记录文件中的夹爪开度。
7. 回放人工介入采用“控制权抢占”：一旦确认介入，Replay立即失去servoJ发送权，直接切换实时遥操作。

依赖：DMMotor.py, Robot.py, USBCANFD.py, GripperController.py, zlgcan.py, zlgcan.dll
"""

from __future__ import annotations

import math
import os
import time
import threading
from typing import List, Optional, Sequence, Tuple
from enum import Enum
from collections import deque

# zlgcan.py 在 Windows 下使用 ./zlgcan.dll，确保工作目录为脚本所在目录。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR:
    os.chdir(SCRIPT_DIR)

import numpy as np

from PySide6.QtCore import QObject, Signal
from DMMotor import DMMotor
from Robot import Robot
from USBCANFD import USBCANFD
from GripperController import GripperController

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None

__all__ = [
    "TeleopCoordinator",
    "MasterArmController",
    "UR5eController",
    "GripperTeleopController",
    "Robot",
    "UR_DEFAULT_IP",
    "GRIPPER_TCP_PORT",
    "ARM_DOF",
    "TOTAL_MOTOR_NUM",
    "PREP_PV_VEL_DEFAULT",
    "PREP_TOL_DEFAULT",
    "PREP_TIMEOUT_DEFAULT",
    "GRIPPER_FILTER_ALPHA",
    "GRIPPER_SEND_DEADBAND",
    "GRIPPER_TARGET_PERIOD_S",
    "REPLAY_MASTER_KP_DEFAULT",
    "REPLAY_MASTER_KD_DEFAULT",
    "REPLAY_TOOL_KP_DEFAULT",
    "REPLAY_TOOL_KD_DEFAULT",
    "RECORD_DIR_NAME",
    "TAKEOVER_ENABLED_DEFAULT",
    "TAKEOVER_BASE_ERROR_RAD",
    "TAKEOVER_PERSIST_S",
    "TAKEOVER_FAST_ERROR_RAD",
    "TAKEOVER_FAST_ACTUAL_VEL_RAD_S",
    "TAKEOVER_FAST_PERSIST_S",
    "TAKEOVER_TOOL_BASE_ERROR_RAD",
    "ControlMode",
    "SCRIPT_DIR",
]


# ============================================================
# 全局参数
# ============================================================
UR_DEFAULT_IP = "192.168.3.15"
GRIPPER_TCP_PORT = 54321
GRIPPER_SLAVE_ID = 1

ARM_DOF = 6
TOTAL_MOTOR_NUM = 7

MODE_MIT = 1
MODE_PV = 2
MODE_PVT = 4

MODE_NAME = {
    MODE_MIT: "MIT",
    MODE_PV: "PV",
    MODE_PVT: "PVT",
}

# 与现有 GUIyemian_URtonggou.py 保持一致。
GRAVITY_TORQUE_SCALE = [0.0, 1.1, 1.1, 1.2, 1.1, 0.0]
GRAVITY_PERIOD_S = 0.001

# 准备模式：主端 PV 移动参数。
PREP_PV_VEL_DEFAULT = 0.30       # rad/s
PREP_TOL_DEFAULT = 0.035         # rad，约 2°
PREP_STABLE_TIME_S = 0.50
PREP_TIMEOUT_DEFAULT = 40.0      # s

# 主端软件工作区（项目约定，不代表机械硬限位）。
MASTER_DH_MIN = -math.pi
MASTER_DH_MAX = math.pi

# UR5e 遥操作控制。
# Control 保持 125 Hz 用于 servoJ；Receive 单独降到 50 Hz，
# 避免本机在较高 RTDE Receive 频率下出现连接不稳定。
RTDE_CONTROL_FREQUENCY_HZ = 125.0
RTDE_RECEIVE_FREQUENCY_HZ = 50.0
TELEOP_PERIOD_S = 1.0 / RTDE_CONTROL_FREQUENCY_HZ
TELEOP_SERVO_LOOKAHEAD = 0.10
TELEOP_SERVO_GAIN = 300
TELEOP_MAX_MASTER_STEP = 0.20    # 单周期异常跳变保护，rad
TELEOP_START_ALIGN_TOL = 0.10

# 第7电机 -> 夹爪开度。
TOOL7_CLOSED_POS_RAD = 0.0
TOOL7_OPEN_POS_RAD = 1.0
TOOL7_INPUT_MIN_GUARD = -0.25    # 超出此范围认为反馈/机械输入异常
TOOL7_INPUT_MAX_GUARD = 1.25
GRIPPER_FILTER_ALPHA = 0.25      # 越大越跟手，越小越平滑
GRIPPER_SEND_DEADBAND = 0.01     # 开度变化小于1%不重复更新目标
GRIPPER_TARGET_PERIOD_S = 0.04   # 最快25Hz更新目标
GRIPPER_CONTROL_INTERVAL_S = 0.05
GRIPPER_FEEDBACK_TIMEOUT_S = 3.0
GRIPPER_STALE_TIMEOUT_S = 1.5
GRIPPER_SPEED_DEFAULT = 20
GRIPPER_FORCE_DEFAULT = 25
GRIPPER_ACCEL_DEFAULT = 20
GRIPPER_DECEL_DEFAULT = 20

MODE_SWITCH_TIMEOUT_S = 3.0
POWER_ON_WAIT_S = 1.0

# 示教记录 / 回放。
RECORD_DIR_NAME = "teach_records"
REPLAY_MOVEJ_SPEED = 0.35
REPLAY_MOVEJ_ACCEL = 0.60
REPLAY_UR_ALIGN_TOL = 0.05
REPLAY_UR_ALIGN_STABLE_S = 0.20
REPLAY_UR_ALIGN_TIMEOUT_S = 30.0
REPLAY_MASTER_ALIGN_TOL = 0.05
REPLAY_MASTER_ALIGN_STABLE_S = 0.40
REPLAY_MASTER_ALIGN_TIMEOUT_S = 40.0

# 主端在回放期间使用 MIT 位置跟随 + 重力补偿前馈。
# 采用逐关节柔顺参数：靠近末端的关节明显更软，更容易被操作者人工介入。
# GUI 中的 Kp/Kd 仍作为 J1/J2 的“基准值”，其余关节按下面的默认比例缩放。
REPLAY_MASTER_KP_DEFAULT = 8.0
REPLAY_MASTER_KD_DEFAULT = 0.30
REPLAY_MASTER_KP_PROFILE_DEFAULT = np.asarray([8.0, 8.0, 7.0, 4.0, 3.5, 3.0], dtype=float)
REPLAY_MASTER_KD_PROFILE_DEFAULT = np.asarray([0.30, 0.30, 0.25, 0.15, 0.12, 0.10], dtype=float)
REPLAY_TOOL_KP_DEFAULT = 2.0
REPLAY_TOOL_KD_DEFAULT = 0.10
REPLAY_MAX_FOLLOW_STEP = 0.20

# =========================
# 独立人工介入检测器（200 Hz）
# =========================
# 回放线程仍负责 UR5e / 夹爪 / MIT 参考目标；Detector 只负责“人有没有动主端”。
# J1~J6 直接使用达妙电机 Velocity / Robot.ratio；M7 直接使用 tool.Velocity。
TAKEOVER_ENABLED_DEFAULT = True
TAKEOVER_DETECT_HZ = 200.0

# 参考速度不再使用相邻两帧直接差分：
# 使用约5个记录周期的窗口求平均速度，再做低通；若速度明显不合理，则本周期禁止介入判定。
TAKEOVER_REF_VEL_WINDOW_FRAMES = 5
TAKEOVER_REF_VEL_MIN_DT_S = 0.020
TAKEOVER_REF_VEL_ALPHA = 0.25
TAKEOVER_REF_VEL_SANITY_MAX_RAD_S = 2.0
TAKEOVER_TOOL_REF_VEL_SANITY_MAX_RAD_S = 2.0

# 武装：自动跟随先稳定一小段时间，避免刚从 PV 切 MIT 的瞬态误触发。
TAKEOVER_ARM_STABLE_TIME_S = 0.30
TAKEOVER_ARM_ERROR_RAD = 0.08
TAKEOVER_TOOL_ARM_ERROR_RAD = 0.10

# SLOW：较慢但持续地由主端自身运动把误差扩大。
TAKEOVER_BASE_ERROR_RAD = 0.075
TAKEOVER_SPEED_GAIN_S = 0.12
TAKEOVER_MAX_ERROR_RAD = 0.18
TAKEOVER_MIN_ACTUAL_VEL_RAD_S = 0.04
TAKEOVER_PERSIST_S = 0.12
TAKEOVER_TOOL_BASE_ERROR_RAD = 0.060
TAKEOVER_TOOL_SPEED_GAIN_S = 0.10
TAKEOVER_TOOL_MAX_ERROR_RAD = 0.14
TAKEOVER_TOOL_MIN_ACTUAL_VEL_RAD_S = 0.04

# FAST：快速短促推动。除了误差/速度门槛，还要求误差正在扩大，
# 并且主端是反向运动或明显比参考运动得更快，正常“落后后追赶”不会触发。
TAKEOVER_FAST_ERROR_RAD = 0.040
TAKEOVER_FAST_ACTUAL_VEL_RAD_S = 0.35
TAKEOVER_FAST_REL_VEL_RAD_S = 0.25
TAKEOVER_FAST_EXCESS_VEL_RAD_S = 0.25
TAKEOVER_FAST_PERSIST_S = 0.040
TAKEOVER_TOOL_FAST_ERROR_RAD = 0.030
TAKEOVER_TOOL_FAST_ACTUAL_VEL_RAD_S = 0.24
TAKEOVER_TOOL_FAST_REL_VEL_RAD_S = 0.18
TAKEOVER_TOOL_FAST_EXCESS_VEL_RAD_S = 0.18

# OPPOSITE：实际运动方向与回放参考方向相反。
TAKEOVER_OPPOSITE_ERROR_RAD = 0.035
TAKEOVER_OPPOSITE_ACTUAL_VEL_RAD_S = 0.25
TAKEOVER_OPPOSITE_REF_VEL_RAD_S = 0.060
TAKEOVER_OPPOSITE_PERSIST_S = 0.040
TAKEOVER_TOOL_OPPOSITE_ERROR_RAD = 0.025
TAKEOVER_TOOL_OPPOSITE_ACTUAL_VEL_RAD_S = 0.18
TAKEOVER_TOOL_OPPOSITE_REF_VEL_RAD_S = 0.045

# STATIONARY_PUSH：参考轨迹几乎静止，但主端突然明显运动。
TAKEOVER_STATIONARY_REF_VEL_RAD_S = 0.050
TAKEOVER_STATIONARY_ACTUAL_VEL_RAD_S = 0.28
TAKEOVER_STATIONARY_ERROR_RAD = 0.030
TAKEOVER_STATIONARY_PERSIST_S = 0.040
TAKEOVER_TOOL_STATIONARY_REF_VEL_RAD_S = 0.040
TAKEOVER_TOOL_STATIONARY_ACTUAL_VEL_RAD_S = 0.20
TAKEOVER_TOOL_STATIONARY_ERROR_RAD = 0.022


class ControlMode(Enum):
    """UR5e 外部控制权所有者。TAKEOVER 是极短暂的抢占/交接状态。"""
    IDLE = "IDLE"
    TELEOP = "TELEOP"
    REPLAY = "REPLAY"
    TAKEOVER = "TAKEOVER"


# ============================================================
# 主端控制器
# ============================================================
class MasterArmController(QObject):
    log_signal = Signal(str)

    def __init__(self, device_index: int = 0, channel_index: int = 0):
        super().__init__()
        self.device_index = int(device_index)
        self.channel_index = int(channel_index)

        self.can: Optional[USBCANFD] = None
        self.robot: Optional[Robot] = None
        self.initialized = False
        self.current_mode: Optional[int] = None

        self.command_lock = threading.RLock()
        self.data_lock = threading.RLock()

        self.gravity_stop_event = threading.Event()
        self.gravity_thread: Optional[threading.Thread] = None

        # 回放阶段的 MIT 主端跟随状态。
        # gravity_loop 会统一生成 MIT 指令：
        #   正常遥操作时 -> Kp/Kd=0，仅重力补偿；
        #   回放跟随时   -> 位置目标 + Kp/Kd + 重力补偿。
        self.mit_follow_enabled = False
        self.mit_follow_motor_targets: Optional[List[float]] = None
        self.mit_follow_tool_target: Optional[float] = None
        self.mit_follow_kp = REPLAY_MASTER_KP_DEFAULT
        self.mit_follow_kd = REPLAY_MASTER_KD_DEFAULT
        self.mit_follow_tool_kp = REPLAY_TOOL_KP_DEFAULT
        self.mit_follow_tool_kd = REPLAY_TOOL_KD_DEFAULT

    def log(self, msg: str):
        self.log_signal.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    @staticmethod
    def _pack_command_for_mode(motor: DMMotor, mode: int) -> None:
        if mode == MODE_MIT:
            motor._mit_command = bytearray(
                motor._convert_to_candata_MIT(
                    motor.MIT.position_set,
                    motor.MIT.velocity_set,
                    motor.MIT.torque_set,
                    motor.MIT.kp_set,
                    motor.MIT.kd_set,
                )
            )
        elif mode == MODE_PV:
            motor._pv_command = bytearray(
                motor._convert_to_candata_PV(
                    motor.PV.position_set,
                    motor.PV.velocity_lim,
                )
            )
        elif mode == MODE_PVT:
            motor._pvt_command = bytearray(
                motor._convert_to_candata_PVT_from_torque(
                    motor.PVT.position_set,
                    int(motor.PVT.velocity_lim * 100),
                    motor.PVT.torque_lim,
                )
            )

    def _all_actuators(self) -> List[DMMotor]:
        if self.can is None:
            return []
        return list(self.can.motors) + list(self.can.tools)

    def _clear_error_and_enable(self, motor: DMMotor) -> bool:
        assert self.can is not None

        data = self.can.send_wait(1, motor.ID, DMMotor.clear_error_command, 100)
        if not motor.read_motor(data):
            self.log(f"[ERR] 电机 {motor.ID} 清错无有效回复")
            return False

        data = self.can.send_wait(1, motor.ID, DMMotor.enable_command, 100)
        if not motor.read_motor(data):
            self.log(f"[ERR] 电机 {motor.ID} 使能无有效回复")
            return False

        if not motor.Enable:
            self.log(f"[ERR] 电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
            return False

        self.log(f"[OK] 电机 {motor.ID} 已使能")
        return True

    def _enable_all_before_thread(self) -> bool:
        assert self.can is not None
        self.can.stop_can()
        self.can.clearRecvBuffer()

        for motor in self._all_actuators():
            if not self._clear_error_and_enable(motor):
                return False
        return True

    def _switch_tool_mode_before_thread(self, target_mode: int) -> bool:
        """在 CAN 连续线程启动前单独切换第7号工具电机模式。"""
        assert self.can is not None
        if not self.can.tools:
            self.log("[ERR] 未检测到第7号工具电机")
            return False

        tool = self.can.tools[0]
        if target_mode == MODE_MIT:
            cmd = tool.set_mit_command
        elif target_mode == MODE_PV:
            cmd = tool.set_pv_command
        elif target_mode == MODE_PVT:
            cmd = tool.set_pvt_command
        else:
            return False

        data = self.can.send_wait(1, DMMotor.PARAM_SET_ID, cmd, 100)
        if data is None or len(data) < 8:
            self.log("[ERR] 第7号工具电机模式切换无有效回复")
            return False
        if not tool.get_motor_mode(data):
            self.log("[ERR] 第7号工具电机模式回复解析失败")
            return False
        if tool.Mode != target_mode:
            self.log(
                f"[ERR] 第7号工具电机模式切换失败，"
                f"当前={tool.ModeName}，目标={MODE_NAME[target_mode]}"
            )
            return False

        self.log(f"[OK] 第7号工具电机已切换到 {MODE_NAME[target_mode]}")
        return True

    def _set_initial_commands(self) -> None:
        """1~6 当前 PV 保持；7号 MIT 零力矩。"""
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PV.position_set = float(motor.Position)
                motor.PV.velocity_lim = PREP_PV_VEL_DEFAULT
                self._pack_command_for_mode(motor, MODE_PV)

            if self.can.tools:
                tool = self.can.tools[0]
                tool.MIT.position_set = 0.0
                tool.MIT.velocity_set = 0.0
                tool.MIT.kp_set = 0.0
                tool.MIT.kd_set = 0.0
                tool.MIT.torque_set = 0.0
                self._pack_command_for_mode(tool, MODE_MIT)

    def initialize(self) -> bool:
        with self.command_lock:
            if self.initialized:
                self.log("[INFO] 主端已经初始化")
                return True

            self.can = USBCANFD(
                device_index=self.device_index,
                channel_index=self.channel_index,
            )
            self.robot = Robot()

            self.log("[1] 打开 ZLG CANFD 设备...")
            if not self.can.open_device():
                self.log("[ERR] CANFD 设备打开失败")
                return False

            self.log("[2] 初始化 CANFD...")
            if not self.can.init_device():
                self.can.close_device()
                self.log("[ERR] CANFD 初始化失败")
                return False

            self.log("[3] 启动 CANFD 通道...")
            if not self.can.start_device():
                self.can.close_device()
                self.log("[ERR] CANFD 通道启动失败")
                return False

            time.sleep(POWER_ON_WAIT_S)
            self.can.clearRecvBuffer()

            self.log("[4] 清错并使能主端 1~7 号电机...")
            if not self._enable_all_before_thread():
                self.can.close_device()
                return False

            self.log("[5] 主端 1~6 号切换到 PV（准备模式）...")
            if not self.can.set_mode_all(MODE_PV):
                self.log("[ERR] 主端 1~6 号切换 PV 失败")
                self.can.close_device()
                return False
            self.current_mode = MODE_PV

            self.log("[6] 第7号工具电机切换到 MIT 零力矩输入模式...")
            if not self._switch_tool_mode_before_thread(MODE_MIT):
                self.can.close_device()
                return False

            self.log("[7] 设置安全初始命令...")
            self._set_initial_commands()

            self.log("[8] 启动 CANFD 连续收发线程...")
            self.can.start_can_thread(1)

            if not self.wait_feedback(2.5):
                self.log("[ERR] 未建立完整的 1~7 号电机反馈")
                self.cleanup()
                return False

            arm_modes = [m.Mode for m in self.can.motors]
            self.current_mode = arm_modes[0] if arm_modes and all(
                x == arm_modes[0] for x in arm_modes
            ) else None

            self.gravity_stop_event.clear()
            self.gravity_thread = threading.Thread(
                target=self._gravity_loop,
                name="master_gravity_comp",
                daemon=True,
            )
            self.gravity_thread.start()

            self.initialized = True
            self.log("[OK] 主端初始化完成")
            self.log("[INFO] 1~6号：PV准备 / MIT重力补偿遥操作")
            self.log("[INFO] 7号：MIT零力矩，pos 0~1 rad 作为夹爪输入")
            return True

    def wait_feedback(self, timeout_s: float) -> bool:
        assert self.can is not None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            arm_ok = all(m.recv_num > 0 for m in self.can.motors)
            tool_ok = bool(self.can.tools) and self.can.tools[0].recv_num > 0
            if arm_ok and tool_ok:
                return True
            time.sleep(0.005)
        self.log(
            "[WARN] 等待反馈超时，arm recv="
            + str([m.recv_num for m in self.can.motors])
            + ", tool recv="
            + str([m.recv_num for m in self.can.tools])
        )
        return False

    def get_dh_q(self) -> Optional[List[float]]:
        if not self.initialized or self.can is None or self.robot is None:
            return None
        try:
            with self.data_lock:
                return [float(x) for x in self.robot.motor2dh(self.can.motors)]
        except Exception as e:
            self.log(f"[WARN] 读取主端 DH 角失败: {e}")
            return None

    def get_tool_position(self) -> Optional[float]:
        if not self.initialized or self.can is None or not self.can.tools:
            return None
        try:
            with self.data_lock:
                return float(self.can.tools[0].Position)
        except Exception as e:
            self.log(f"[WARN] 读取第7号工具电机位置失败: {e}")
            return None

    def get_dh_velocity(self) -> Optional[List[float]]:
        """直接使用达妙电机速度反馈换算 DH 关节速度，供 200 Hz 接管检测。"""
        if not self.initialized or self.can is None or self.robot is None:
            return None
        try:
            with self.data_lock:
                motor_vel = np.asarray([float(m.Velocity) for m in self.can.motors], dtype=float)
                ratio = np.asarray(self.robot.ratio, dtype=float).reshape(ARM_DOF)
                ratio = np.where(np.abs(ratio) < 1.0e-12, 1.0, ratio)
                return [float(x) for x in (motor_vel / ratio)]
        except Exception as e:
            self.log(f"[WARN] 读取主端DH速度失败: {e}")
            return None

    def get_tool_velocity(self) -> Optional[float]:
        """直接读取第7号达妙工具电机速度反馈。"""
        if not self.initialized or self.can is None or not self.can.tools:
            return None
        try:
            with self.data_lock:
                return float(self.can.tools[0].Velocity)
        except Exception as e:
            self.log(f"[WARN] 读取第7号工具电机速度失败: {e}")
            return None

    def get_motor_snapshot(self) -> Optional[List[dict]]:
        if not self.initialized or self.can is None:
            return None
        with self.data_lock:
            result = []
            for m in self._all_actuators():
                result.append(
                    {
                        "id": m.ID,
                        "mode": m.ModeName,
                        "enable": m.Enable,
                        "err": m.ERRCODE,
                        "pos": float(m.Position),
                        "vel": float(m.Velocity),
                        "recv": int(m.recv_num),
                    }
                )
            return result

    def _switch_arm_mode(self, target_mode: int) -> bool:
        assert self.can is not None

        if target_mode not in (MODE_MIT, MODE_PV, MODE_PVT):
            return False

        modes = [m.Mode for m in self.can.motors]
        if modes and all(m == target_mode for m in modes):
            self.current_mode = target_mode
            return True

        self.can.motor_mode = [0] * ARM_DOF
        self.can.mode_switch_flag = target_mode
        deadline = time.time() + MODE_SWITCH_TIMEOUT_S

        while time.time() < deadline:
            modes = [m.Mode for m in self.can.motors]
            if (
                self.can.mode_switch_flag == 0
                and modes
                and all(m == target_mode for m in modes)
            ):
                self.current_mode = target_mode
                self.log(f"[OK] 主端 1~6 号已切换到 {MODE_NAME[target_mode]}")
                return True
            time.sleep(0.01)

        self.log(
            f"[ERR] 主端切换到 {MODE_NAME[target_mode]} 超时，"
            f"modes={[m.Mode for m in self.can.motors]}"
        )
        self.can.mode_switch_flag = 0
        return False

    def set_pv_target_dh(self, target_dh: Sequence[float], vel: float) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None or self.robot is None:
                self.log("[ERR] 主端未初始化")
                return False

            if len(target_dh) != ARM_DOF:
                self.log("[ERR] PV 目标必须包含 6 个关节角")
                return False

            target = [float(x) for x in target_dh]
            vel = float(vel)

            if not (0.0 < vel <= 1.0):
                self.log("[ERR] PV 速度限制要求 0 < vel <= 1.0 rad/s")
                return False

            for i, q in enumerate(target):
                if not math.isfinite(q):
                    self.log(f"[ERR] J{i + 1} 目标不是有限数")
                    return False
                if not (MASTER_DH_MIN <= q <= MASTER_DH_MAX):
                    self.log(
                        f"[ERR] J{i + 1}={q:.4f} 超出当前主端软件工作区 [-pi, pi]"
                    )
                    return False

            ok, motor_targets, valid = self.robot.dh2motor(self.can.motors, target)
            if not ok or not all(valid):
                self.log("[ERR] DH -> 电机角转换失败")
                return False

            with self.data_lock:
                for motor, q_motor in zip(self.can.motors, motor_targets):
                    if abs(float(q_motor)) > float(motor.max_position):
                        self.log(
                            f"[ERR] M{motor.ID} 目标 {q_motor:.4f} rad 超出电机协议编码范围"
                        )
                        return False
                    motor.PV.position_set = float(q_motor)
                    motor.PV.velocity_lim = vel
                    self._pack_command_for_mode(motor, MODE_PV)

            if not self._switch_arm_mode(MODE_PV):
                return False

            self.log("[PV] 主端目标 DH(rad): " + str([round(x, 4) for x in target]))
            return True

    def hold_current_in_pv(self, vel: float = PREP_PV_VEL_DEFAULT) -> bool:
        q = self.get_dh_q()
        if q is None:
            return False
        return self.set_pv_target_dh(q, vel)

    def switch_to_mit_gravity(self) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None:
                return False
            self.disable_mit_follow()
            self.log("[MODE] 主端 1~6 切换 MIT + 重力补偿，接受人工拖动")
            return self._switch_arm_mode(MODE_MIT)

    def _dh_to_motor_targets_for_mit(
        self,
        target_dh: Sequence[float],
    ) -> Optional[List[float]]:
        """MIT 跟随使用的 DH -> 电机目标转换，不做 [-pi, pi] 强制裁剪。"""
        if self.can is None or self.robot is None or len(target_dh) != ARM_DOF:
            return None

        target = [float(x) for x in target_dh]
        if any(not math.isfinite(x) for x in target):
            return None

        ok, motor_targets, valid = self.robot.dh2motor(self.can.motors, target)
        if not ok or not all(valid):
            return None

        for motor, q_motor in zip(self.can.motors, motor_targets):
            if abs(float(q_motor)) > float(motor.max_position):
                self.log(
                    f"[ERR] MIT跟随目标 M{motor.ID}={float(q_motor):.4f} "
                    "超出电机协议编码范围"
                )
                return None
        return [float(x) for x in motor_targets]

    def enable_mit_follow(
        self,
        target_dh: Sequence[float],
        tool_target_rad: Optional[float],
        kp: float,
        kd: float,
        tool_kp: float,
        tool_kd: float,
    ) -> bool:
        """回放模式：主端 1~6 以 MIT 位置阻抗方式跟随，7号可同步夹爪开度。"""
        with self.command_lock:
            if not self.initialized or self.can is None:
                return False

            motor_targets = self._dh_to_motor_targets_for_mit(target_dh)
            if motor_targets is None:
                return False

            kp = max(0.0, min(500.0, float(kp)))
            kd = max(0.0, min(5.0, float(kd)))
            # GUI 参数作为 J1/J2 基准值，其余关节保持柔顺比例。
            kp_scale = kp / max(float(REPLAY_MASTER_KP_DEFAULT), 1.0e-9)
            kd_scale = kd / max(float(REPLAY_MASTER_KD_DEFAULT), 1.0e-9)
            kp_profile = np.clip(REPLAY_MASTER_KP_PROFILE_DEFAULT * kp_scale, 0.0, 500.0)
            kd_profile = np.clip(REPLAY_MASTER_KD_PROFILE_DEFAULT * kd_scale, 0.0, 5.0)
            tool_kp = max(0.0, min(500.0, float(tool_kp)))
            tool_kd = max(0.0, min(5.0, float(tool_kd)))

            tool_target = None
            if tool_target_rad is not None:
                tool_target = max(
                    TOOL7_CLOSED_POS_RAD,
                    min(TOOL7_OPEN_POS_RAD, float(tool_target_rad)),
                )

            with self.data_lock:
                self.mit_follow_motor_targets = motor_targets
                self.mit_follow_tool_target = tool_target
                self.mit_follow_kp = kp_profile.copy()
                self.mit_follow_kd = kd_profile.copy()
                self.mit_follow_tool_kp = tool_kp
                self.mit_follow_tool_kd = tool_kd
                self.mit_follow_enabled = True

            if not self._switch_arm_mode(MODE_MIT):
                self.disable_mit_follow()
                return False

            self.log(
                "[MIT FOLLOW] 已启用逐关节柔顺MIT跟随："
                f"Kp={[round(float(x),2) for x in kp_profile]}, "
                f"Kd={[round(float(x),2) for x in kd_profile]}, "
                f"ToolKp={tool_kp:.2f}, ToolKd={tool_kd:.2f}"
            )
            return True

    def update_mit_follow_target(
        self,
        target_dh: Sequence[float],
        tool_target_rad: Optional[float] = None,
    ) -> bool:
        """更新回放期间的 MIT 跟随目标；模式切换只在 enable 时做一次。"""
        if not self.initialized or self.can is None:
            return False

        motor_targets = self._dh_to_motor_targets_for_mit(target_dh)
        if motor_targets is None:
            return False

        tool_target = None
        if tool_target_rad is not None:
            tool_target = max(
                TOOL7_CLOSED_POS_RAD,
                min(TOOL7_OPEN_POS_RAD, float(tool_target_rad)),
            )

        with self.data_lock:
            if not self.mit_follow_enabled:
                return False
            self.mit_follow_motor_targets = motor_targets
            self.mit_follow_tool_target = tool_target
        return True

    def disable_mit_follow(self):
        with self.data_lock:
            self.mit_follow_enabled = False
            self.mit_follow_motor_targets = None
            self.mit_follow_tool_target = None

    def _gravity_loop(self):
        assert self.can is not None
        assert self.robot is not None

        self.log("[GRAVITY] 主端重力补偿计算线程启动")
        while not self.gravity_stop_event.is_set():
            try:
                with self.data_lock:
                    self.robot.Angle = self.robot.motor2dh(self.can.motors)
                    if not self.robot.set_robot():
                        time.sleep(GRAVITY_PERIOD_S)
                        continue

                    tau_g_motor = self.robot.Tau_G_Motor
                    follow = (
                        self.mit_follow_enabled
                        and self.mit_follow_motor_targets is not None
                        and len(self.mit_follow_motor_targets) == ARM_DOF
                    )

                    for i, motor in enumerate(self.can.motors):
                        if follow:
                            motor.MIT.position_set = float(
                                self.mit_follow_motor_targets[i]
                            )
                            motor.MIT.velocity_set = 0.0
                            kp_arr = np.asarray(self.mit_follow_kp, dtype=float).reshape(-1)
                            kd_arr = np.asarray(self.mit_follow_kd, dtype=float).reshape(-1)
                            motor.MIT.kp_set = float(kp_arr[i] if kp_arr.size > 1 else kp_arr[0])
                            motor.MIT.kd_set = float(kd_arr[i] if kd_arr.size > 1 else kd_arr[0])
                        else:
                            motor.MIT.position_set = 0.0
                            motor.MIT.velocity_set = 0.0
                            motor.MIT.kp_set = 0.0
                            motor.MIT.kd_set = 0.0

                        # 两种 MIT 状态都叠加原有重力补偿前馈。
                        motor.MIT.torque_set = float(
                            tau_g_motor[i] * GRAVITY_TORQUE_SCALE[i]
                        )
                        self._pack_command_for_mode(motor, MODE_MIT)

                    # 第7号：遥操作时为MIT零力矩输入；回放时可用MIT位置跟随夹爪。
                    if self.can.tools:
                        tool = self.can.tools[0]
                        if follow and self.mit_follow_tool_target is not None:
                            tool.MIT.position_set = float(
                                self.mit_follow_tool_target
                            )
                            tool.MIT.velocity_set = 0.0
                            tool.MIT.kp_set = float(self.mit_follow_tool_kp)
                            tool.MIT.kd_set = float(self.mit_follow_tool_kd)
                            tool.MIT.torque_set = 0.0
                        else:
                            tool.MIT.position_set = 0.0
                            tool.MIT.velocity_set = 0.0
                            tool.MIT.kp_set = 0.0
                            tool.MIT.kd_set = 0.0
                            tool.MIT.torque_set = 0.0
                        self._pack_command_for_mode(tool, MODE_MIT)
            except Exception as e:
                self.log(f"[WARN] 重力补偿计算异常: {e}")
                time.sleep(0.01)

            time.sleep(GRAVITY_PERIOD_S)

        self.log("[GRAVITY] 主端重力补偿计算线程退出")

    def cleanup(self):
        with self.command_lock:
            if self.can is None:
                return

            self.disable_mit_follow()
            self.gravity_stop_event.set()
            if self.gravity_thread is not None and self.gravity_thread.is_alive():
                self.gravity_thread.join(timeout=1.0)

            try:
                self.can.stop_can()
            except Exception:
                pass

            try:
                for motor in self._all_actuators():
                    data = self.can.send_wait(1, motor.ID, DMMotor.disable_command, 50)
                    motor.read_motor(data)
            except Exception as e:
                self.log(f"[WARN] 主端失能异常: {e}")

            try:
                self.can.close_device()
            except Exception as e:
                self.log(f"[WARN] 关闭 CANFD 异常: {e}")

            self.initialized = False
            self.current_mode = None
            self.log("[END] 主端已停止并关闭")


# ============================================================
# UR5e RTDE 控制器
# ============================================================
class UR5eController(QObject):
    log_signal = Signal(str)

    def __init__(self, ip: str = UR_DEFAULT_IP):
        super().__init__()
        self.ip = ip
        self.rtde_c = None
        self.rtde_r = None
        self.connected = False
        self.lock = threading.RLock()

    def log(self, msg: str):
        self.log_signal.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def connect(self, ip: Optional[str] = None) -> bool:
        with self.lock:
            if rtde_control is None or rtde_receive is None:
                self.log("[ERR] 未安装 ur-rtde，请执行: pip install ur-rtde")
                return False

            if ip:
                self.ip = str(ip).strip()

            if self.connected:
                self.log("[INFO] UR5e 已连接")
                return True

            try:
                self.log(f"[UR] 连接 UR5e: {self.ip} ...")
                self.rtde_r = rtde_receive.RTDEReceiveInterface(
                    self.ip, RTDE_RECEIVE_FREQUENCY_HZ
                )
                self.rtde_c = rtde_control.RTDEControlInterface(
                    self.ip, RTDE_CONTROL_FREQUENCY_HZ
                )
                q = self.rtde_r.getActualQ()
                if q is None or len(q) != ARM_DOF:
                    raise RuntimeError("getActualQ() 返回无效")
                self.connected = True
                self.log(
                    f"[OK] UR5e RTDE 已连接：Control={RTDE_CONTROL_FREQUENCY_HZ:.0f}Hz, "
                    f"Receive={RTDE_RECEIVE_FREQUENCY_HZ:.0f}Hz"
                )
                return True
            except Exception as e:
                self.rtde_r = None
                self.rtde_c = None
                self.connected = False
                self.log(f"[ERR] UR5e 连接失败: {e}")
                return False

    def get_actual_q(self) -> Optional[List[float]]:
        with self.lock:
            if not self.connected or self.rtde_r is None:
                return None
            try:
                q = self.rtde_r.getActualQ()
                if q is None or len(q) != ARM_DOF:
                    return None
                return [float(x) for x in q]
            except Exception as e:
                self.log(f"[WARN] 读取 UR5e 关节角失败: {e}")
                return None

    def servo_j(self, q: Sequence[float], dt: float) -> bool:
        if not self.connected or self.rtde_c is None:
            return False
        try:
            return bool(
                self.rtde_c.servoJ(
                    [float(x) for x in q],
                    0.0,
                    0.0,
                    float(dt),
                    TELEOP_SERVO_LOOKAHEAD,
                    TELEOP_SERVO_GAIN,
                )
            )
        except Exception as e:
            self.log(f"[ERR] servoJ 异常: {e}")
            return False

    def move_j_async(
        self,
        q: Sequence[float],
        speed: float = REPLAY_MOVEJ_SPEED,
        acceleration: float = REPLAY_MOVEJ_ACCEL,
    ) -> bool:
        """回放前异步启动 moveJ；到位等待由协调器完成，因此停止按钮仍可响应。"""
        if not self.connected or self.rtde_c is None:
            return False
        try:
            result = self.rtde_c.moveJ(
                [float(x) for x in q],
                float(speed),
                float(acceleration),
                True,
            )
            return bool(result)
        except Exception as e:
            self.log(f"[ERR] 异步 moveJ 到回放起点异常: {e}")
            return False

    def stop_j(self, deceleration: float = 2.0):
        if self.rtde_c is None:
            return
        try:
            self.rtde_c.stopJ(float(deceleration))
        except Exception as e:
            self.log(f"[WARN] stopJ 异常: {e}")

    def servo_stop(self):
        if self.rtde_c is None:
            return
        try:
            self.rtde_c.servoStop()
        except Exception as e:
            self.log(f"[WARN] servoStop 异常: {e}")

    def disconnect(self):
        with self.lock:
            self.servo_stop()

            if self.rtde_c is not None:
                try:
                    if hasattr(self.rtde_c, "disconnect"):
                        self.rtde_c.disconnect()
                except Exception:
                    pass

            if self.rtde_r is not None:
                try:
                    if hasattr(self.rtde_r, "disconnect"):
                        self.rtde_r.disconnect()
                except Exception:
                    pass

            self.rtde_c = None
            self.rtde_r = None
            self.connected = False
            self.log("[UR] RTDE 已断开")


# ============================================================
# 夹爪控制器封装
# ============================================================
class GripperTeleopController(QObject):
    log_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.gripper: Optional[GripperController] = None
        self.connected = False
        self.port = f"{UR_DEFAULT_IP}:{GRIPPER_TCP_PORT}"
        self.last_feedback_time = 0.0
        self.last_command_opening: Optional[float] = None
        self.lock = threading.RLock()

    def log(self, msg: str):
        self.log_signal.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _feedback_callback(self, status, position, speed, current):
        self.last_feedback_time = time.monotonic()

    def connect(self, robot_ip: str) -> bool:
        with self.lock:
            if self.connected and self.gripper is not None:
                self.log("[INFO] 夹爪已经连接")
                return True

            self.port = f"{str(robot_ip).strip()}:{GRIPPER_TCP_PORT}"
            try:
                self.log(f"[GRIPPER] 连接 {self.port} ...")
                g = GripperController(
                    port=self.port,
                    slave_id=GRIPPER_SLAVE_ID,
                    connection_type="tcp",
                    timeout=0.5,
                    debug=False,
                )
                g.on_feedback(self._feedback_callback)
                self.last_feedback_time = 0.0
                g.start(interval=GRIPPER_CONTROL_INTERVAL_S)

                deadline = time.monotonic() + GRIPPER_FEEDBACK_TIMEOUT_S
                while time.monotonic() < deadline:
                    if not g.is_running:
                        raise RuntimeError("夹爪后台控制线程意外停止")
                    if self.last_feedback_time > 0.0:
                        break
                    time.sleep(0.02)
                else:
                    raise TimeoutError(
                        f"{GRIPPER_FEEDBACK_TIMEOUT_S:.1f}s 内没有收到夹爪反馈；"
                        "请检查54321端口、Tool Communication Forwarder、RS485接线和供电"
                    )

                self.gripper = g
                self.connected = True
                self.last_command_opening = None
                self.log(
                    f"[OK] 夹爪已连接，当前开度={float(g.feedback.open):.3f}"
                )
                return True
            except Exception as e:
                try:
                    if 'g' in locals():
                        g.close()
                except Exception:
                    pass
                self.gripper = None
                self.connected = False
                self.log(f"[ERR] 夹爪连接失败: {e}")
                return False

    def feedback_is_fresh(self) -> bool:
        if not self.connected or self.gripper is None:
            return False
        if not self.gripper.is_running:
            return False
        if self.last_feedback_time <= 0.0:
            return False
        return (time.monotonic() - self.last_feedback_time) <= GRIPPER_STALE_TIMEOUT_S

    def get_actual_opening(self) -> Optional[float]:
        if not self.connected or self.gripper is None:
            return None
        try:
            return float(self.gripper.feedback.open)
        except Exception:
            return None

    def command_opening(self, opening: float) -> bool:
        """非阻塞更新夹爪目标；不等待到位。"""
        with self.lock:
            if not self.connected or self.gripper is None:
                return False

            opening = max(0.0, min(1.0, float(opening)))
            try:
                self.gripper.set_motion_params(
                    position=opening,
                    speed=GRIPPER_SPEED_DEFAULT,
                    force=GRIPPER_FORCE_DEFAULT,
                    accel=GRIPPER_ACCEL_DEFAULT,
                    decel=GRIPPER_DECEL_DEFAULT,
                )
                self.last_command_opening = opening
                return True
            except Exception as e:
                self.log(f"[ERR] 更新夹爪目标失败: {e}")
                return False

    def disconnect(self):
        with self.lock:
            if self.gripper is not None:
                try:
                    self.gripper.close()
                except Exception as e:
                    self.log(f"[WARN] 关闭夹爪通信异常: {e}")
            self.gripper = None
            self.connected = False
            self.last_command_opening = None
            self.last_feedback_time = 0.0
            self.log("[GRIPPER] 夹爪通信已断开")


# ============================================================
# 准备 + 遥操作协调器
# ============================================================
class TeleopCoordinator(QObject):
    log_signal = Signal(str)
    state_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.master = MasterArmController()
        self.ur = UR5eController()
        self.gripper = GripperTeleopController()

        self.master.log_signal.connect(self.log_signal.emit)
        self.ur.log_signal.connect(self.log_signal.emit)
        self.gripper.log_signal.connect(self.log_signal.emit)

        self.preparation_done = False
        self.prep_target_master_q: Optional[List[float]] = None
        self.last_ur_q: Optional[List[float]] = None

        self.teleop_stop_event = threading.Event()
        self.teleop_thread: Optional[threading.Thread] = None
        self.teleop_running = False
        self.lock = threading.RLock()

        # ---------- UR5e 控制权 ----------
        # 任意时刻只有一个控制源拥有 servoJ 发送权。
        # 人工接管一旦确认：REPLAY -> TAKEOVER -> TELEOP。
        self.control_lock = threading.RLock()
        self.control_mode = ControlMode.IDLE
        self.takeover_event = threading.Event()

        self.latest_tool_raw_opening: Optional[float] = None
        self.latest_tool_filtered_opening: Optional[float] = None

        # ---------- 遥操作示教记录 ----------
        self.record_lock = threading.RLock()
        self.recording = False
        self.record_start_mono = 0.0
        self.record_frames = []
        self.last_record_path: Optional[str] = None

        # 当前已加载的示教轨迹。回放使用 actual_q + actual gripper opening。
        self.loaded_trajectory: Optional[dict] = None
        self.loaded_trajectory_path: Optional[str] = None

        # ---------- 示教回放 ----------
        self.replay_stop_event = threading.Event()
        self.replay_thread: Optional[threading.Thread] = None
        self.replay_running = False

        # 回放人工介入 / 接管状态。接管后采用主从绝对关节角1:1映射。
        self.takeover_enabled = TAKEOVER_ENABLED_DEFAULT
        self.takeover_armed = False
        self.takeover_triggered = False
        self.takeover_last_reason = ""

        # 独立 200 Hz 人工接管检测器共享状态。
        self.takeover_detector_stop_event = threading.Event()
        self.takeover_detector_thread: Optional[threading.Thread] = None
        self.takeover_ref_lock = threading.RLock()
        self.takeover_ref_master: Optional[np.ndarray] = None
        self.takeover_ref_tool: Optional[float] = None
        # Replay 线程直接提供参考速度，避免 200Hz Detector 对 125Hz
        # 阶梯位置参考做差分而产生“假高速/假静止”。
        self.takeover_ref_master_vel = np.zeros(ARM_DOF, dtype=float)
        self.takeover_ref_tool_vel = 0.0
        self.takeover_ref_vel_valid = False
        self.takeover_ref_vel_max_abs = 0.0

    def log(self, msg: str):
        self.log_signal.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _set_control_mode(self, mode: ControlMode):
        with self.control_lock:
            self.control_mode = mode

    def get_control_mode_name(self) -> str:
        with self.control_lock:
            return self.control_mode.value

    def _send_replay_servo_j(self, q: Sequence[float]) -> bool:
        """仅 REPLAY 控制权有效时允许发送回放 servoJ。"""
        with self.control_lock:
            if self.control_mode is not ControlMode.REPLAY:
                return False
            if self.takeover_event.is_set():
                return False
            return self.ur.servo_j(q, TELEOP_PERIOD_S)

    def _send_teleop_servo_j(self, q: Sequence[float]) -> bool:
        """仅 TELEOP 控制权有效时允许发送人工遥操作 servoJ。"""
        with self.control_lock:
            if self.control_mode is not ControlMode.TELEOP:
                return False
            return self.ur.servo_j(q, TELEOP_PERIOD_S)

    @staticmethod
    def ur_q_to_master_equivalent(ur_q: Sequence[float]) -> List[float]:
        return [Robot.angle_clip_pnpi(float(q)) for q in ur_q]

    @staticmethod
    def joint_error(current: Sequence[float], target: Sequence[float]) -> List[float]:
        return [
            float(Robot.minor_arc_dir(float(c), float(t)))
            for c, t in zip(current, target)
        ]

    @staticmethod
    def tool7_to_opening(tool_pos_rad: float) -> float:
        """0 rad -> 0闭合，1 rad -> 1张开；中间线性映射并限幅。"""
        span = TOOL7_OPEN_POS_RAD - TOOL7_CLOSED_POS_RAD
        if span <= 0.0:
            raise RuntimeError("第7轴标定范围无效")
        opening = (float(tool_pos_rad) - TOOL7_CLOSED_POS_RAD) / span
        return max(0.0, min(1.0, opening))

    def read_ur_configuration(self) -> Optional[Tuple[List[float], List[float]]]:
        q_ur = self.ur.get_actual_q()
        if q_ur is None:
            self.log("[ERR] 无法读取 UR5e 当前关节角")
            return None

        q_master_equiv = self.ur_q_to_master_equivalent(q_ur)
        self.last_ur_q = q_ur
        self.prep_target_master_q = q_master_equiv
        self.preparation_done = False

        self.log("[UR] ActualQ(rad): " + str([round(x, 4) for x in q_ur]))
        self.log(
            "[PREP] 主端等价目标(rad): "
            + str([round(x, 4) for x in q_master_equiv])
        )
        return q_ur, q_master_equiv

    def run_preparation(self, pv_velocity: float, tolerance: float, timeout_s: float) -> bool:
        with self.lock:
            with self.control_lock:
                if self.control_mode is ControlMode.TAKEOVER:
                    self.log("[ERR] 人工接管正在交接控制权，暂不能进入准备模式")
                    return False
            if self.teleop_running:
                self.log("[ERR] 遥操作运行中，不能进入准备模式")
                return False
            if self.replay_running:
                self.log("[ERR] 示教回放运行中，不能进入准备模式")
                return False
            if not self.master.initialized:
                self.log("[ERR] 请先初始化主端")
                return False
            if not self.ur.connected:
                self.log("[ERR] 请先连接 UR5e")
                return False

            readout = self.read_ur_configuration()
            if readout is None:
                return False

            _, target_master_q = readout
            self.state_signal.emit("准备模式：主端 PV 对齐中")
            self.log("[PREP] 主端 1~6 号以 PV 移动到 UR5e 当前构型")
            self.log("[PREP] 第7号保持 MIT 零力矩，可人工拨动，不影响准备过程")

            if not self.master.set_pv_target_dh(target_master_q, pv_velocity):
                self.state_signal.emit("准备模式失败")
                return False

        deadline = time.time() + float(timeout_s)
        stable_since: Optional[float] = None

        while time.time() < deadline:
            q_now = self.master.get_dh_q()
            if q_now is None:
                time.sleep(0.02)
                continue

            err = self.joint_error(q_now, target_master_q)
            max_err = max(abs(x) for x in err)

            if max_err <= tolerance:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= PREP_STABLE_TIME_S:
                    with self.lock:
                        self.preparation_done = True
                    self.log(
                        f"[OK] 准备完成：主端与 UR5e 构型误差 max={max_err:.4f} rad"
                    )
                    self.state_signal.emit("准备完成：可以开始7轴遥操作")
                    return True
            else:
                stable_since = None

            time.sleep(0.02)

        q_now = self.master.get_dh_q()
        if q_now is not None:
            err = self.joint_error(q_now, target_master_q)
            self.log(
                "[ERR] 准备模式超时，各关节误差(rad): "
                + str([round(x, 4) for x in err])
            )
        self.preparation_done = False
        self.state_signal.emit("准备模式超时/失败")
        return False

    def start_teleop(self) -> bool:
        with self.lock:
            with self.control_lock:
                if self.control_mode is ControlMode.TAKEOVER:
                    self.log("[ERR] 人工接管正在交接控制权，请勿重复启动遥操作")
                    return False
            if self.teleop_running:
                self.log("[INFO] 遥操作已经运行")
                return True
            if self.replay_running:
                self.log("[ERR] 示教回放运行中，不能启动遥操作")
                return False
            if not self.preparation_done or self.prep_target_master_q is None:
                self.log("[ERR] 请先完成准备模式")
                return False
            if not self.master.initialized or not self.ur.connected:
                self.log("[ERR] 主端或 UR5e 未连接")
                return False
            if not self.gripper.connected:
                self.log("[ERR] 请先连接末端夹爪")
                return False
            if not self.gripper.feedback_is_fresh():
                self.log("[ERR] 夹爪反馈已失效，拒绝启动遥操作")
                return False

            q_master = self.master.get_dh_q()
            q_ur = self.ur.get_actual_q()
            q7 = self.master.get_tool_position()
            if q_master is None or q_ur is None or q7 is None:
                self.log("[ERR] 启动遥操作前无法读取完整主/从状态")
                return False

            if not (TOOL7_INPUT_MIN_GUARD <= q7 <= TOOL7_INPUT_MAX_GUARD):
                self.log(
                    f"[ERR] 第7号电机 pos={q7:.4f} rad 超出允许输入保护范围 "
                    f"[{TOOL7_INPUT_MIN_GUARD:.2f}, {TOOL7_INPUT_MAX_GUARD:.2f}]"
                )
                return False

            prep_err = self.joint_error(q_master, self.prep_target_master_q)
            if max(abs(x) for x in prep_err) > TELEOP_START_ALIGN_TOL:
                self.log(
                    "[ERR] 主端已经离开准备构型，请重新执行准备模式；err="
                    + str([round(x, 4) for x in prep_err])
                )
                self.preparation_done = False
                return False

            if not self.master.switch_to_mit_gravity():
                self.log("[ERR] 主端 1~6 切换 MIT + 重力补偿失败")
                return False

            time.sleep(0.05)
            q_master_start = self.master.get_dh_q()
            q_ur_start = self.ur.get_actual_q()
            q7_start = self.master.get_tool_position()
            if q_master_start is None or q_ur_start is None or q7_start is None:
                self.log("[ERR] 无法建立遥操作起始状态")
                # 已切到MIT但启动失败时，立即切回PV锁住当前位置。
                self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                return False

            initial_opening = self.tool7_to_opening(q7_start)
            if not self.gripper.command_opening(initial_opening):
                self.log("[ERR] 无法同步夹爪到第7号电机当前开度")
                self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                return False

            self.latest_tool_raw_opening = initial_opening
            self.latest_tool_filtered_opening = initial_opening

            self.teleop_stop_event.clear()
            self._set_control_mode(ControlMode.TELEOP)
            self.teleop_running = True
            self.state_signal.emit("遥操作中：1~6轴1:1 + 7号夹爪")

            self.teleop_thread = threading.Thread(
                target=self._teleop_loop,
                args=(q_master_start, q_ur_start, initial_opening),
                name="ur5e_joint_gripper_teleop",
                daemon=True,
            )
            self.teleop_thread.start()

            self.log("[TELEOP] 已启动 1~6 轴关节角 1:1 遥操作")
            self.log(
                f"[TELEOP] 7号工具电机绝对映射夹爪："
                f"0rad=闭合，1rad=张开，启动开度={initial_opening:.3f}"
            )
            return True

    def _update_takeover_reference(
        self,
        master_ref: Sequence[float],
        tool_ref: float,
        master_ref_vel: Optional[Sequence[float]] = None,
        tool_ref_vel: float = 0.0,
        ref_vel_valid: bool = True,
    ):
        """
        由 Replay 线程更新 Detector 的共享参考。

        位置和速度都由 Replay 时间轴直接提供。Detector 不再自行对 125Hz
        阶梯位置参考求差分，从而避免把正常自动跟随误判成 FAST 或
        STATIONARY_PUSH。
        """
        with self.takeover_ref_lock:
            self.takeover_ref_master = np.asarray(
                master_ref, dtype=float
            ).reshape(ARM_DOF).copy()
            self.takeover_ref_tool = float(tool_ref)
            if master_ref_vel is None:
                self.takeover_ref_master_vel = np.zeros(ARM_DOF, dtype=float)
            else:
                self.takeover_ref_master_vel = np.asarray(
                    master_ref_vel, dtype=float
                ).reshape(ARM_DOF).copy()
            self.takeover_ref_tool_vel = float(tool_ref_vel)
            self.takeover_ref_vel_valid = bool(ref_vel_valid)
            arm_max = float(np.max(np.abs(self.takeover_ref_master_vel))) if self.takeover_ref_master_vel.size else 0.0
            self.takeover_ref_vel_max_abs = max(arm_max, abs(self.takeover_ref_tool_vel))

    def _stop_takeover_detector(self):
        self.takeover_detector_stop_event.set()
        th = self.takeover_detector_thread
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=0.8)
        self.takeover_detector_thread = None

    def _start_takeover_detector(self):
        """独立 200 Hz 检测 J1~J6 + M7，直接使用达妙 Velocity 反馈。"""
        self._stop_takeover_detector()
        if not self.takeover_enabled:
            return
        self.takeover_detector_stop_event.clear()
        self.takeover_armed = False

        def worker():
            period = 1.0 / max(float(TAKEOVER_DETECT_HZ), 1.0)
            stable_since = fast_since = opposite_since = stationary_since = slow_since = None
            last_diag = 0.0
            last_ref_warn = 0.0
            self.log(
                f"[TAKEOVER] 独立检测线程启动：{TAKEOVER_DETECT_HZ:.0f}Hz，"
                "直接使用达妙Velocity；FAST/OPPOSITE/STATIONARY_PUSH/SLOW 四通道"
            )
            try:
                while not self.takeover_detector_stop_event.is_set():
                    loop_t = time.perf_counter()
                    if self.replay_stop_event.is_set() or self.takeover_event.is_set():
                        break
                    with self.control_lock:
                        if self.control_mode is not ControlMode.REPLAY:
                            break
                    with self.takeover_ref_lock:
                        ref6 = (
                            None
                            if self.takeover_ref_master is None
                            else self.takeover_ref_master.copy()
                        )
                        ref7 = self.takeover_ref_tool
                        refv6 = self.takeover_ref_master_vel.copy()
                        refv7 = float(self.takeover_ref_tool_vel)
                        ref_vel_valid = bool(self.takeover_ref_vel_valid)
                        ref_vel_max_abs = float(self.takeover_ref_vel_max_abs)
                    if ref6 is None or ref7 is None:
                        time.sleep(period)
                        continue

                    q6 = self.master.get_dh_q()
                    q7 = self.master.get_tool_position()
                    v6 = self.master.get_dh_velocity()
                    v7 = self.master.get_tool_velocity()
                    if q6 is None or q7 is None or v6 is None or v7 is None:
                        time.sleep(period)
                        continue

                    now = time.perf_counter()

                    actual6 = np.asarray(q6, dtype=float)
                    actualv6 = np.asarray(v6, dtype=float)
                    err6 = np.asarray(
                        [
                            Robot.minor_arc_dir(float(r), float(a))
                            for r, a in zip(ref6, actual6)
                        ],
                        dtype=float,
                    )
                    err7 = float(q7) - float(ref7)
                    err = np.concatenate([err6, [err7]])
                    actualv = np.concatenate([actualv6, [float(v7)]])
                    refv = np.concatenate([
                        np.asarray(refv6, dtype=float).reshape(ARM_DOF),
                        [float(refv7)],
                    ])
                    relv = actualv - refv
                    abserr = np.abs(err)

                    if not self.takeover_armed:
                        if float(np.max(abserr[:ARM_DOF])) <= TAKEOVER_ARM_ERROR_RAD and abserr[ARM_DOF] <= TAKEOVER_TOOL_ARM_ERROR_RAD:
                            if stable_since is None: stable_since = now
                            elif now - stable_since >= TAKEOVER_ARM_STABLE_TIME_S:
                                self.takeover_armed = True
                                fast_since = opposite_since = stationary_since = slow_since = None
                                self.log(f"[TAKEOVER] 人工介入检测已武装：稳定{TAKEOVER_ARM_STABLE_TIME_S:.2f}s，J1~J6 + M7")
                        else:
                            stable_since = None
                    else:
                        # 参考速度窗口尚未建立，或Replay检测到时间戳/速度异常时，
                        # 本周期不允许触发人工接管。这样异常 ref_vel 不会放大 rel_vel。
                        if not ref_vel_valid:
                            fast_since = opposite_since = stationary_since = slow_since = None
                            if now - last_ref_warn >= 1.0:
                                last_ref_warn = now
                                self.log(
                                    f"[TAKEOVER] 参考速度暂不可用/异常(max={ref_vel_max_abs:.3f}rad/s)，"
                                    "本周期跳过人工介入判定"
                                )
                            elapsed = time.perf_counter() - loop_t
                            time.sleep(max(0.0, period - elapsed))
                            continue

                        base = np.asarray([TAKEOVER_BASE_ERROR_RAD]*ARM_DOF + [TAKEOVER_TOOL_BASE_ERROR_RAD])
                        gain = np.asarray([TAKEOVER_SPEED_GAIN_S]*ARM_DOF + [TAKEOVER_TOOL_SPEED_GAIN_S])
                        mx = np.asarray([TAKEOVER_MAX_ERROR_RAD]*ARM_DOF + [TAKEOVER_TOOL_MAX_ERROR_RAD])
                        minv = np.asarray([TAKEOVER_MIN_ACTUAL_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_MIN_ACTUAL_VEL_RAD_S])
                        dynamic = np.minimum(mx, base + gain*np.abs(refv))

                        # err = q_master - q_ref，relv = v_master - v_ref = d(err)/dt。
                        # err*relv > 0：误差幅值正在扩大；<0：主端正在追赶参考。
                        diverging = (err * relv) > 0.0
                        worsening = diverging & (np.abs(actualv) >= minv)

                        fast_err = np.asarray([TAKEOVER_FAST_ERROR_RAD]*ARM_DOF + [TAKEOVER_TOOL_FAST_ERROR_RAD])
                        fast_av = np.asarray([TAKEOVER_FAST_ACTUAL_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_FAST_ACTUAL_VEL_RAD_S])
                        fast_rv = np.asarray([TAKEOVER_FAST_REL_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_FAST_REL_VEL_RAD_S])
                        fast_excess = np.asarray([TAKEOVER_FAST_EXCESS_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_FAST_EXCESS_VEL_RAD_S])

                        # FAST 只允许两类明显主动动作：
                        # 1) 与参考运动方向相反；
                        # 2) 与参考同方向，但主端速度明显超过参考速度。
                        fast_opposite = (actualv * refv) < 0.0
                        fast_overspeed = np.abs(actualv) >= (np.abs(refv) + fast_excess)
                        fast_mask = (
                            (abserr >= fast_err)
                            & (np.abs(actualv) >= fast_av)
                            & (np.abs(relv) >= fast_rv)
                            & diverging
                            & (fast_opposite | fast_overspeed)
                        )

                        opp_err = np.asarray([TAKEOVER_OPPOSITE_ERROR_RAD]*ARM_DOF + [TAKEOVER_TOOL_OPPOSITE_ERROR_RAD])
                        opp_av = np.asarray([TAKEOVER_OPPOSITE_ACTUAL_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_OPPOSITE_ACTUAL_VEL_RAD_S])
                        opp_rv = np.asarray([TAKEOVER_OPPOSITE_REF_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_OPPOSITE_REF_VEL_RAD_S])
                        opposite_mask = (
                            (abserr >= opp_err)
                            & ((actualv * refv) < 0.0)
                            & (np.abs(actualv) >= opp_av)
                            & (np.abs(refv) >= opp_rv)
                            & diverging
                        )

                        stat_ref = np.asarray([TAKEOVER_STATIONARY_REF_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_STATIONARY_REF_VEL_RAD_S])
                        stat_av = np.asarray([TAKEOVER_STATIONARY_ACTUAL_VEL_RAD_S]*ARM_DOF + [TAKEOVER_TOOL_STATIONARY_ACTUAL_VEL_RAD_S])
                        stat_err = np.asarray([TAKEOVER_STATIONARY_ERROR_RAD]*ARM_DOF + [TAKEOVER_TOOL_STATIONARY_ERROR_RAD])
                        stationary_mask = (
                            (np.abs(refv) <= stat_ref)
                            & (np.abs(actualv) >= stat_av)
                            & (abserr >= stat_err)
                            & diverging
                        )
                        slow_mask = (abserr >= dynamic) & worsening

                        trigger_kind = None; trigger_mask = None
                        def persist(mask, since, need):
                            if bool(np.any(mask)):
                                if since is None: return now, False
                                return since, (now-since >= need)
                            return None, False
                        fast_since, hit = persist(fast_mask, fast_since, TAKEOVER_FAST_PERSIST_S)
                        if hit: trigger_kind, trigger_mask = 'FAST', fast_mask
                        if trigger_kind is None:
                            opposite_since, hit = persist(opposite_mask, opposite_since, TAKEOVER_OPPOSITE_PERSIST_S)
                            if hit: trigger_kind, trigger_mask = 'OPPOSITE', opposite_mask
                        if trigger_kind is None:
                            stationary_since, hit = persist(stationary_mask, stationary_since, TAKEOVER_STATIONARY_PERSIST_S)
                            if hit: trigger_kind, trigger_mask = 'STATIONARY_PUSH', stationary_mask
                        if trigger_kind is None:
                            slow_since, hit = persist(slow_mask, slow_since, TAKEOVER_PERSIST_S)
                            if hit: trigger_kind, trigger_mask = 'SLOW', slow_mask

                        if trigger_kind is not None:
                            axes = np.where(trigger_mask)[0]
                            axis = int(axes[np.argmax(abserr[axes])])
                            name = f"J{axis+1}" if axis < ARM_DOF else 'M7'
                            self.takeover_last_reason = (
                                f"{trigger_kind} {name}: err={err[axis]:.3f}rad, "
                                f"actual_vel={actualv[axis]:.3f}rad/s, ref_vel={refv[axis]:.3f}rad/s, "
                                f"rel_vel={relv[axis]:.3f}rad/s"
                            )
                            self.takeover_triggered = True
                            self.takeover_event.set()
                            self._set_control_mode(ControlMode.TAKEOVER)
                            self.log('[TAKEOVER] 检测到人工介入：' + self.takeover_last_reason)
                            break

                        if now - last_diag >= 0.75:
                            last_diag = now
                            self.log(
                                f"[TAKEOVER] ARMED | arm_err={float(np.max(abserr[:ARM_DOF])):.3f}rad | "
                                f"M7_err={abserr[ARM_DOF]:.3f}rad | max|v|={float(np.max(np.abs(actualv))):.3f}rad/s | "
                                f"max|ref_v|={float(np.max(np.abs(refv))):.3f}rad/s"
                            )

                    elapsed = time.perf_counter() - loop_t
                    time.sleep(max(0.0, period - elapsed))
            except Exception as e:
                self.log(f"[WARN] 人工介入检测线程异常: {e}")
            finally:
                self.log('[TAKEOVER] 独立人工介入检测线程退出')

        self.takeover_detector_thread = threading.Thread(target=worker, name='takeover_detector_200hz', daemon=True)
        self.takeover_detector_thread.start()

    @staticmethod
    def _master_absolute_to_ur_nearest(
        master_q: Sequence[float],
        ur_reference: Sequence[float],
    ) -> List[float]:
        """
        将主端 DH 关节角直接作为 UR5e 的绝对同构目标。

        主端 Robot.motor2dh() 输出位于 (-pi, pi] 附近，而 UR5e RTDE 的实际关节角
        可能处在其它 2*pi 等价分支。这里不引入任何主从零点偏置，只选择距离
        当前 UR 参考角最近的 2*pi 等价角，避免经过 +/-pi 时突然跳一整圈。
        """
        if len(master_q) != ARM_DOF or len(ur_reference) != ARM_DOF:
            raise ValueError("绝对同构映射需要6个主端角和6个UR参考角")

        result: List[float] = []
        two_pi = 2.0 * math.pi
        for qm, qr in zip(master_q, ur_reference):
            qm = float(qm)
            qr = float(qr)
            k = round((qr - qm) / two_pi)
            result.append(qm + k * two_pi)
        return result

    def _start_takeover_teleop_from_current(self) -> bool:
        """
        回放人工介入后切换到“绝对同构遥操作”。

        与普通手动启动遥操作不同，这里不重新建立 master_start / ur_start 零点：
        1) 关闭主端 MIT 自动跟随，保留 MIT 重力补偿；
        2) 从端 UR5e 的目标关节角直接由主端当前 DH 关节角决定；
        3) 仅对 +/-pi 的 2*pi 等价分支做连续化处理，不增加任何位置偏置；
        4) 第7轴继续使用 0~1 rad -> 夹爪 0~1 的绝对映射。
        """
        if not self.master.initialized or not self.ur.connected:
            self.log("[ERR] 人工接管失败：主端或UR5e未连接")
            return False
        if not self.gripper.connected or not self.gripper.feedback_is_fresh():
            self.log("[ERR] 人工接管失败：夹爪通信未就绪")
            return False

        # 退出MIT自动位置跟随，但不经过PV，直接回到MIT重力补偿的人机拖动状态。
        if not self.master.switch_to_mit_gravity():
            self.log("[ERR] 人工接管失败：主端无法退出MIT自动跟随")
            return False

        time.sleep(0.005)
        q_master_now = self.master.get_dh_q()
        q_ur_now = self.ur.get_actual_q()
        q7_now = self.master.get_tool_position()
        if q_master_now is None or q_ur_now is None or q7_now is None:
            self.log("[ERR] 人工接管失败：无法读取当前主/从状态")
            return False

        if not (TOOL7_INPUT_MIN_GUARD <= q7_now <= TOOL7_INPUT_MAX_GUARD):
            self.log(
                f"[ERR] 人工接管失败：M7={q7_now:.4f}rad超出保护范围"
            )
            return False

        # 这里只用于确定UR5e当前2*pi分支，不作为零点偏置。
        initial_ur_target = self._master_absolute_to_ur_nearest(
            q_master_now, q_ur_now
        )

        initial_opening = self.tool7_to_opening(q7_now)
        if not self.gripper.command_opening(initial_opening):
            self.log("[ERR] 人工接管失败：夹爪目标同步失败")
            return False

        self.latest_tool_raw_opening = initial_opening
        self.latest_tool_filtered_opening = initial_opening

        self.teleop_stop_event.clear()
        self._set_control_mode(ControlMode.TELEOP)
        self.teleop_running = True
        self.preparation_done = False
        self.teleop_thread = threading.Thread(
            target=self._takeover_absolute_teleop_loop,
            args=(q_master_now, initial_ur_target, initial_opening),
            name="ur5e_takeover_absolute_teleop",
            daemon=True,
        )
        self.teleop_thread.start()

        initial_err = [
            float(t - u) for t, u in zip(initial_ur_target, q_ur_now)
        ]
        self.log("[TAKEOVER] 已停止示教回放并切换到绝对同构遥操作")
        self.log(
            "[TAKEOVER] 不再重新设置1:1零点："
            "UR5e目标角 = 主端当前DH角（仅处理2pi等价分支）"
        )
        self.log(
            "[TAKEOVER] 接管瞬间主从绝对角误差(rad): "
            + str([round(x, 4) for x in initial_err])
        )
        self.log("[TAKEOVER] 第7轴恢复人工夹爪绝对控制")
        self.state_signal.emit("人工介入接管：绝对1:1遥操作中")
        return True

    def _takeover_absolute_teleop_loop(
        self,
        master_initial: List[float],
        ur_target_initial: List[float],
        initial_opening: float,
    ):
        """人工接管专用：UR5e绝对关节目标始终等于主端当前同构关节角。"""
        prev_master_wrapped = np.asarray(master_initial, dtype=float)
        prev_ur_target = np.asarray(ur_target_initial, dtype=float)

        filtered_opening = float(initial_opening)
        last_sent_opening = float(initial_opening)
        next_gripper_target_time = time.monotonic()

        next_tick = time.perf_counter()
        failure_reason: Optional[str] = None

        try:
            while not self.teleop_stop_event.is_set():
                # ---------- 1~6号：绝对同构映射 ----------
                q_master_list = self.master.get_dh_q()
                if q_master_list is None:
                    failure_reason = "主端1~6号关节反馈丢失"
                    break

                q_master = np.asarray(q_master_list, dtype=float)
                master_step = np.asarray(
                    [
                        Robot.minor_arc_dir(float(a), float(b))
                        for a, b in zip(prev_master_wrapped, q_master)
                    ],
                    dtype=float,
                )
                if np.any(~np.isfinite(master_step)):
                    failure_reason = "主端出现非有限关节数据"
                    break
                if float(np.max(np.abs(master_step))) > TELEOP_MAX_MASTER_STEP:
                    failure_reason = (
                        "检测到异常关节跳变，max_step="
                        f"{float(np.max(np.abs(master_step))):.4f} rad"
                    )
                    break

                ur_target = np.asarray(
                    self._master_absolute_to_ur_nearest(
                        q_master.tolist(), prev_ur_target.tolist()
                    ),
                    dtype=float,
                )
                if np.any(~np.isfinite(ur_target)):
                    failure_reason = "UR5e绝对同构目标出现非有限数"
                    break

                if not self._send_teleop_servo_j(ur_target.tolist()):
                    if self.teleop_stop_event.is_set():
                        break
                    failure_reason = "UR5e servoJ 返回失败或遥操作已失去控制权"
                    break

                prev_master_wrapped = q_master
                prev_ur_target = ur_target

                # ---------- 7号工具电机 -> 夹爪，保持绝对映射 ----------
                q7 = self.master.get_tool_position()
                if q7 is None or not math.isfinite(q7):
                    failure_reason = "第7号工具电机反馈丢失/无效"
                    break
                if not (TOOL7_INPUT_MIN_GUARD <= q7 <= TOOL7_INPUT_MAX_GUARD):
                    failure_reason = (
                        f"第7号工具电机输入越界：{q7:.4f} rad，"
                        f"保护范围=[{TOOL7_INPUT_MIN_GUARD:.2f}, "
                        f"{TOOL7_INPUT_MAX_GUARD:.2f}]"
                    )
                    break

                raw_opening = self.tool7_to_opening(q7)
                filtered_opening = (
                    GRIPPER_FILTER_ALPHA * raw_opening
                    + (1.0 - GRIPPER_FILTER_ALPHA) * filtered_opening
                )
                self.latest_tool_raw_opening = raw_opening
                self.latest_tool_filtered_opening = filtered_opening

                now_mono = time.monotonic()
                if now_mono >= next_gripper_target_time:
                    if abs(filtered_opening - last_sent_opening) >= GRIPPER_SEND_DEADBAND:
                        if not self.gripper.command_opening(filtered_opening):
                            failure_reason = "夹爪目标更新失败"
                            break
                        last_sent_opening = filtered_opening
                    next_gripper_target_time = now_mono + GRIPPER_TARGET_PERIOD_S

                if not self.gripper.feedback_is_fresh():
                    failure_reason = "夹爪通信反馈超时"
                    break

                # ---------- 如人工接管后又开始记录，记录绝对同构目标 ----------
                if self.recording:
                    q_ur_actual = self.ur.get_actual_q()
                    gripper_actual = self.gripper.get_actual_opening()
                    if (
                        q_ur_actual is not None
                        and gripper_actual is not None
                        and math.isfinite(float(gripper_actual))
                    ):
                        self._append_record_frame(
                            ur_actual_q=q_ur_actual,
                            gripper_actual_open=float(gripper_actual),
                            master_q=q_master.tolist(),
                            tool7_pos=float(q7),
                            ur_target_q=ur_target.tolist(),
                            gripper_target_open=float(filtered_opening),
                        )

                next_tick += TELEOP_PERIOD_S
                remain = next_tick - time.perf_counter()
                if remain > 0:
                    time.sleep(remain)
                else:
                    next_tick = time.perf_counter()

        except Exception as e:
            failure_reason = f"绝对同构遥操作线程异常: {e}"
        finally:
            if self.recording:
                try:
                    self.stop_recording(save=True)
                except Exception as record_error:
                    self.log(f"[WARN] 遥操作结束时保存记录失败: {record_error}")

            with self.control_lock:
                if self.control_mode is ControlMode.TELEOP:
                    self.ur.servo_stop()
                    self.control_mode = ControlMode.IDLE
            self.teleop_running = False

            if failure_reason:
                try:
                    if self.master.initialized:
                        self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                except Exception as hold_error:
                    self.log(f"[WARN] 异常停止后主端PV保持失败: {hold_error}")
                self.log(f"[SAFE] {failure_reason}，绝对同构遥操作已停止")
                self.state_signal.emit("绝对同构遥操作异常停止")
            else:
                self.log("[TELEOP] 人工接管绝对同构遥操作线程停止")
                self.state_signal.emit("遥操作已停止")

    def _teleop_loop(
        self,
        master_start: List[float],
        ur_start: List[float],
        initial_opening: float,
    ):
        prev_wrapped = np.array(master_start, dtype=float)
        master_cont = np.array(master_start, dtype=float)
        master_start_arr = np.array(master_start, dtype=float)
        ur_start_arr = np.array(ur_start, dtype=float)

        filtered_opening = float(initial_opening)
        last_sent_opening = float(initial_opening)
        next_gripper_target_time = time.monotonic()

        next_tick = time.perf_counter()
        failure_reason: Optional[str] = None

        try:
            while not self.teleop_stop_event.is_set():
                # ---------- 1~6号机械臂 ----------
                q_wrapped_list = self.master.get_dh_q()
                if q_wrapped_list is None:
                    failure_reason = "主端1~6号关节反馈丢失"
                    break

                q_wrapped = np.array(q_wrapped_list, dtype=float)
                step = np.array(
                    [
                        Robot.minor_arc_dir(float(a), float(b))
                        for a, b in zip(prev_wrapped, q_wrapped)
                    ],
                    dtype=float,
                )

                if np.any(~np.isfinite(step)):
                    failure_reason = "主端出现非有限关节数据"
                    break

                if float(np.max(np.abs(step))) > TELEOP_MAX_MASTER_STEP:
                    failure_reason = (
                        "检测到异常关节跳变，max_step="
                        f"{float(np.max(np.abs(step))):.4f} rad"
                    )
                    break

                master_cont += step
                prev_wrapped = q_wrapped
                ur_target = ur_start_arr + (master_cont - master_start_arr)

                if np.any(~np.isfinite(ur_target)):
                    failure_reason = "UR5e 目标出现非有限数"
                    break

                if not self._send_teleop_servo_j(ur_target.tolist()):
                    if self.teleop_stop_event.is_set():
                        break
                    failure_reason = "UR5e servoJ 返回失败或遥操作已失去控制权"
                    break

                # ---------- 7号工具电机 -> 夹爪 ----------
                q7 = self.master.get_tool_position()
                if q7 is None or not math.isfinite(q7):
                    failure_reason = "第7号工具电机反馈丢失/无效"
                    break

                if not (TOOL7_INPUT_MIN_GUARD <= q7 <= TOOL7_INPUT_MAX_GUARD):
                    failure_reason = (
                        f"第7号工具电机输入越界：{q7:.4f} rad，"
                        f"保护范围=[{TOOL7_INPUT_MIN_GUARD:.2f}, "
                        f"{TOOL7_INPUT_MAX_GUARD:.2f}]"
                    )
                    break

                raw_opening = self.tool7_to_opening(q7)
                filtered_opening = (
                    GRIPPER_FILTER_ALPHA * raw_opening
                    + (1.0 - GRIPPER_FILTER_ALPHA) * filtered_opening
                )
                self.latest_tool_raw_opening = raw_opening
                self.latest_tool_filtered_opening = filtered_opening

                now_mono = time.monotonic()
                if now_mono >= next_gripper_target_time:
                    if abs(filtered_opening - last_sent_opening) >= GRIPPER_SEND_DEADBAND:
                        if not self.gripper.command_opening(filtered_opening):
                            failure_reason = "夹爪目标更新失败"
                            break
                        last_sent_opening = filtered_opening
                    next_gripper_target_time = now_mono + GRIPPER_TARGET_PERIOD_S

                if not self.gripper.feedback_is_fresh():
                    failure_reason = "夹爪通信反馈超时"
                    break

                # ---------- 示教记录 ----------
                # 记录的是 UR5e 实际关节角和夹爪实际开度，而不是只记录命令值。
                if self.recording:
                    q_ur_actual = self.ur.get_actual_q()
                    gripper_actual = self.gripper.get_actual_opening()
                    if (
                        q_ur_actual is not None
                        and gripper_actual is not None
                        and math.isfinite(float(gripper_actual))
                    ):
                        self._append_record_frame(
                            ur_actual_q=q_ur_actual,
                            gripper_actual_open=float(gripper_actual),
                            master_q=q_wrapped.tolist(),
                            tool7_pos=float(q7),
                            ur_target_q=ur_target.tolist(),
                            gripper_target_open=float(filtered_opening),
                        )

                # ---------- 周期控制 ----------
                next_tick += TELEOP_PERIOD_S
                remain = next_tick - time.perf_counter()
                if remain > 0:
                    time.sleep(remain)
                else:
                    next_tick = time.perf_counter()

        except Exception as e:
            failure_reason = f"遥操作线程异常: {e}"
        finally:
            if self.recording:
                try:
                    self.stop_recording(save=True)
                except Exception as record_error:
                    self.log(f"[WARN] 遥操作结束时保存记录失败: {record_error}")

            # 只有当前仍由 TELEOP 持有控制权时，遥操作线程才有权 servoStop。
            # 这样未来若发生其它高优先级状态切换，不会由旧线程误停新控制源。
            with self.control_lock:
                if self.control_mode is ControlMode.TELEOP:
                    self.ur.servo_stop()
                    self.control_mode = ControlMode.IDLE
            self.teleop_running = False

            if failure_reason:
                # 异常退出时主动把主端1~6从MIT切回PV并保持当前姿态，
                # 避免操作者误以为已经停止而主端仍处于可自由拖动状态。
                try:
                    if self.master.initialized:
                        self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                except Exception as hold_error:
                    self.log(f"[WARN] 异常停止后主端PV保持失败: {hold_error}")
                self.log(f"[SAFE] {failure_reason}，遥操作已停止")
                self.state_signal.emit("遥操作异常停止")
            else:
                self.log("[TELEOP] 遥操作线程停止")
                self.state_signal.emit("遥操作已停止")

    def stop_teleop(self, hold_master: bool = True) -> bool:
        with self.lock:
            self.teleop_stop_event.set()
            th = self.teleop_thread

        if th is not None and th.is_alive():
            th.join(timeout=1.5)

        with self.control_lock:
            if self.control_mode is ControlMode.TELEOP:
                self.ur.servo_stop()
                self.control_mode = ControlMode.IDLE

        if hold_master and self.master.initialized:
            try:
                self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                self.log("[STOP] 主端1~6已切回PV并保持当前位置；7号继续MIT零力矩")
            except Exception as e:
                self.log(f"[WARN] 主端切回 PV 保持失败: {e}")

        self.teleop_running = False
        self.teleop_thread = None
        return True

    # ========================================================
    # 遥操作记录
    # ========================================================
    def start_recording(self) -> bool:
        with self.record_lock:
            if self.replay_running:
                self.log("[ERR] 回放运行中不能开始记录")
                return False
            if not self.teleop_running:
                self.log("[ERR] 只有在遥操作运行中才能开始示教记录")
                return False
            if self.recording:
                self.log("[INFO] 示教记录已经开始")
                return True

            self.record_frames = []
            self.record_start_mono = time.monotonic()
            self.recording = True
            self.state_signal.emit("遥操作 + 示教记录中")
            self.log("[RECORD] 开始记录 UR5e actual_q + 夹爪 actual_open")
            return True

    def _append_record_frame(
        self,
        ur_actual_q: Sequence[float],
        gripper_actual_open: float,
        master_q: Sequence[float],
        tool7_pos: float,
        ur_target_q: Sequence[float],
        gripper_target_open: float,
    ):
        if not self.recording:
            return

        t = time.monotonic() - self.record_start_mono
        frame = (
            float(t),
            [float(x) for x in ur_actual_q],
            float(gripper_actual_open),
            [float(x) for x in master_q],
            float(tool7_pos),
            [float(x) for x in ur_target_q],
            float(gripper_target_open),
        )
        with self.record_lock:
            if self.recording:
                self.record_frames.append(frame)

    @staticmethod
    def _validate_trajectory_arrays(
        t: np.ndarray,
        ur_q: np.ndarray,
        gripper_open: np.ndarray,
    ) -> Tuple[bool, str]:
        if t.ndim != 1:
            return False, "timestamp 必须是一维数组"
        if ur_q.ndim != 2 or ur_q.shape[1] != ARM_DOF:
            return False, "ur_q 必须是 N×6"
        if gripper_open.ndim != 1:
            return False, "gripper_open 必须是一维数组"
        n = len(t)
        if n < 2 or len(ur_q) != n or len(gripper_open) != n:
            return False, "轨迹长度无效或数组长度不一致"
        if not np.all(np.isfinite(t)):
            return False, "timestamp 包含非有限数"
        if not np.all(np.isfinite(ur_q)):
            return False, "ur_q 包含非有限数"
        if not np.all(np.isfinite(gripper_open)):
            return False, "gripper_open 包含非有限数"
        if np.any(np.diff(t) < 0.0):
            return False, "timestamp 不是单调递增"
        if np.any((gripper_open < -0.05) | (gripper_open > 1.05)):
            return False, "夹爪开度明显超出 0~1"
        return True, ""

    def stop_recording(self, save: bool = True) -> bool:
        with self.record_lock:
            if not self.recording:
                self.log("[INFO] 当前没有正在进行的示教记录")
                return False

            # 先关标志，避免保存期间遥操作线程继续 append。
            self.recording = False
            frames = list(self.record_frames)
            self.record_frames = []

        if len(frames) < 2:
            self.log("[ERR] 记录帧数不足，未生成示教文件")
            self.state_signal.emit("遥操作中" if self.teleop_running else "记录失败")
            return False

        t = np.asarray([f[0] for f in frames], dtype=np.float64)
        ur_q = np.asarray([f[1] for f in frames], dtype=np.float64)
        gripper_open = np.asarray([f[2] for f in frames], dtype=np.float64)
        master_q = np.asarray([f[3] for f in frames], dtype=np.float64)
        tool7_pos = np.asarray([f[4] for f in frames], dtype=np.float64)
        ur_target_q = np.asarray([f[5] for f in frames], dtype=np.float64)
        gripper_target_open = np.asarray([f[6] for f in frames], dtype=np.float64)

        ok, reason = self._validate_trajectory_arrays(t, ur_q, gripper_open)
        if not ok:
            self.log(f"[ERR] 记录轨迹校验失败: {reason}")
            return False

        trajectory = {
            "timestamp": t,
            "ur_q": ur_q,
            "gripper_open": np.clip(gripper_open, 0.0, 1.0),
            "master_q": master_q,
            "tool7_pos": tool7_pos,
            "ur_target_q": ur_target_q,
            "gripper_target_open": np.clip(gripper_target_open, 0.0, 1.0),
        }

        self.loaded_trajectory = trajectory
        duration = float(t[-1] - t[0])

        if save:
            record_dir = os.path.join(SCRIPT_DIR, RECORD_DIR_NAME)
            os.makedirs(record_dir, exist_ok=True)
            filename = time.strftime("teach_%Y%m%d_%H%M%S.npz")
            path = os.path.join(record_dir, filename)

            np.savez_compressed(
                path,
                timestamp=t,
                ur_q=ur_q,
                gripper_open=np.clip(gripper_open, 0.0, 1.0),
                master_q=master_q,
                tool7_pos=tool7_pos,
                ur_target_q=ur_target_q,
                gripper_target_open=np.clip(gripper_target_open, 0.0, 1.0),
                rtde_control_frequency_hz=np.asarray(
                    [RTDE_CONTROL_FREQUENCY_HZ], dtype=np.float64
                ),
                rtde_receive_frequency_hz=np.asarray(
                    [RTDE_RECEIVE_FREQUENCY_HZ], dtype=np.float64
                ),
            )
            self.last_record_path = path
            self.loaded_trajectory_path = path
            self.log(
                f"[RECORD] 已保存 {len(t)} 帧，时长 {duration:.2f}s -> {path}"
            )
        else:
            self.log(f"[RECORD] 已结束记录，共 {len(t)} 帧，时长 {duration:.2f}s")

        self.state_signal.emit("遥操作中" if self.teleop_running else "示教记录完成")
        return True

    def load_trajectory(self, path: str) -> bool:
        path = os.path.abspath(str(path))
        try:
            with np.load(path, allow_pickle=False) as data:
                required = ("timestamp", "ur_q", "gripper_open")
                for key in required:
                    if key not in data:
                        raise ValueError(f"缺少字段: {key}")

                t = np.asarray(data["timestamp"], dtype=np.float64).copy()
                ur_q = np.asarray(data["ur_q"], dtype=np.float64).copy()
                gripper_open = np.asarray(data["gripper_open"], dtype=np.float64).copy()

                ok, reason = self._validate_trajectory_arrays(t, ur_q, gripper_open)
                if not ok:
                    raise ValueError(reason)

                trajectory = {
                    "timestamp": t,
                    "ur_q": ur_q,
                    "gripper_open": np.clip(gripper_open, 0.0, 1.0),
                }

                # 以下字段是本程序自己保存的扩展数据；老文件没有也能回放。
                for key in (
                    "master_q",
                    "tool7_pos",
                    "ur_target_q",
                    "gripper_target_open",
                ):
                    if key in data:
                        trajectory[key] = np.asarray(data[key], dtype=np.float64).copy()

            self.loaded_trajectory = trajectory
            self.loaded_trajectory_path = path
            self.last_record_path = path
            self.log(
                f"[REPLAY] 已加载轨迹：{os.path.basename(path)}，"
                f"{len(t)} 帧，时长 {float(t[-1] - t[0]):.2f}s"
            )
            return True
        except Exception as e:
            self.log(f"[ERR] 加载示教轨迹失败: {e}")
            return False

    def _wait_ur_alignment(
        self,
        target_ur_q: Sequence[float],
        timeout_s: float = REPLAY_UR_ALIGN_TIMEOUT_S,
    ) -> bool:
        deadline = time.time() + float(timeout_s)
        stable_since: Optional[float] = None

        while time.time() < deadline:
            if self.replay_stop_event.is_set():
                self.ur.stop_j()
                return False

            q_now = self.ur.get_actual_q()
            if q_now is None:
                time.sleep(0.02)
                continue

            err = [
                abs(Robot.minor_arc_dir(float(c), float(t)))
                for c, t in zip(q_now, target_ur_q)
            ]
            max_err = max(err)

            if max_err <= REPLAY_UR_ALIGN_TOL:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= REPLAY_UR_ALIGN_STABLE_S:
                    return True
            else:
                stable_since = None
            time.sleep(0.02)

        self.ur.stop_j()
        return False

    def _wait_master_alignment(
        self,
        target_master_q: Sequence[float],
        timeout_s: float = REPLAY_MASTER_ALIGN_TIMEOUT_S,
    ) -> bool:
        deadline = time.time() + float(timeout_s)
        stable_since: Optional[float] = None

        while time.time() < deadline and not self.replay_stop_event.is_set():
            q_now = self.master.get_dh_q()
            if q_now is None:
                time.sleep(0.02)
                continue

            err = self.joint_error(q_now, target_master_q)
            max_err = max(abs(x) for x in err)

            if max_err <= REPLAY_MASTER_ALIGN_TOL:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= REPLAY_MASTER_ALIGN_STABLE_S:
                    return True
            else:
                stable_since = None
            time.sleep(0.02)

        return False

    # ========================================================
    # UR5e 示教回放 + 主端 MIT 跟随
    # ========================================================
    def start_replay(
        self,
        master_kp: float = REPLAY_MASTER_KP_DEFAULT,
        master_kd: float = REPLAY_MASTER_KD_DEFAULT,
        tool_kp: float = REPLAY_TOOL_KP_DEFAULT,
        tool_kd: float = REPLAY_TOOL_KD_DEFAULT,
        takeover_enabled: bool = TAKEOVER_ENABLED_DEFAULT,
    ) -> bool:
        with self.lock:
            with self.control_lock:
                if self.control_mode is ControlMode.TAKEOVER:
                    self.log("[ERR] 人工接管正在交接控制权，不能启动示教回放")
                    return False
            if self.teleop_running:
                self.log("[ERR] 请先停止遥操作，再开始示教回放")
                return False
            if self.recording:
                self.log("[ERR] 当前正在记录，不能开始回放")
                return False
            if self.replay_running:
                self.log("[INFO] 示教回放已经运行")
                return True
            if not self.master.initialized:
                self.log("[ERR] 请先初始化主端")
                return False
            if not self.ur.connected:
                self.log("[ERR] 请先连接 UR5e")
                return False
            if not self.gripper.connected or not self.gripper.feedback_is_fresh():
                self.log("[ERR] 夹爪未连接或反馈失效")
                return False
            if self.loaded_trajectory is None:
                self.log("[ERR] 没有示教轨迹；请先记录或加载 .npz 文件")
                return False

            traj = self.loaded_trajectory
            t = np.asarray(traj["timestamp"], dtype=np.float64)
            q = np.asarray(traj["ur_q"], dtype=np.float64)
            g = np.asarray(traj["gripper_open"], dtype=np.float64)
            ok, reason = self._validate_trajectory_arrays(t, q, g)
            if not ok:
                self.log(f"[ERR] 回放轨迹无效: {reason}")
                return False

            self.replay_stop_event.clear()
            self.takeover_event.clear()
            self._set_control_mode(ControlMode.REPLAY)
            self.replay_running = True
            self.preparation_done = False
            self.takeover_enabled = bool(takeover_enabled)
            self.takeover_armed = False
            self.takeover_triggered = False
            self.takeover_last_reason = ""
            self.state_signal.emit("示教回放准备中")

            self.replay_thread = threading.Thread(
                target=self._replay_loop,
                args=(
                    t.copy(),
                    q.copy(),
                    g.copy(),
                    float(master_kp),
                    float(master_kd),
                    float(tool_kp),
                    float(tool_kd),
                ),
                name="ur5e_teach_replay",
                daemon=True,
            )
            self.replay_thread.start()
            self.log("[REPLAY] 示教回放线程已启动")
            self.log(
                "[TAKEOVER] 人工介入检测："
                + ("开启" if self.takeover_enabled else "关闭")
            )
            return True

    def _replay_loop(
        self,
        timestamp: np.ndarray,
        ur_q_record: np.ndarray,
        gripper_record: np.ndarray,
        master_kp: float,
        master_kd: float,
        tool_kp: float,
        tool_kd: float,
    ):
        failure_reason: Optional[str] = None
        master_follow_enabled = False
        takeover_triggered = False

        try:
            first_q = ur_q_record[0].tolist()
            first_open = float(np.clip(gripper_record[0], 0.0, 1.0))

            # ---------- 回放准备1：UR5e 到轨迹第一帧 ----------
            self.state_signal.emit("回放准备：UR5e移动到轨迹起点")
            self.log("[REPLAY] UR5e 异步 moveJ 到示教轨迹第一帧...")
            if not self.ur.move_j_async(first_q):
                failure_reason = "UR5e 无法启动回放起点 moveJ"
                return
            if not self._wait_ur_alignment(first_q):
                if self.replay_stop_event.is_set():
                    return
                failure_reason = "UR5e 移动到回放起点超时"
                return

            # 夹爪先同步到第一帧开度。
            if not self.gripper.command_opening(first_open):
                failure_reason = "夹爪无法同步到回放起始开度"
                return
            time.sleep(0.20)

            # ---------- 回放准备2：主端先PV对齐 ----------
            # 真正播放时会改成MIT跟随；这里用PV只是避免MIT大误差突然拉动主端。
            q_ur_now = self.ur.get_actual_q()
            if q_ur_now is None:
                failure_reason = "无法读取UR5e回放起点姿态"
                return

            first_master = self.ur_q_to_master_equivalent(q_ur_now)
            self.state_signal.emit("回放准备：主端PV对齐轨迹起点")
            if not self.master.set_pv_target_dh(first_master, PREP_PV_VEL_DEFAULT):
                failure_reason = "主端无法PV对齐回放起点"
                return

            if not self._wait_master_alignment(first_master):
                if self.replay_stop_event.is_set():
                    return
                failure_reason = "主端PV对齐回放起点超时"
                return

            if self.replay_stop_event.is_set():
                return

            # ---------- 正式进入 MIT 跟随 ----------
            if not self.master.enable_mit_follow(
                first_master,
                first_open,
                master_kp,
                master_kd,
                tool_kp,
                tool_kd,
            ):
                failure_reason = "主端无法进入MIT跟随模式"
                return
            master_follow_enabled = True

            self.state_signal.emit("示教回放中：UR5e回放 + 主端MIT跟随")
            self.log(
                f"[REPLAY] 正式回放，共 {len(timestamp)} 帧，"
                f"时长 {float(timestamp[-1] - timestamp[0]):.2f}s"
            )
            self.log(
                "[REPLAY] 主端MIT目标直接来自记录轨迹；"
                "RTDE actual_q仅用于反馈/显示，不参与主端目标生成"
            )
            if self.takeover_enabled:
                self.log(
                    "[TAKEOVER] 7轴检测已启用：J1~J6 + M7，"
                    "FAST/OPPOSITE/STATIONARY_PUSH/SLOW 四通道并行"
                )

            # 为避免 ±pi 包络导致主端MIT目标突然跳约2pi，
            # 主端跟随“记录轨迹本身”，而不是每帧依赖 RTDE actual_q。
            # 记录文件中的 ur_q 原本就是示教时采集到的 UR5e actual_q，
            # 因此它可以作为主端回放的确定性参考。RTDE Receive 仅用于
            # 状态显示、准备阶段和必要的监测，不再成为主端运动的唯一数据源。
            prev_record_wrapped = np.asarray(
                self.ur_q_to_master_equivalent(ur_q_record[0].tolist()),
                dtype=float,
            )
            master_follow_cont = prev_record_wrapped.copy()

            # ---------- 独立人工介入检测器初始化 ----------
            self._update_takeover_reference(
                master_follow_cont.tolist(),
                first_open,
                master_ref_vel=[0.0] * ARM_DOF,
                tool_ref_vel=0.0,
                ref_vel_valid=False,
            )
            self._start_takeover_detector()

            # 参考速度历史：用多帧窗口消除相邻时间戳过近造成的几十/上百 rad/s 假速度。
            ref_vel_history = deque(maxlen=TAKEOVER_REF_VEL_WINDOW_FRAMES + 1)
            master_ref_vel_filt = np.zeros(ARM_DOF, dtype=float)
            tool_ref_vel_filt = 0.0

            replay_start = time.perf_counter()
            t0 = float(timestamp[0])
            last_gripper_cmd = first_open
            next_gripper_time = time.monotonic()

            for i in range(len(timestamp)):
                # 人工接管具有比 REPLAY 更高的控制优先级。
                if self.replay_stop_event.is_set() or self.takeover_event.is_set():
                    break
                with self.control_lock:
                    if self.control_mode is not ControlMode.REPLAY:
                        break

                # 依据原记录时间轴调度当前帧。
                due = replay_start + max(0.0, float(timestamp[i]) - t0)
                remain = due - time.perf_counter()
                if remain > 0.0:
                    time.sleep(remain)

                # 睡眠结束后再次做控制权门禁，避免接管发生在等待期间时
                # Replay 仍多发一帧旧轨迹。
                if self.replay_stop_event.is_set() or self.takeover_event.is_set():
                    break

                q_cmd = ur_q_record[i].tolist()
                if not self._send_replay_servo_j(q_cmd):
                    if self.takeover_event.is_set():
                        break
                    failure_reason = f"第{i}帧 UR5e servoJ 失败或Replay已失去控制权"
                    break

                # 夹爪按较低频率更新，避免 Modbus 通信拖慢 servoJ。
                now_mono = time.monotonic()
                g_cmd = float(np.clip(gripper_record[i], 0.0, 1.0))
                if now_mono >= next_gripper_time:
                    if (
                        abs(g_cmd - last_gripper_cmd) >= GRIPPER_SEND_DEADBAND
                        or i == len(timestamp) - 1
                    ):
                        if not self.gripper.command_opening(g_cmd):
                            failure_reason = f"第{i}帧夹爪目标更新失败"
                            break
                        last_gripper_cmd = g_cmd
                    next_gripper_time = now_mono + GRIPPER_TARGET_PERIOD_S

                if not self.gripper.feedback_is_fresh():
                    failure_reason = "回放期间夹爪反馈超时"
                    break

                # ---------- 主端 MIT 跟随 ----------
                # 直接使用当前记录帧作为主端参考，而不是再等待/依赖
                # RTDE Receive 的 actual_q 刷新。这样 UR5e 和主端共享同一个
                # 示教时间轴，Receive 50 Hz 只承担反馈与监测职责。
                record_wrapped = np.asarray(
                    self.ur_q_to_master_equivalent(q_cmd), dtype=float
                )
                step = np.asarray(
                    [
                        Robot.minor_arc_dir(float(a), float(b))
                        for a, b in zip(prev_record_wrapped, record_wrapped)
                    ],
                    dtype=float,
                )

                if np.any(~np.isfinite(step)):
                    failure_reason = "主端跟随记录目标出现非有限角度"
                    break
                if float(np.max(np.abs(step))) > REPLAY_MAX_FOLLOW_STEP:
                    failure_reason = (
                        "主端MIT跟随记录目标出现异常跳变，max_step="
                        f"{float(np.max(np.abs(step))):.4f} rad"
                    )
                    break

                master_follow_cont += step
                prev_record_wrapped = record_wrapped

                # ---------- 稳健参考速度：多帧窗口 + 低通 + 合理性校验 ----------
                # 不再使用相邻两帧 timestamp 直接相除；某两个记录时间戳若异常接近，
                # 会产生几十甚至上百 rad/s 的假速度并导致人工介入误触发。
                ref_vel_history.append(
                    (
                        float(timestamp[i]),
                        master_follow_cont.copy(),
                        float(g_cmd),
                    )
                )

                ref_vel_valid = False
                master_ref_vel = master_ref_vel_filt.copy()
                tool_ref_vel = float(tool_ref_vel_filt)

                if len(ref_vel_history) >= 2:
                    t_now, q_now_ref, g_now_ref = ref_vel_history[-1]
                    # 优先使用窗口最老样本；若总时间跨度仍太短，则暂不提供速度给Detector。
                    t_old, q_old_ref, g_old_ref = ref_vel_history[0]
                    dt_window = float(t_now - t_old)
                    if dt_window >= TAKEOVER_REF_VEL_MIN_DT_S:
                        raw_master_ref_vel = (
                            np.asarray(q_now_ref, dtype=float)
                            - np.asarray(q_old_ref, dtype=float)
                        ) / dt_window
                        raw_tool_ref_vel = (float(g_now_ref) - float(g_old_ref)) / dt_window

                        raw_arm_max = float(np.max(np.abs(raw_master_ref_vel)))
                        raw_tool_abs = abs(float(raw_tool_ref_vel))
                        finite_ok = (
                            np.all(np.isfinite(raw_master_ref_vel))
                            and math.isfinite(float(raw_tool_ref_vel))
                        )
                        sanity_ok = (
                            raw_arm_max <= TAKEOVER_REF_VEL_SANITY_MAX_RAD_S
                            and raw_tool_abs <= TAKEOVER_TOOL_REF_VEL_SANITY_MAX_RAD_S
                        )

                        if finite_ok and sanity_ok:
                            alpha_v = float(TAKEOVER_REF_VEL_ALPHA)
                            master_ref_vel_filt = (
                                master_ref_vel_filt
                                + alpha_v * (raw_master_ref_vel - master_ref_vel_filt)
                            )
                            tool_ref_vel_filt = (
                                tool_ref_vel_filt
                                + alpha_v * (float(raw_tool_ref_vel) - tool_ref_vel_filt)
                            )
                            master_ref_vel = master_ref_vel_filt.copy()
                            tool_ref_vel = float(tool_ref_vel_filt)
                            ref_vel_valid = True
                        else:
                            # 位置参考仍然继续更新，但异常速度绝不参与人工介入判定。
                            ref_vel_valid = False

                # M7 同样直接跟随示教文件中的夹爪开度。真实夹爪反馈仍由
                # GripperController 持续读取，用于状态显示和通信新鲜度检查。
                if not self.master.update_mit_follow_target(
                    master_follow_cont.tolist(),
                    g_cmd,
                ):
                    failure_reason = "更新主端MIT跟随目标失败"
                    break

                # ---------- 更新独立 Detector 的共享参考 ----------
                # Detector 以 200 Hz 独立读取达妙电机 Velocity，不再依赖本回放循环的 dt。
                self._update_takeover_reference(
                    master_follow_cont.tolist(),
                    g_cmd,
                    master_ref_vel=master_ref_vel.tolist(),
                    tool_ref_vel=tool_ref_vel,
                    ref_vel_valid=ref_vel_valid,
                )


        except Exception as e:
            failure_reason = f"示教回放线程异常: {e}"
        finally:
            self._stop_takeover_detector()
            # ========================================================
            # 回放退出路径必须区分：人工抢占 vs 普通结束/停止/故障。
            # TAKEOVER 路径绝不能 servoStop、不能切PV，否则会打断新遥操作。
            # ========================================================
            self.replay_running = False
            self.replay_thread = None

            if takeover_triggered or self.takeover_event.is_set():
                # 最高优先级人工接管：Replay 已在触发点失去 servoJ 所有权。
                # 不调用 stopJ()/servoStop()，直接在同一个回放线程里完成交接，
                # 避免“旧Replay线程”和“新Teleop线程”并发争夺 UR5e。
                self.log("[TAKEOVER] Replay控制权已撤销，立即交接到人工遥操作...")
                if master_follow_enabled:
                    try:
                        self.master.disable_mit_follow()
                    except Exception:
                        pass

                takeover_ok = False
                try:
                    takeover_ok = bool(self._start_takeover_teleop_from_current())
                except Exception as e:
                    failure_reason = f"人工接管交接异常: {e}"
                    self.log(f"[ERR] {failure_reason}")
                    takeover_ok = False

                if not takeover_ok:
                    if failure_reason is None:
                        failure_reason = "人工介入已触发，但切换实时遥操作失败"
                    # 只有交接失败才执行安全停止和PV保持；同时强制退出
                    # TAKEOVER 中间态，避免主端自由而 UR5e 无控制源。
                    self.teleop_stop_event.set()
                    self.teleop_running = False
                    self.ur.stop_j()
                    self.ur.servo_stop()
                    self._set_control_mode(ControlMode.IDLE)
                    if self.master.initialized:
                        try:
                            self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                        except Exception as e:
                            self.log(f"[WARN] 接管失败后主端PV保持失败: {e}")
            else:
                # 普通完成 / 用户停止 / 异常：Replay仍是原控制源，安全结束。
                self.ur.stop_j()
                self.ur.servo_stop()
                if master_follow_enabled:
                    try:
                        self.master.disable_mit_follow()
                    except Exception:
                        pass
                self._set_control_mode(ControlMode.IDLE)

                if self.master.initialized:
                    try:
                        self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                    except Exception as e:
                        self.log(f"[WARN] 回放结束后主端PV保持失败: {e}")

            if failure_reason:
                self.log(f"[SAFE] {failure_reason}，示教回放停止")
                self.state_signal.emit("示教回放异常停止")
            elif takeover_triggered or self.takeover_triggered:
                # 成功接管时 _start_takeover_teleop_from_current() 已更新状态。
                pass
            elif self.replay_stop_event.is_set():
                self.log("[REPLAY] 用户停止示教回放")
                self.state_signal.emit("示教回放已停止")
            else:
                self.log("[REPLAY] 示教回放完成")
                self.state_signal.emit("示教回放完成")

    def stop_replay(self) -> bool:
        """用户主动停止回放。人工 TAKEOVER/已接管 TELEOP 时不允许旧Replay停止新控制源。"""
        self.replay_stop_event.set()
        mode = None
        with self.control_lock:
            mode = self.control_mode

        # 只有 Replay 仍持有控制权时，普通“停止回放”才可发停止命令。
        if mode is ControlMode.REPLAY:
            self.ur.stop_j()
            self.ur.servo_stop()
        elif mode is ControlMode.TAKEOVER:
            self.log("[TAKEOVER] 控制权正在交接，忽略普通Replay停止命令以保护人工接管")
        elif mode is ControlMode.TELEOP and self.takeover_triggered:
            self.log("[TAKEOVER] 当前已进入人工遥操作，停止Replay不会中断Teleop")

        th = self.replay_thread
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=3.0)

        # 仅普通回放结束时恢复主端；接管路径由接管函数管理 MIT/Teleop。
        with self.control_lock:
            mode_after = self.control_mode

        if mode_after is ControlMode.REPLAY:
            try:
                self.master.disable_mit_follow()
            except Exception:
                pass
            self._set_control_mode(ControlMode.IDLE)
            if self.master.initialized and (th is None or not th.is_alive()):
                try:
                    self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                except Exception:
                    pass

        if th is None or not th.is_alive():
            self.replay_running = False
            self.replay_thread = None
            return True

        self.log("[WARN] 回放线程尚未完全退出，但停止信号已发送")
        return False

    def cleanup(self):
        self._stop_takeover_detector()
        if self.recording:
            try:
                self.stop_recording(save=True)
            except Exception:
                pass
        self.stop_replay()
        self.stop_teleop(hold_master=False)
        self.gripper.disconnect()
        self.ur.disconnect()
        self.master.cleanup()
        self.preparation_done = False
        self.prep_target_master_q = None
        self.latest_tool_raw_opening = None
        self.latest_tool_filtered_opening = None
        self.takeover_armed = False
        self.takeover_triggered = False
        self.takeover_last_reason = ""
        self.takeover_event.clear()
        self._set_control_mode(ControlMode.IDLE)
