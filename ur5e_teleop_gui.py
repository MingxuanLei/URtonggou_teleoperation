"""
UR 同构小机械臂 -> UR5e 六关节遥操作 GUI

工作流程
--------
1. 初始化主端达妙电机（本版本只使能 1~6 号做遥操作；7 号保持失能）
2. 连接 UR5e（默认 IP: 192.168.3.15）
3. 读取 UR5e 当前 6 个关节角
4. 【准备模式】主端 1~6 号电机切到 PV，移动到 UR5e 的等价关节构型
5. 到位后【遥操作模式】主端 1~6 号切到 MIT + 重力补偿
6. 主端 6 个 DH 关节角的变化量按 1:1 映射到 UR5e，并通过 RTDE servoJ 实时控制

说明
----
- 第 7 号工具电机暂不映射到 UR5e 夹爪；本程序中保持失能。
- Robot.py 中第 3 轴方向 ratio=-1 的坐标换算会继续生效。
- 遥操作采用“相对增量 1:1”而不是直接把 [-pi, pi] 绝对值硬塞给 UR5e，
  这样可以保留 UR5e 当前的 2*pi 分支，避免 ±pi 处突然跳变。

依赖
----
pip install PySide6 numpy ur-rtde

请把本文件与以下文件放在同一目录：
DMMotor.py, Robot.py, USBCANFD.py, zlgcan.py, zlgcan.dll
"""

from __future__ import annotations

import math
import os
import sys
import time
import threading
from typing import List, Optional, Sequence, Tuple

# zlgcan.py 在 Windows 下使用 ./zlgcan.dll，确保工作目录为脚本所在目录。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR:
    os.chdir(SCRIPT_DIR)

import numpy as np

from PySide6.QtCore import QObject, QTimer, Qt, Signal
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from DMMotor import DMMotor
from Robot import Robot
from USBCANFD import USBCANFD

try:
    import rtde_control
    import rtde_receive
except ImportError:
    rtde_control = None
    rtde_receive = None


# ============================================================
# 全局参数
# ============================================================
UR_DEFAULT_IP = "192.168.3.15"

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

# 与你现有 GUIyemian_URtonggou.py 保持一致。
GRAVITY_TORQUE_SCALE = [0.0, 1.1, 1.1, 1.2, 1.1, 0.0]
GRAVITY_PERIOD_S = 0.001

# 准备模式：主端 PV 移动参数。
PREP_PV_VEL_DEFAULT = 0.30       # rad/s
PREP_TOL_DEFAULT = 0.035         # rad，约 2°
PREP_STABLE_TIME_S = 0.50        # 连续满足误差要求后才认为到位
PREP_TIMEOUT_DEFAULT = 40.0      # s

# 主端软件工作区（沿用现有项目约定，不代表机械硬限位）。
MASTER_DH_MIN = -math.pi
MASTER_DH_MAX = math.pi

# 遥操作控制。
RTDE_FREQUENCY_HZ = 125.0       # RTDE Receive / Control 显式设为 125 Hz
TELEOP_PERIOD_S = 0.008          # 125 Hz
TELEOP_SERVO_LOOKAHEAD = 0.10
TELEOP_SERVO_GAIN = 300
TELEOP_MAX_MASTER_STEP = 0.20    # 单周期主端角跳变保护，rad
TELEOP_START_ALIGN_TOL = 0.10    # 启动遥操作前主/从构型最大误差，rad

MODE_SWITCH_TIMEOUT_S = 3.0
POWER_ON_WAIT_S = 1.0


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

    def log(self, msg: str):
        self.log_signal.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    @staticmethod
    def _pack_command_for_mode(motor: DMMotor, mode: int) -> None:
        """刷新对应模式的缓存报文，不改变 motor.Mode。"""
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

    def _enable_arm_only_before_thread(self) -> bool:
        """只使能 1~6 号机械臂电机；第7号工具电机保持失能。"""
        assert self.can is not None
        self.can.stop_can()
        self.can.clearRecvBuffer()

        for motor in self.can.motors:
            data = self.can.send_wait(
                1, motor.ID, DMMotor.clear_error_command, 100
            )
            if not motor.read_motor(data):
                self.log(f"[ERR] 电机 {motor.ID} 清错无有效回复")
                return False

            data = self.can.send_wait(
                1, motor.ID, DMMotor.enable_command, 100
            )
            if not motor.read_motor(data):
                self.log(f"[ERR] 电机 {motor.ID} 使能无有效回复")
                return False

            if not motor.Enable:
                self.log(
                    f"[ERR] 电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}"
                )
                return False

            self.log(f"[OK] 电机 {motor.ID} 已使能")

        # 本版本明确不使用第7号工具电机。为避免任何无意运动，
        # 主动发送失能命令；USBCANFD 后续即使周期性填充其缓存，
        # 工具电机也不会执行位置运动。
        for tool in self.can.tools:
            try:
                data = self.can.send_wait(
                    1, tool.ID, DMMotor.disable_command, 100
                )
                tool.read_motor(data)
                self.log(f"[SAFE] 第7号工具电机 {tool.ID} 保持失能")
            except Exception as e:
                self.log(f"[WARN] 第7号工具电机失能命令异常: {e}")

        return True

    def _set_initial_hold_commands(self) -> None:
        """在 CAN 连续发送线程启动前，先把所有执行器目标设为当前位置。

        这样可以避免：
        - 1~6 号 PV 默认目标 0 导致意外回零；
        - 第 7 号工具 PVT 默认目标 0 导致本版本未使用工具时仍突然运动。
        """
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PV.position_set = float(motor.Position)
                motor.PV.velocity_lim = PREP_PV_VEL_DEFAULT
                self._pack_command_for_mode(motor, MODE_PV)

            if self.can.tools:
                tool = self.can.tools[0]
                tool.PVT.position_set = float(tool.Position)
                tool.PVT.velocity_lim = 0.0
                tool.PVT.torque_lim = 0.0
                self._pack_command_for_mode(tool, MODE_PVT)

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

            self.log("[4] 清错并使能 1~6 号电机；第7号保持失能...")
            if not self._enable_arm_only_before_thread():
                self.can.close_device()
                return False

            self.log("[5] 明确将主端 1~6 号切换到 PV 模式...")
            if not self.can.set_mode_all(MODE_PV):
                self.log("[ERR] 主端 1~6 号切换 PV 失败")
                self.can.close_device()
                return False
            self.current_mode = MODE_PV

            self.log("[6] 设置 1~6 号当前位置保持；第7号不参与控制...")
            self._set_initial_hold_commands()

            self.log("[7] 启动 CANFD 连续收发线程...")
            self.can.start_can_thread(1)

            if not self.wait_arm_feedback(2.0):
                self.log("[ERR] 未建立 1~6 号电机反馈")
                self.cleanup()
                return False

            arm_modes = [m.Mode for m in self.can.motors]
            if arm_modes and all(x == arm_modes[0] for x in arm_modes):
                self.current_mode = arm_modes[0]
            else:
                self.current_mode = None

            # 后台持续计算 MIT 重力补偿缓存；在 PV 模式下不会发送 MIT 缓存，
            # 切换到 MIT 时即可立即接管。
            self.gravity_stop_event.clear()
            self.gravity_thread = threading.Thread(
                target=self._gravity_loop,
                name="master_gravity_comp",
                daemon=True,
            )
            self.gravity_thread.start()

            self.initialized = True
            self.log("[OK] 主端初始化完成")
            self.log("[INFO] 第7号工具电机本版本保持失能，不参与遥操作")
            return True

    def wait_arm_feedback(self, timeout_s: float) -> bool:
        assert self.can is not None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if all(m.recv_num > 0 for m in self.can.motors):
                return True
            time.sleep(0.005)
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
                    self.log(f"[ERR] J{i+1} 目标不是有限数")
                    return False
                if not (MASTER_DH_MIN <= q <= MASTER_DH_MAX):
                    self.log(
                        f"[ERR] J{i+1}={q:.4f} 超出当前主端软件工作区 "
                        f"[-pi, pi]"
                    )
                    return False

            ok, motor_targets, valid = self.robot.dh2motor(
                self.can.motors, target
            )
            if not ok or not all(valid):
                self.log("[ERR] DH -> 电机角转换失败")
                return False

            # 先更新 PV 缓存，再切换模式，防止切换到 PV 的瞬间使用旧目标。
            with self.data_lock:
                for motor, q_motor in zip(self.can.motors, motor_targets):
                    if abs(float(q_motor)) > float(motor.max_position):
                        self.log(
                            f"[ERR] M{motor.ID} 目标 {q_motor:.4f} rad "
                            f"超出电机协议编码范围"
                        )
                        return False
                    motor.PV.position_set = float(q_motor)
                    motor.PV.velocity_lim = vel
                    self._pack_command_for_mode(motor, MODE_PV)

            if not self._switch_arm_mode(MODE_PV):
                return False

            self.log(
                "[PV] 主端目标 DH(rad): "
                + str([round(x, 4) for x in target])
            )
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
            # 重力线程一直在刷新 MIT 缓存，直接切模式即可。
            self.log("[MODE] 主端切换 MIT + 重力补偿，准备接受人工拖动")
            return self._switch_arm_mode(MODE_MIT)

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
                    for i, motor in enumerate(self.can.motors):
                        motor.MIT.position_set = 0.0
                        motor.MIT.velocity_set = 0.0
                        motor.MIT.kp_set = 0.0
                        motor.MIT.kd_set = 0.0
                        motor.MIT.torque_set = float(
                            tau_g_motor[i] * GRAVITY_TORQUE_SCALE[i]
                        )
                        self._pack_command_for_mode(motor, MODE_MIT)
            except Exception as e:
                self.log(f"[WARN] 重力补偿计算异常: {e}")
                time.sleep(0.01)

            time.sleep(GRAVITY_PERIOD_S)

        self.log("[GRAVITY] 主端重力补偿计算线程退出")

    def cleanup(self):
        with self.command_lock:
            if self.can is None:
                return

            self.gravity_stop_event.set()
            if self.gravity_thread is not None and self.gravity_thread.is_alive():
                self.gravity_thread.join(timeout=1.0)

            try:
                self.can.stop_can()
            except Exception:
                pass

            # 连续线程停掉后逐个发失能。
            try:
                for motor in self._all_actuators():
                    data = self.can.send_wait(
                        1, motor.ID, DMMotor.disable_command, 50
                    )
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
                    self.ip, RTDE_FREQUENCY_HZ
                )
                self.rtde_c = rtde_control.RTDEControlInterface(
                    self.ip, RTDE_FREQUENCY_HZ
                )
                q = self.rtde_r.getActualQ()
                if q is None or len(q) != ARM_DOF:
                    raise RuntimeError("getActualQ() 返回无效")
                self.connected = True
                self.log("[OK] UR5e RTDE 已连接")
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
        # 遥操作线程独占调用，不额外持长锁，减少抖动。
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
# 准备 + 遥操作协调器
# ============================================================
class TeleopCoordinator(QObject):
    log_signal = Signal(str)
    state_signal = Signal(str)
    operation_done_signal = Signal(str, bool)

    def __init__(self):
        super().__init__()
        self.master = MasterArmController()
        self.ur = UR5eController()

        self.master.log_signal.connect(self.log_signal.emit)
        self.ur.log_signal.connect(self.log_signal.emit)

        self.preparation_done = False
        self.prep_target_master_q: Optional[List[float]] = None
        self.last_ur_q: Optional[List[float]] = None

        self.teleop_stop_event = threading.Event()
        self.teleop_thread: Optional[threading.Thread] = None
        self.teleop_running = False
        self.lock = threading.RLock()

    def log(self, msg: str):
        self.log_signal.emit(f"[{time.strftime('%H:%M:%S')}] {msg}")

    @staticmethod
    def ur_q_to_master_equivalent(ur_q: Sequence[float]) -> List[float]:
        """把 UR5e 实际角映射到主端当前使用的 (-pi, pi] 表示。"""
        return [Robot.angle_clip_pnpi(float(q)) for q in ur_q]

    @staticmethod
    def joint_error(current: Sequence[float], target: Sequence[float]) -> List[float]:
        return [
            float(Robot.minor_arc_dir(float(c), float(t)))
            for c, t in zip(current, target)
        ]

    def read_ur_configuration(self) -> Optional[Tuple[List[float], List[float]]]:
        q_ur = self.ur.get_actual_q()
        if q_ur is None:
            self.log("[ERR] 无法读取 UR5e 当前关节角")
            return None
        q_master_equiv = self.ur_q_to_master_equivalent(q_ur)
        self.last_ur_q = q_ur
        self.prep_target_master_q = q_master_equiv
        self.preparation_done = False

        self.log(
            "[UR] ActualQ(rad): "
            + str([round(x, 4) for x in q_ur])
        )
        self.log(
            "[PREP] 主端等价目标(rad): "
            + str([round(x, 4) for x in q_master_equiv])
        )
        return q_ur, q_master_equiv

    def run_preparation(
        self,
        pv_velocity: float,
        tolerance: float,
        timeout_s: float,
    ) -> bool:
        with self.lock:
            if self.teleop_running:
                self.log("[ERR] 遥操作运行中，不能进入准备模式")
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
            self.log("[PREP] 开始：主端以 PV 移动到 UR5e 当前构型")

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
                    self.state_signal.emit("准备完成：可以开始遥操作")
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
            if self.teleop_running:
                self.log("[INFO] 遥操作已经运行")
                return True
            if not self.preparation_done or self.prep_target_master_q is None:
                self.log("[ERR] 请先完成准备模式")
                return False
            if not self.master.initialized or not self.ur.connected:
                self.log("[ERR] 主端或 UR5e 未连接")
                return False

            q_master = self.master.get_dh_q()
            q_ur = self.ur.get_actual_q()
            if q_master is None or q_ur is None:
                self.log("[ERR] 启动遥操作前无法读取主/从关节角")
                return False

            # 确认主端仍在准备构型附近。
            prep_err = self.joint_error(q_master, self.prep_target_master_q)
            if max(abs(x) for x in prep_err) > TELEOP_START_ALIGN_TOL:
                self.log(
                    "[ERR] 主端已经离开准备构型，请重新执行准备模式；err="
                    + str([round(x, 4) for x in prep_err])
                )
                self.preparation_done = False
                return False

            # 先切换主端为可拖动的 MIT + 重力补偿。
            if not self.master.switch_to_mit_gravity():
                self.log("[ERR] 主端切换 MIT + 重力补偿失败")
                return False

            # 切换后重新读取一次，作为 1:1 增量映射的零点。
            time.sleep(0.05)
            q_master_start = self.master.get_dh_q()
            q_ur_start = self.ur.get_actual_q()
            if q_master_start is None or q_ur_start is None:
                self.log("[ERR] 无法建立遥操作起始零点")
                return False

            self.teleop_stop_event.clear()
            self.teleop_running = True
            self.state_signal.emit("遥操作中：1~6 轴 1:1")

            self.teleop_thread = threading.Thread(
                target=self._teleop_loop,
                args=(q_master_start, q_ur_start),
                name="ur5e_joint_teleop",
                daemon=True,
            )
            self.teleop_thread.start()

            self.log("[TELEOP] 已启动 1~6 轴关节角 1:1 遥操作")
            self.log("[TELEOP] 第7号工具电机未映射")
            return True

    def _teleop_loop(self, master_start: List[float], ur_start: List[float]):
        prev_wrapped = np.array(master_start, dtype=float)
        master_cont = np.array(master_start, dtype=float)
        master_start_arr = np.array(master_start, dtype=float)
        ur_start_arr = np.array(ur_start, dtype=float)

        next_tick = time.perf_counter()
        failure_reason: Optional[str] = None

        try:
            while not self.teleop_stop_event.is_set():
                q_wrapped_list = self.master.get_dh_q()
                if q_wrapped_list is None:
                    failure_reason = "主端关节反馈丢失"
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

                # 1:1 增量映射：UR_target = UR_start + (Master - Master_start)
                ur_target = ur_start_arr + (master_cont - master_start_arr)

                if np.any(~np.isfinite(ur_target)):
                    failure_reason = "UR5e 目标出现非有限数"
                    break

                ok = self.ur.servo_j(ur_target.tolist(), TELEOP_PERIOD_S)
                if not ok:
                    failure_reason = "UR5e servoJ 返回失败"
                    break

                next_tick += TELEOP_PERIOD_S
                remain = next_tick - time.perf_counter()
                if remain > 0:
                    time.sleep(remain)
                else:
                    # 如果系统调度偶尔落后，不累计越来越大的延迟。
                    next_tick = time.perf_counter()

        except Exception as e:
            failure_reason = f"遥操作线程异常: {e}"
        finally:
            self.ur.servo_stop()
            self.teleop_running = False

            if failure_reason:
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

        self.ur.servo_stop()

        if hold_master and self.master.initialized:
            # 停止后把主端当前姿态锁住，避免一直处于可自由拖动状态。
            try:
                self.master.hold_current_in_pv(PREP_PV_VEL_DEFAULT)
                self.log("[STOP] 主端已切回 PV 并保持当前姿态")
            except Exception as e:
                self.log(f"[WARN] 主端切回 PV 保持失败: {e}")

        self.teleop_running = False
        self.teleop_thread = None
        return True

    def cleanup(self):
        self.stop_teleop(hold_master=False)
        self.ur.disconnect()
        self.master.cleanup()
        self.preparation_done = False
        self.prep_target_master_q = None


# ============================================================
# GUI
# ============================================================
class MainWindow(QMainWindow):
    worker_done = Signal(str, bool)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR 同构主端 → UR5e 遥操作（准备模式 + 6轴1:1）")
        self.resize(1380, 860)

        self.coordinator = TeleopCoordinator()
        self.coordinator.log_signal.connect(self.append_log)
        self.coordinator.state_signal.connect(self.set_state)
        self.worker_done.connect(self.on_worker_done)

        self.worker_running = False

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(150)

    def _build_ui(self):
        central = QWidget()
        outer = QVBoxLayout(central)

        splitter = QSplitter(Qt.Vertical)
        top = QWidget()
        top_layout = QHBoxLayout(top)

        left = QWidget()
        left_layout = QVBoxLayout(left)

        self._build_connection_group(left_layout)
        self._build_prepare_group(left_layout)
        self._build_teleop_group(left_layout)
        left_layout.addStretch(1)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        self._build_status_group(right_layout)

        top_layout.addWidget(left, 0)
        top_layout.addWidget(right, 1)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        splitter.addWidget(top)
        splitter.addWidget(self.log_box)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        outer.addWidget(splitter)
        self.setCentralWidget(central)

    def _build_connection_group(self, parent):
        box = QGroupBox("1. 设备连接")
        g = QGridLayout(box)

        self.ip_edit = QLineEdit(UR_DEFAULT_IP)
        self.btn_master_init = QPushButton("初始化主端 CAN + 7电机")
        self.btn_ur_connect = QPushButton("连接 UR5e")
        self.btn_read_ur = QPushButton("读取 UR5e 当前关节角")

        self.btn_master_init.clicked.connect(
            lambda: self.run_async("主端初始化", self.coordinator.master.initialize)
        )
        self.btn_ur_connect.clicked.connect(self.connect_ur_async)
        self.btn_read_ur.clicked.connect(self.read_ur_now)

        g.addWidget(QLabel("UR5e IP"), 0, 0)
        g.addWidget(self.ip_edit, 0, 1)
        g.addWidget(self.btn_master_init, 1, 0, 1, 2)
        g.addWidget(self.btn_ur_connect, 2, 0, 1, 2)
        g.addWidget(self.btn_read_ur, 3, 0, 1, 2)

        parent.addWidget(box)

    def _build_prepare_group(self, parent):
        box = QGroupBox("2. 准备模式：主端 PV 对齐 UR5e")
        g = QGridLayout(box)

        self.prep_vel = QDoubleSpinBox()
        self.prep_vel.setRange(0.05, 1.0)
        self.prep_vel.setDecimals(2)
        self.prep_vel.setSingleStep(0.05)
        self.prep_vel.setValue(PREP_PV_VEL_DEFAULT)

        self.prep_tol = QDoubleSpinBox()
        self.prep_tol.setRange(0.005, 0.20)
        self.prep_tol.setDecimals(3)
        self.prep_tol.setSingleStep(0.005)
        self.prep_tol.setValue(PREP_TOL_DEFAULT)

        self.prep_timeout = QDoubleSpinBox()
        self.prep_timeout.setRange(5.0, 120.0)
        self.prep_timeout.setDecimals(1)
        self.prep_timeout.setValue(PREP_TIMEOUT_DEFAULT)

        self.btn_prepare = QPushButton("开始准备：读取UR5e → 主端PV对齐")
        self.btn_prepare.clicked.connect(self.prepare_async)

        note = QLabel(
            "准备阶段只控制主端1~6轴。第7工具电机保持失能。\n"
            "UR5e 本身在准备阶段不运动。"
        )
        note.setWordWrap(True)

        g.addWidget(QLabel("主端 PV 速度上限(rad/s)"), 0, 0)
        g.addWidget(self.prep_vel, 0, 1)
        g.addWidget(QLabel("到位容差(rad)"), 1, 0)
        g.addWidget(self.prep_tol, 1, 1)
        g.addWidget(QLabel("准备超时(s)"), 2, 0)
        g.addWidget(self.prep_timeout, 2, 1)
        g.addWidget(self.btn_prepare, 3, 0, 1, 2)
        g.addWidget(note, 4, 0, 1, 2)

        parent.addWidget(box)

    def _build_teleop_group(self, parent):
        box = QGroupBox("3. 遥操作模式")
        v = QVBoxLayout(box)

        self.state_label = QLabel("状态：等待初始化")
        self.state_label.setStyleSheet("font-weight: bold; font-size: 15px;")

        self.btn_start_teleop = QPushButton("开始遥操作（1~6轴 1:1）")
        self.btn_stop_teleop = QPushButton("停止遥操作")
        self.btn_safe_stop = QPushButton("安全停止并退出控制")

        self.btn_start_teleop.clicked.connect(
            lambda: self.run_async("启动遥操作", self.coordinator.start_teleop)
        )
        self.btn_stop_teleop.clicked.connect(
            lambda: self.run_async("停止遥操作", self.coordinator.stop_teleop)
        )
        self.btn_safe_stop.clicked.connect(self.safe_stop_async)

        note = QLabel(
            "开始遥操作时：主端 1~6 号从 PV 切换到 MIT + 重力补偿；\n"
            "UR5e 目标 = UR5e启动角 + 主端相对启动角变化，比例严格为 1:1。"
        )
        note.setWordWrap(True)

        v.addWidget(self.state_label)
        v.addWidget(self.btn_start_teleop)
        v.addWidget(self.btn_stop_teleop)
        v.addWidget(self.btn_safe_stop)
        v.addWidget(note)
        parent.addWidget(box)

    def _build_status_group(self, parent):
        box = QGroupBox("实时关节状态")
        v = QVBoxLayout(box)

        self.connection_label = QLabel("主端：未初始化 | UR5e：未连接")
        v.addWidget(self.connection_label)

        self.joint_table = QTableWidget(ARM_DOF, 6)
        self.joint_table.setHorizontalHeaderLabels(
            [
                "Joint",
                "主端 DH(rad)",
                "主端(deg)",
                "UR5e(rad)",
                "UR5e(deg)",
                "主-UR等价误差(rad)",
            ]
        )
        self.joint_table.verticalHeader().setVisible(False)
        for r in range(ARM_DOF):
            for c in range(6):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.joint_table.setItem(r, c, item)
        v.addWidget(self.joint_table)

        self.motor_table = QTableWidget(TOTAL_MOTOR_NUM, 7)
        self.motor_table.setHorizontalHeaderLabels(
            ["ID", "Mode", "Enable", "ERR", "Pos(rad)", "Vel", "Recv"]
        )
        self.motor_table.verticalHeader().setVisible(False)
        for r in range(TOTAL_MOTOR_NUM):
            for c in range(7):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.motor_table.setItem(r, c, item)
        v.addWidget(self.motor_table)

        parent.addWidget(box)

    def append_log(self, text: str):
        self.log_box.append(text)
        self.log_box.moveCursor(QTextCursor.End)

    def set_state(self, state: str):
        self.state_label.setText("状态：" + state)

    def set_action_buttons_enabled(self, enabled: bool):
        for btn in (
            self.btn_master_init,
            self.btn_ur_connect,
            self.btn_read_ur,
            self.btn_prepare,
            self.btn_start_teleop,
            self.btn_safe_stop,
        ):
            btn.setEnabled(enabled)
        # 停止按钮始终尽量可用。
        self.btn_stop_teleop.setEnabled(True)

    def run_async(self, label: str, func):
        if self.worker_running:
            self.append_log("[WARN] 上一个操作仍在执行")
            return

        self.worker_running = True
        self.set_action_buttons_enabled(False)

        def worker():
            ok = False
            try:
                ok = bool(func())
            except Exception as e:
                self.coordinator.log(f"[ERR] {label} 异常: {e}")
                ok = False
            finally:
                self.worker_done.emit(label, ok)

        threading.Thread(target=worker, daemon=True).start()

    def on_worker_done(self, label: str, ok: bool):
        self.worker_running = False
        self.set_action_buttons_enabled(True)
        self.append_log(f"[DONE] {label}: {'成功' if ok else '失败'}")
        self.refresh_status()

    def connect_ur_async(self):
        ip = self.ip_edit.text().strip()
        self.run_async("连接UR5e", lambda: self.coordinator.ur.connect(ip))

    def read_ur_now(self):
        readout = self.coordinator.read_ur_configuration()
        if readout is None:
            QMessageBox.warning(self, "读取失败", "请先连接 UR5e。")
            return
        self.refresh_status()

    def prepare_async(self):
        vel = float(self.prep_vel.value())
        tol = float(self.prep_tol.value())
        timeout_s = float(self.prep_timeout.value())
        self.run_async(
            "准备模式",
            lambda: self.coordinator.run_preparation(vel, tol, timeout_s),
        )

    def safe_stop_async(self):
        def task():
            self.coordinator.cleanup()
            return True
        self.run_async("安全停止", task)

    def refresh_status(self):
        master_ok = self.coordinator.master.initialized
        ur_ok = self.coordinator.ur.connected
        self.connection_label.setText(
            f"主端：{'已初始化' if master_ok else '未初始化'} | "
            f"UR5e：{'已连接' if ur_ok else '未连接'} | "
            f"遥操作：{'运行中' if self.coordinator.teleop_running else '停止'}"
        )

        q_master = self.coordinator.master.get_dh_q() if master_ok else None
        q_ur = self.coordinator.ur.get_actual_q() if ur_ok else None
        q_ur_equiv = (
            self.coordinator.ur_q_to_master_equivalent(q_ur)
            if q_ur is not None
            else None
        )

        for i in range(ARM_DOF):
            self.joint_table.item(i, 0).setText(f"J{i+1}")

            if q_master is not None:
                self.joint_table.item(i, 1).setText(f"{q_master[i]:.4f}")
                self.joint_table.item(i, 2).setText(
                    f"{math.degrees(q_master[i]):.2f}"
                )
            else:
                self.joint_table.item(i, 1).setText("-")
                self.joint_table.item(i, 2).setText("-")

            if q_ur is not None:
                self.joint_table.item(i, 3).setText(f"{q_ur[i]:.4f}")
                self.joint_table.item(i, 4).setText(
                    f"{math.degrees(q_ur[i]):.2f}"
                )
            else:
                self.joint_table.item(i, 3).setText("-")
                self.joint_table.item(i, 4).setText("-")

            if q_master is not None and q_ur_equiv is not None:
                e = Robot.minor_arc_dir(q_ur_equiv[i], q_master[i])
                self.joint_table.item(i, 5).setText(f"{e:.4f}")
            else:
                self.joint_table.item(i, 5).setText("-")

        motors = self.coordinator.master.get_motor_snapshot() if master_ok else None
        for r in range(TOTAL_MOTOR_NUM):
            if motors is None or r >= len(motors):
                for c in range(7):
                    self.motor_table.item(r, c).setText("-")
                continue

            m = motors[r]
            vals = [
                str(m["id"]),
                str(m["mode"]),
                str(m["enable"]),
                str(m["err"]),
                f"{m['pos']:.4f}",
                f"{m['vel']:.4f}",
                str(m["recv"]),
            ]
            for c, text in enumerate(vals):
                self.motor_table.item(r, c).setText(text)

    def closeEvent(self, event):
        if self.worker_running:
            QMessageBox.warning(
                self,
                "提示",
                "当前操作仍在执行，请先使用停止按钮或待当前命令结束后退出。",
            )
            event.ignore()
            return

        reply = QMessageBox.question(
            self,
            "确认退出",
            "退出前将停止 UR5e servoJ、断开 RTDE，并失能主端 7 个电机。\n是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            event.ignore()
            return

        self.timer.stop()
        try:
            self.coordinator.cleanup()
        except Exception as e:
            self.append_log(f"[WARN] 退出清理异常: {e}")
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
