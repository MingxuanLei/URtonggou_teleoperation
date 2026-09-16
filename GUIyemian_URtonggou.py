# 功能：
# 1. 打开、初始化 CANFD，并使能 1~7 号电机（第7号为工具电机）
# 2. 启动 CANFD 连续收发线程
# 3. 启动 MIT 重力补偿线程
# 4. 在线切换 1~6 号机械臂电机的 MIT / PV / PVT 模式
# 5. 第7号工具电机与 1~6 号机械臂模式分开管理
# 6. PV 模式按界面输入的 DH 关节角目标运动
# 7. PV 安全检查独立于 Robot.py / USBCANFD.py
# 8. 超限目标直接拒绝，不再静默退回当前位置保持
# 9. 已经处于 PV 模式时，允许再次更新新的 PV 目标

import sys
import time
import math
import threading
from dataclasses import dataclass
from typing import Optional, List, Sequence, Tuple

from PySide6.QtGui import QTextCursor
from PySide6.QtCore import QObject, Signal, QTimer, Qt
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QTextEdit,
    QTableWidget,
    QTableWidgetItem,
    QDoubleSpinBox,
    QCheckBox,
    QMessageBox,
    QSplitter,
) 

from USBCANFD import USBCANFD
from DMMotor import DMMotor
from Robot import Robot


MODE_MIT = 1
MODE_PV = 2
MODE_PVT = 4

MODE_NAME = {
    MODE_MIT: "MIT",
    MODE_PV: "PV",
    MODE_PVT: "PVT",
}

MODE_SWITCH_TIMEOUT_S = 3.0
GRAVITY_COMP_PERIOD_S = 0.001

ARM_DOF = 6
TOTAL_MOTOR_NUM = 7

# 重力补偿系数：保持你当前版本的设置。
GRAVITY_TORQUE_SCALE = [0.0, 1.1, 1.1, 1.2, 1.1, 0.0]

# 使用你最近提供的 PV 目标。
DEFAULT_PV_TARGET_DH_Q = [
    -1.56,
    -1.56,
    -1.56,
    -1.56,
     1.56,
     0.0,
]

DEFAULT_PV_MOVE_VEL_LIM = 0.4
DEFAULT_TOOL_TARGET_Q = 0.0

PVT_HOLD_VEL_LIM = 0.0
PVT_HOLD_TORQUE_LIM = 0.0
POWER_ON_WAIT_S = 1.0


@dataclass
class PVSafetyConfig:
    """PV 软件安全层。

    这些参数是 Python 控制程序自己的软件保护，不等价于机械硬限位。
    真正的机械极限仍应根据实机结构重新标定。

    当前设计原则：
    - Robot.py 只做坐标转换；
    - USBCANFD.py 只负责 CAN/协议；
    - 所有“是否允许运动”的判断集中在这里。
    """

    enabled: bool = True

    # DH角输入统一限定到 [-pi, pi]。
    # 这是与 Robot.Angle 的归一化范围一致的软件工作区，
    # 不是机械臂厂家给出的机械硬限位。
    dh_soft_limits: Tuple[Tuple[float, float], ...] = (
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
    )

    # 单次目标与当前 DH 角之间允许的最大最短角差。
    # 2.20 rad ≈ 126°，你当前从接近零位移动到 ±1.56 rad 可通过。
    max_delta_rad: float = 2.20

    # Python GUI允许的 PV 速度上限。
    max_velocity_lim: float = 1.0

    # 进入 PV 位置运动前必须至少收到过 1~6 号电机反馈。
    require_arm_feedback: bool = True


class ArmController(QObject):
    log_signal = Signal(str)

    def __init__(
        self,
        device_index: int = 0,
        channel_index: int = 0,
        name: str = "arm",
    ):
        super().__init__()

        self.device_index = int(device_index)
        self.channel_index = int(channel_index)
        self.name = name

        self.can: Optional[USBCANFD] = None
        self.robot: Optional[Robot] = None

        self.initialized = False
        self.current_mode: Optional[int] = None

        self.gravity_stop_event = threading.Event()
        self.gravity_thread: Optional[threading.Thread] = None

        self.command_lock = threading.RLock()
        self.data_lock = threading.RLock()

        self.disable_on_exit = True
        self.safety = PVSafetyConfig()

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_signal.emit(f"[{ts}] {msg}")

    def all_actuators(self) -> List[DMMotor]:
        if self.can is None:
            return []
        return list(self.can.motors) + list(self.can.tools)

    @staticmethod
    def pack_command_for_mode(motor: DMMotor, mode: int) -> None:
        """Refresh one mode-specific cached command without changing Mode."""
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

    # ============================================================
    # Status
    # ============================================================
    def get_status_snapshot(self) -> Optional[dict]:
        if self.can is None or self.robot is None:
            return None

        try:
            with self.data_lock:
                motors = []

                for m in self.all_actuators():
                    motors.append(
                        {
                            "id": m.ID,
                            "mode": m.ModeName,
                            "enable": m.Enable,
                            "err": m.ERRCODE,
                            "pos": float(m.Position),
                            "vel": float(m.Velocity),
                            "tau": float(m.Torque),
                            "recv": int(m.recv_num),
                        }
                    )

                q_now = self.robot.motor2dh(self.can.motors)
                q_rad = [float(x) for x in q_now]
                q_deg = [
                    float(x * 180.0 / math.pi) for x in q_now
                ]

                return {
                    "initialized": self.initialized,
                    "is_updating": bool(self.can.IsUpdating),
                    "current_mode": self.current_mode,
                    "arm_modes": [m.Mode for m in self.can.motors],
                    "tool_modes": [m.Mode for m in self.can.tools],
                    "motors": motors,
                    "dh_rad": q_rad,
                    "dh_deg": q_deg,
                    "can_param": self.can.CanParam,
                }

        except Exception as e:
            self.log(f"[WARN] 状态读取失败: {e}")
            return None

    # ============================================================
    # Enable / disable
    # ============================================================
    def enable_motors_only_before_thread(self) -> bool:
        assert self.can is not None

        self.can.stop_can()
        self.can.clearRecvBuffer()

        for motor in self.all_actuators():
            data = self.can.send_wait(
                1,
                motor.ID,
                DMMotor.clear_error_command,
                100,
            )
            if not motor.read_motor(data):
                self.log(
                    f"[ERR] 电机 {motor.ID} 清错无有效回复"
                )
                return False

            data = self.can.send_wait(
                1,
                motor.ID,
                DMMotor.enable_command,
                100,
            )
            if not motor.read_motor(data):
                self.log(
                    f"[ERR] 电机 {motor.ID} 使能无有效回复"
                )
                return False

            if not motor.Enable:
                self.log(
                    f"[ERR] 电机 {motor.ID} 使能失败，"
                    f"ERR={motor.ERRCODE}"
                )
                return False

            self.log(f"[OK] 电机 {motor.ID} 已使能")

        return True

    def disable_motors_only_at_exit(self):
        if self.can is None:
            return

        self.can.stop_can()

        for motor in self.all_actuators():
            try:
                data = self.can.send_wait(
                    1,
                    motor.ID,
                    DMMotor.disable_command,
                    50,
                )
                motor.read_motor(data)

                self.log(
                    f"[EXIT] 电机 {motor.ID} 已发送失能命令，"
                    f"Enable={motor.Enable}, ERR={motor.ERRCODE}"
                )
            except Exception as e:
                self.log(
                    f"[WARN] 电机 {motor.ID} 失能异常: {e}"
                )

    # ============================================================
    # Command cache preparation
    # ============================================================
    def set_all_mit_zero_torque(self):
        """Prepare zero-torque MIT cache for arm motors only."""
        if self.can is None:
            return

        with self.data_lock:
            for motor in self.can.motors:
                motor.MIT.position_set = 0.0
                motor.MIT.velocity_set = 0.0
                motor.MIT.kp_set = 0.0
                motor.MIT.kd_set = 0.0
                motor.MIT.torque_set = 0.0
                self.pack_command_for_mode(motor, MODE_MIT)

    def set_pv_hold_current_position(self, velocity_lim: float):
        """Explicit arm hold command.  It is NOT an automatic safety fallback."""
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PV.position_set = float(motor.Position)
                motor.PV.velocity_lim = float(velocity_lim)
                self.pack_command_for_mode(motor, MODE_PV)

        self.log("[PV] 已显式设置 1~6 号电机当前位置保持")

    def set_pvt_hold_current_position(self):
        """Prepare PVT hold commands for arm motors 1~6."""
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PVT.position_set = float(motor.Position)
                motor.PVT.velocity_lim = PVT_HOLD_VEL_LIM
                motor.PVT.torque_lim = PVT_HOLD_TORQUE_LIM
                self.pack_command_for_mode(motor, MODE_PVT)

        self.log("[PVT] 已将 1~6 号机械臂电机设置为当前位置保持")

    # ============================================================
    # Independent PV safety layer
    # ============================================================
    def validate_pv_target(
        self,
        target_dh_q: Sequence[float],
        target_motor_q: Sequence[float],
        velocity_lim: float,
    ) -> Tuple[bool, List[str]]:
        """Check whether a PV target may be sent.

        This is intentionally separate from Robot.dh2motor().
        """
        errors: List[str] = []

        if self.can is None or self.robot is None:
            return False, ["控制器尚未初始化"]

        if len(target_dh_q) != ARM_DOF:
            return False, [f"DH目标必须为 {ARM_DOF} 个数"]

        if len(target_motor_q) != ARM_DOF:
            return False, [f"电机目标必须为 {ARM_DOF} 个数"]

        for i, q in enumerate(target_dh_q):
            try:
                qf = float(q)
            except Exception:
                errors.append(f"J{i + 1} DH目标不是有效数字")
                continue

            if not math.isfinite(qf):
                errors.append(f"J{i + 1} DH目标不是有限数")

        try:
            velocity_lim = float(velocity_lim)
        except Exception:
            errors.append("PV速度限制不是有效数字")
            velocity_lim = float("nan")

        if (
            not math.isfinite(velocity_lim)
            or velocity_lim <= 0.0
        ):
            errors.append("PV速度限制必须 > 0")

        if (
            math.isfinite(velocity_lim)
            and velocity_lim > self.safety.max_velocity_lim
        ):
            errors.append(
                f"PV速度 {velocity_lim:.3f} rad/s 超过软件上限 "
                f"{self.safety.max_velocity_lim:.3f} rad/s"
            )

        # Protocol encoding boundary.  This is NOT a mechanical limit.
        for i, (motor, target) in enumerate(
            zip(self.can.motors, target_motor_q)
        ):
            tf = float(target)

            if not math.isfinite(tf):
                errors.append(
                    f"M{i + 1} 电机目标不是有限数"
                )
                continue

            protocol_max = float(motor.max_position)
            if not (-protocol_max <= tf <= protocol_max):
                errors.append(
                    f"M{i + 1} 目标 {tf:.4f} rad 超出电机协议编码范围 "
                    f"[-{protocol_max:.4f}, {protocol_max:.4f}]"
                )

        if not self.safety.enabled:
            return len(errors) == 0, errors

        if self.safety.require_arm_feedback:
            no_feedback = [
                m.ID for m in self.can.motors if m.recv_num <= 0
            ]
            if no_feedback:
                errors.append(
                    "以下机械臂电机尚无有效反馈，拒绝PV运动: "
                    + str(no_feedback)
                )

        # Explicit DH software limits.
        for i, q in enumerate(target_dh_q):
            qf = float(q)
            lo, hi = self.safety.dh_soft_limits[i]

            if not (lo <= qf <= hi):
                errors.append(
                    f"J{i + 1} DH目标 {qf:.4f} rad 超出软件DH范围 "
                    f"[{lo:.4f}, {hi:.4f}]"
                )

        # Explicit maximum one-command target change.
        try:
            current_dh = self.robot.motor2dh(self.can.motors)

            for i, (current, target) in enumerate(
                zip(current_dh, target_dh_q)
            ):
                delta = abs(
                    self.robot.minor_arc_dir(
                        float(current),
                        float(target),
                    )
                )

                if delta > self.safety.max_delta_rad:
                    errors.append(
                        f"J{i + 1} 单次目标变化 {delta:.4f} rad "
                        f"超过软件上限 {self.safety.max_delta_rad:.4f} rad；"
                        f"current={float(current):.4f}, "
                        f"target={float(target):.4f}"
                    )

        except Exception as e:
            errors.append(
                f"计算当前DH角/单次变化量失败: {e}"
            )

        return len(errors) == 0, errors

    def set_pv_target_motor_position(
        self,
        target_motor_q: List[float],
        velocity_lim: float,
    ) -> bool:
        assert self.can is not None

        if len(target_motor_q) != ARM_DOF:
            self.log(
                f"[ERR] target_motor_q 必须是 {ARM_DOF} 个数"
            )
            return False

        with self.data_lock:
            for i, motor in enumerate(self.can.motors):
                motor.PV.position_set = float(target_motor_q[i])
                motor.PV.velocity_lim = float(velocity_lim)
                self.pack_command_for_mode(motor, MODE_PV)

        self.log(
            "[PV] 已写入 1~6 号电机目标 rad: "
            + str(
                [
                    f"{float(x):.4f}"
                    for x in target_motor_q
                ]
            )
        )
        return True

    def set_tool_pvt_target(
        self,
        target: float,
        velocity_lim: float,
    ) -> bool:
        """Optional tool command; tool mode remains PVT.

        The current DMMotor PVT encoder caps positive position at 2.35 rad.
        To avoid hidden clipping, this GUI explicitly restricts the optional
        tool target to [-2.35, 2.35] rad.  This is a software/protocol guard,
        NOT a calibrated mechanical tool limit.
        """
        assert self.can is not None

        if not self.can.tools:
            self.log("[WARN] 未检测到第7工具电机")
            return False

        target = float(target)

        if not math.isfinite(target):
            self.log("[ERR] 工具电机目标不是有限数")
            return False

        if not (-2.35 <= target <= 2.35):
            self.log(
                f"[ERR] 工具PVT目标 {target:.4f} rad 超出 "
                "[-2.35, 2.35] 软件/协议范围"
            )
            return False

        tool = self.can.tools[0]

        if tool.Mode != MODE_PVT:
            self.log(
                f"[ERR] 第7工具电机当前模式={tool.ModeName}，"
                "本程序不自动把工具跟随机械臂切换模式；"
                "请先确认工具模式。"
            )
            return False

        with self.data_lock:
            tool.PVT.position_set = target
            tool.PVT.velocity_lim = min(
                float(velocity_lim),
                self.safety.max_velocity_lim,
            )
            tool.PVT.torque_lim = 0.0
            self.pack_command_for_mode(tool, MODE_PVT)

        self.log(
            f"[TOOL] 已写入第7电机 PVT 目标 {target:.4f} rad"
        )
        return True

    def set_pv_target_dh_position(
        self,
        target_dh_q: List[float],
        velocity_lim: float,
        tool_target_q: Optional[float] = None,
    ) -> bool:
        assert self.can is not None
        assert self.robot is not None

        self.log(
            "[PV] 请求 DH 目标 rad: "
            + str([f"{float(x):.4f}" for x in target_dh_q])
        )

        ok, target_motor_q, valid_input = self.robot.dh2motor(
            self.can.motors,
            target_dh_q,
        )

        if not ok:
            self.log(
                "[ERR] DH→电机角转换失败，"
                f"valid_input={valid_input}"
            )
            return False

        self.log(
            "[PV] DH→电机原始目标 rad: "
            + str([f"{float(x):.4f}" for x in target_motor_q])
        )

        safe, errors = self.validate_pv_target(
            target_dh_q,
            target_motor_q,
            velocity_lim,
        )

        if not safe:
            self.log("[SAFE] PV目标被软件安全层拒绝：")
            for msg in errors:
                self.log("       - " + msg)
            self.log(
                "[SAFE] 本次不会修改PV目标，也不会自动退回当前位置保持"
            )
            return False

        if not self.set_pv_target_motor_position(
            target_motor_q,
            velocity_lim,
        ):
            return False

        if tool_target_q is not None:
            if not self.set_tool_pvt_target(
                tool_target_q,
                velocity_lim,
            ):
                return False

        self.log("[SAFE] PV目标安全检查通过")
        return True

    # ============================================================
    # Mode command preparation / switching
    # ============================================================
    def prepare_command_for_target_mode(
        self,
        target_mode: int,
        target_dh_q: Optional[List[float]],
        pv_velocity_lim: float,
        tool_target_q: Optional[float] = None,
    ) -> bool:
        if target_mode == MODE_MIT:
            self.log(
                "[PREPARE] MIT：使用重力补偿线程维护的MIT命令"
            )
            return True

        if target_mode == MODE_PV:
            if target_dh_q is None:
                self.log("[ERR] PV模式必须提供目标DH关节角")
                return False

            return self.set_pv_target_dh_position(
                target_dh_q,
                pv_velocity_lim,
                tool_target_q,
            )

        if target_mode == MODE_PVT:
            self.set_pvt_hold_current_position()
            return True

        return False

    def wait_arm_feedback(
        self,
        timeout_s: float = 2.0,
    ) -> bool:
        assert self.can is not None

        deadline = time.time() + timeout_s

        while time.time() < deadline:
            if all(m.recv_num > 0 for m in self.can.motors):
                return True
            time.sleep(0.005)

        self.log(
            "[WARN] 等待1~6号反馈超时，recv_num="
            + str([m.recv_num for m in self.can.motors])
        )
        return False

    def initialize_system(self) -> bool:
        with self.command_lock:
            if self.initialized:
                self.log("[INFO] 系统已经初始化")
                return True

            self.can = USBCANFD(
                device_index=self.device_index,
                channel_index=self.channel_index,
            )
            self.robot = Robot()

            self.log("[1] 打开 CANFD 设备...")
            if not self.can.open_device():
                self.log("[ERR] 打开 CANFD 设备失败")
                return False

            self.log("[2] 初始化 CANFD 设备...")
            if not self.can.init_device():
                self.log("[ERR] 初始化 CANFD 设备失败")
                self.can.close_device()
                return False

            self.log("[3] 启动 CANFD 通道...")
            if not self.can.start_device():
                self.log("[ERR] 启动 CANFD 通道失败")
                self.can.close_device()
                return False

            time.sleep(POWER_ON_WAIT_S)
            self.can.clearRecvBuffer()

            self.log("[4] 使能 1~7 号电机...")
            if not self.enable_motors_only_before_thread():
                self.log("[ERR] 电机使能失败")
                self.can.close_device()
                return False

            self.log("[5] 初始化MIT零力矩缓存...")
            self.set_all_mit_zero_torque()

            self.log("[6] 启动CANFD连续收发线程...")
            self.can.start_can_thread(1)

            self.log("[7] 等待1~6号机械臂反馈...")
            if not self.wait_arm_feedback(timeout_s=2.0):
                self.log(
                    "[ERR] 1~6号反馈未建立，为避免位置控制使用无效当前位置，"
                    "初始化失败"
                )
                self.can.stop_can()
                return False

            self.log("[8] 启动重力补偿线程...")
            self.gravity_stop_event.clear()
            self.gravity_thread = threading.Thread(
                target=self.gravity_comp_loop,
                name="gravity_comp_loop",
                daemon=True,
            )
            self.gravity_thread.start()

            # Read the actual arm motor mode from feedback objects.
            arm_modes = [m.Mode for m in self.can.motors]
            if arm_modes and all(m == arm_modes[0] for m in arm_modes):
                self.current_mode = arm_modes[0]
            else:
                self.current_mode = None

            self.initialized = True

            self.log("[OK] 初始化完成")
            self.log(
                "[INFO] 机械臂模式只管理1~6号；第7工具电机独立管理"
            )
            self.log(
                "[INFO] PV安全层：DH软限位、单次变化、速度、反馈、协议范围"
            )
            return True

    def switch_mode(
        self,
        target_mode: int,
        target_dh_q: Optional[List[float]],
        pv_velocity_lim: float,
        tool_target_q: Optional[float] = None,
    ) -> bool:
        with self.command_lock:
            if (
                not self.initialized
                or self.can is None
                or self.robot is None
            ):
                self.log("[ERR] 系统未初始化，无法切换模式")
                return False

            if target_mode not in (
                MODE_MIT,
                MODE_PV,
                MODE_PVT,
            ):
                self.log(f"[ERR] 不支持的模式: {target_mode}")
                return False

            if not self.can.IsUpdating:
                self.log(
                    "[ERR] CANFD连续收发线程未启动"
                )
                return False

            target_name = MODE_NAME[target_mode]

            # KEY FIX:
            # Always prepare / update the target command first.
            # Therefore pressing "PV" again while already in PV really updates
            # PV.position_set instead of returning before the new target is
            # written.
            if not self.prepare_command_for_target_mode(
                target_mode,
                target_dh_q,
                pv_velocity_lim,
                tool_target_q,
            ):
                self.log(
                    f"[ERR] {target_name} 目标命令准备失败；"
                    "不会执行模式切换"
                )
                return False

            arm_modes = [m.Mode for m in self.can.motors]

            if (
                arm_modes
                and all(m == target_mode for m in arm_modes)
            ):
                self.current_mode = target_mode
                self.log(
                    f"[OK] 1~6号已经处于 {target_name}；"
                    "本次仅更新目标命令"
                )
                return True

            self.can.motor_mode = [0] * ARM_DOF

            self.log(
                f"[SWITCH] 1~6号机械臂在线切换到 {target_name}；"
                "第7工具电机不参与本次模式切换"
            )

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
                    self.log(
                        f"[OK] 1~6号已切换到 {target_name}"
                    )
                    return True

                time.sleep(0.01)

            modes = [m.Mode for m in self.can.motors]
            self.log(
                f"[ERR] 切换到 {target_name} 超时，"
                f"arm_modes={modes}, "
                f"flag={self.can.mode_switch_flag}"
            )
            self.can.mode_switch_flag = 0
            return False

    # ============================================================
    # Stop / cleanup
    # ============================================================
    def disable_and_stop(self) -> bool:
        with self.command_lock:
            self.log(
                "[SAFE] 停止重力补偿、清零MIT缓存并失能全部电机"
            )

            self.gravity_stop_event.set()

            if (
                self.gravity_thread is not None
                and self.gravity_thread.is_alive()
            ):
                self.gravity_thread.join(timeout=1.0)

            try:
                self.set_all_mit_zero_torque()
                time.sleep(0.05)
            except Exception as e:
                self.log(
                    f"[WARN] 清零MIT缓存异常: {e}"
                )

            try:
                self.disable_motors_only_at_exit()
            except Exception as e:
                self.log(f"[WARN] 失能异常: {e}")

            try:
                if self.can is not None:
                    self.can.stop_can()
            except Exception as e:
                self.log(f"[WARN] stop_can异常: {e}")

            self.initialized = False
            self.current_mode = None
            self.log("[OK] 已停止并失能")
            return True

    def cleanup(self):
        with self.command_lock:
            self.log("[CLEANUP] 程序退出清理...")

            self.gravity_stop_event.set()

            if (
                self.gravity_thread is not None
                and self.gravity_thread.is_alive()
            ):
                self.gravity_thread.join(timeout=1.0)

            try:
                self.set_all_mit_zero_torque()
                time.sleep(0.05)
            except Exception as e:
                self.log(
                    f"[WARN] 退出清零MIT缓存异常: {e}"
                )

            if self.disable_on_exit:
                try:
                    self.disable_motors_only_at_exit()
                except Exception as e:
                    self.log(
                        f"[WARN] 退出失能异常: {e}"
                    )

            try:
                if self.can is not None:
                    self.can.stop_can()
                    self.can.close_device()
            except Exception as e:
                self.log(
                    f"[WARN] 关闭CANFD异常: {e}"
                )

            self.initialized = False
            self.current_mode = None
            self.log("[END] 清理完成")

    # ============================================================
    # Gravity compensation
    # ============================================================
    def gravity_comp_loop(self):
        assert self.can is not None
        assert self.robot is not None

        self.log("[GRAVITY] 重力补偿线程已启动")

        while not self.gravity_stop_event.is_set():
            try:
                with self.data_lock:
                    self.robot.Angle = self.robot.motor2dh(
                        self.can.motors
                    )

                    if not self.robot.set_robot():
                        time.sleep(GRAVITY_COMP_PERIOD_S)
                        continue

                    tau_g_motor = self.robot.Tau_G_Motor

                    for i, motor in enumerate(self.can.motors):
                        motor.MIT.position_set = 0.0
                        motor.MIT.velocity_set = 0.0
                        motor.MIT.kp_set = 0.0
                        motor.MIT.kd_set = 0.0
                        motor.MIT.torque_set = float(
                            tau_g_motor[i]
                            * GRAVITY_TORQUE_SCALE[i]
                        )
                        self.pack_command_for_mode(
                            motor, MODE_MIT
                        )

                time.sleep(GRAVITY_COMP_PERIOD_S)

            except Exception as e:
                self.log(
                    f"[ERR] 重力补偿线程异常: {e}"
                )
                time.sleep(0.01)

        self.log("[GRAVITY] 重力补偿线程退出")


class MainWindow(QMainWindow):
    command_done_signal = Signal(str, bool)

    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "七电机控制界面 - 独立PV安全层版本"
        )
        self.resize(1320, 820)

        self.controller = ArmController()
        self.controller.log_signal.connect(self.append_log)
        self.command_done_signal.connect(self.on_command_done)

        self.command_running = False

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(200)

    def _build_ui(self):
        central = QWidget()
        main_layout = QVBoxLayout(central)

        splitter = QSplitter(Qt.Vertical)

        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        self._build_connection_group(left_layout)
        self._build_mode_group(left_layout)
        self._build_pv_target_group(left_layout)
        self._build_safety_group(left_layout)
        self._build_options_group(left_layout)

        left_layout.addStretch(1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self._build_status_group(right_layout)

        top_layout.addWidget(left_panel, 0)
        top_layout.addWidget(right_panel, 1)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        splitter.addWidget(top_widget)
        splitter.addWidget(self.log_box)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)
        self.setCentralWidget(central)

    def _build_connection_group(self, parent_layout):
        group = QGroupBox("设备与安全停止")
        layout = QGridLayout(group)

        self.btn_init = QPushButton("初始化并启动控制")
        self.btn_disable = QPushButton("安全失能并停止")
        self.btn_status = QPushButton("刷新状态")
        self.btn_exit = QPushButton("退出程序")

        self.btn_init.clicked.connect(
            lambda: self.run_async(
                "初始化",
                self.controller.initialize_system,
            )
        )
        self.btn_disable.clicked.connect(
            lambda: self.run_async(
                "安全失能",
                self.controller.disable_and_stop,
            )
        )
        self.btn_status.clicked.connect(self.refresh_status)
        self.btn_exit.clicked.connect(self.close)

        layout.addWidget(self.btn_init, 0, 0, 1, 2)
        layout.addWidget(self.btn_disable, 1, 0, 1, 2)
        layout.addWidget(self.btn_status, 2, 0)
        layout.addWidget(self.btn_exit, 2, 1)

        parent_layout.addWidget(group)

    def _build_mode_group(self, parent_layout):
        group = QGroupBox("1~6号机械臂模式")
        layout = QGridLayout(group)

        self.btn_mit = QPushButton(
            "MIT + 重力补偿"
        )
        self.btn_pv = QPushButton(
            "PV：移动到输入目标"
        )
        self.btn_pvt = QPushButton(
            "PVT：当前位置保持"
        )

        self.btn_mit.clicked.connect(
            lambda: self.switch_mode_async(MODE_MIT)
        )
        self.btn_pv.clicked.connect(
            lambda: self.switch_mode_async(MODE_PV)
        )
        self.btn_pvt.clicked.connect(
            lambda: self.switch_mode_async(MODE_PVT)
        )

        layout.addWidget(self.btn_mit, 0, 0, 1, 2)
        layout.addWidget(self.btn_pv, 1, 0, 1, 2)
        layout.addWidget(self.btn_pvt, 2, 0, 1, 2)

        parent_layout.addWidget(group)

    def _build_pv_target_group(self, parent_layout):
        group = QGroupBox("PV目标 DH 关节角")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("rad"), 0, 1)
        layout.addWidget(QLabel("deg"), 0, 2)

        self.q_spin = []
        self.q_deg_label = []

        for i, value in enumerate(DEFAULT_PV_TARGET_DH_Q):
            label = QLabel(f"q{i + 1}")

            spin = QDoubleSpinBox()
            spin.setRange(-math.pi, math.pi)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setValue(float(value))
            spin.valueChanged.connect(
                self.update_target_deg_labels
            )

            deg_label = QLabel("0.00")

            self.q_spin.append(spin)
            self.q_deg_label.append(deg_label)

            layout.addWidget(label, i + 1, 0)
            layout.addWidget(spin, i + 1, 1)
            layout.addWidget(deg_label, i + 1, 2)

        self.check_tool_target = QCheckBox(
            "同时设置第7工具电机PVT目标（默认关闭）"
        )
        self.check_tool_target.setChecked(False)
        layout.addWidget(self.check_tool_target, 7, 0)

        self.tool_spin = QDoubleSpinBox()
        self.tool_spin.setRange(-2.35, 2.35)
        self.tool_spin.setDecimals(4)
        self.tool_spin.setSingleStep(0.01)
        self.tool_spin.setValue(DEFAULT_TOOL_TARGET_Q)
        self.tool_spin.valueChanged.connect(
            self.update_target_deg_labels
        )
        layout.addWidget(self.tool_spin, 7, 1)

        self.tool_deg_label = QLabel("0.00")
        layout.addWidget(self.tool_deg_label, 7, 2)

        layout.addWidget(QLabel("PV速度限制"), 8, 0)

        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(0.01, 1.0)
        self.vel_spin.setDecimals(3)
        self.vel_spin.setSingleStep(0.05)
        self.vel_spin.setValue(DEFAULT_PV_MOVE_VEL_LIM)
        layout.addWidget(self.vel_spin, 8, 1)

        self.btn_use_current = QPushButton(
            "读取当前 DH 角作为 PV 目标"
        )
        self.btn_use_current.clicked.connect(
            self.use_current_dh_as_target
        )
        layout.addWidget(
            self.btn_use_current,
            9, 0, 1, 3,
        )

        self.update_target_deg_labels()
        parent_layout.addWidget(group)

    def _build_safety_group(self, parent_layout):
        group = QGroupBox("PV软件安全层")
        layout = QGridLayout(group)

        self.check_pv_safety = QCheckBox(
            "启用PV软件安全检查"
        )
        self.check_pv_safety.setChecked(True)
        layout.addWidget(
            self.check_pv_safety,
            0, 0, 1, 2,
        )

        layout.addWidget(
            QLabel("单次最大DH变化(rad)"),
            1, 0,
        )

        self.max_delta_spin = QDoubleSpinBox()
        self.max_delta_spin.setRange(0.10, math.pi)
        self.max_delta_spin.setDecimals(3)
        self.max_delta_spin.setSingleStep(0.1)
        self.max_delta_spin.setValue(
            self.controller.safety.max_delta_rad
        )
        layout.addWidget(
            self.max_delta_spin,
            1, 1,
        )

        note = QLabel(
            "说明：DH软件范围统一为[-π, π]；"
            "这不是机械硬限位。真实机械限位需要实机标定后再填写。"
        )
        note.setWordWrap(True)
        layout.addWidget(note, 2, 0, 1, 2)

        parent_layout.addWidget(group)

    def _build_options_group(self, parent_layout):
        group = QGroupBox("退出选项")
        layout = QVBoxLayout(group)

        self.check_disable_exit = QCheckBox(
            "退出程序时失能全部7个电机"
        )
        self.check_disable_exit.setChecked(True)

        layout.addWidget(self.check_disable_exit)
        parent_layout.addWidget(group)

    def _build_status_group(self, parent_layout):
        group = QGroupBox("实时状态")
        layout = QVBoxLayout(group)

        self.mode_label = QLabel(
            "1~6号当前目标模式：未知"
        )
        self.tool_mode_label = QLabel(
            "第7工具模式：未知"
        )
        self.updating_label = QLabel(
            "CANFD线程：未启动"
        )

        layout.addWidget(self.mode_label)
        layout.addWidget(self.tool_mode_label)
        layout.addWidget(self.updating_label)

        self.table = QTableWidget(
            TOTAL_MOTOR_NUM, 10
        )
        self.table.setHorizontalHeaderLabels(
            [
                "ID",
                "Mode",
                "Enable",
                "ERR",
                "Pos(rad)",
                "Vel",
                "Torque",
                "Recv",
                "DH(rad)",
                "DH(deg)",
            ]
        )
        self.table.verticalHeader().setVisible(False)

        for r in range(TOTAL_MOTOR_NUM):
            for c in range(10):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, item)

        layout.addWidget(self.table)

        self.can_param_label = QLabel("CAN参数：")
        layout.addWidget(self.can_param_label)

        parent_layout.addWidget(group)

    def append_log(self, msg: str):
        self.log_box.append(msg)
        self.log_box.moveCursor(QTextCursor.End)

    def set_buttons_enabled(self, enabled: bool):
        for button in (
            self.btn_init,
            self.btn_disable,
            self.btn_mit,
            self.btn_pv,
            self.btn_pvt,
            self.btn_status,
            self.btn_exit,
            self.btn_use_current,
        ):
            button.setEnabled(enabled)

    def run_async(self, label: str, func):
        if self.command_running:
            self.append_log(
                "[WARN] 上一个命令仍在执行，请稍后再操作"
            )
            return

        self.command_running = True
        self.set_buttons_enabled(False)

        def worker():
            ok = False
            try:
                ok = bool(func())
            except Exception as e:
                self.controller.log(
                    f"[ERR] {label}异常: {e}"
                )
                ok = False
            finally:
                self.command_done_signal.emit(
                    label, ok
                )

        threading.Thread(
            target=worker,
            name=f"cmd_{label}",
            daemon=True,
        ).start()

    def on_command_done(self, label: str, ok: bool):
        self.command_running = False
        self.set_buttons_enabled(True)
        self.append_log(
            f"[DONE] {label} {'成功' if ok else '失败'}"
        )
        self.refresh_status()

    def update_controller_options(self):
        self.controller.disable_on_exit = (
            self.check_disable_exit.isChecked()
        )
        self.controller.safety.enabled = (
            self.check_pv_safety.isChecked()
        )
        self.controller.safety.max_delta_rad = float(
            self.max_delta_spin.value()
        )

    def update_target_deg_labels(self):
        for spin, label in zip(
            self.q_spin,
            self.q_deg_label,
        ):
            label.setText(
                f"{spin.value() * 180.0 / math.pi:.2f}"
            )

        self.tool_deg_label.setText(
            f"{self.tool_spin.value() * 180.0 / math.pi:.2f}"
        )

    def get_target_dh_q_from_ui(self) -> List[float]:
        return [
            float(spin.value())
            for spin in self.q_spin
        ]

    def switch_mode_async(self, mode: int):
        self.update_controller_options()

        target_dh_q: Optional[List[float]] = None
        tool_target_q: Optional[float] = None
        pv_vel = float(self.vel_spin.value())

        if mode == MODE_PV:
            target_dh_q = (
                self.get_target_dh_q_from_ui()
            )

            if self.check_tool_target.isChecked():
                tool_target_q = float(
                    self.tool_spin.value()
                )

        def command():
            return self.controller.switch_mode(
                mode,
                target_dh_q,
                pv_vel,
                tool_target_q,
            )

        self.run_async(
            f"切换/更新 {MODE_NAME[mode]}",
            command,
        )

    def use_current_dh_as_target(self):
        snapshot = self.controller.get_status_snapshot()

        if snapshot is None:
            QMessageBox.warning(
                self,
                "提示",
                "当前无法读取DH角，请先初始化并等待反馈。",
            )
            return

        q_rad = snapshot.get("dh_rad")

        if q_rad is None or len(q_rad) != ARM_DOF:
            QMessageBox.warning(
                self,
                "提示",
                "当前DH关节角无效。",
            )
            return

        for i in range(ARM_DOF):
            self.q_spin[i].setValue(
                float(q_rad[i])
            )

        self.update_target_deg_labels()
        self.append_log(
            "[UI] 已将当前DH关节角填入PV目标"
        )

    def refresh_status(self):
        snapshot = self.controller.get_status_snapshot()
        if snapshot is None:
            return

        mode = snapshot.get("current_mode")

        if mode is None:
            self.mode_label.setText(
                "1~6号当前目标模式：未知/不一致"
            )
        else:
            self.mode_label.setText(
                "1~6号当前目标模式："
                + MODE_NAME.get(mode, str(mode))
            )

        tool_modes = snapshot.get("tool_modes", [])
        if tool_modes:
            self.tool_mode_label.setText(
                f"第7工具模式：{tool_modes[0]}"
            )

        self.updating_label.setText(
            "CANFD线程："
            + (
                "运行中"
                if snapshot.get("is_updating")
                else "未运行"
            )
        )

        motors = snapshot.get("motors", [])
        dh_rad = snapshot.get(
            "dh_rad", [0.0] * ARM_DOF
        )
        dh_deg = snapshot.get(
            "dh_deg", [0.0] * ARM_DOF
        )

        for r in range(
            min(TOTAL_MOTOR_NUM, len(motors))
        ):
            m = motors[r]

            dh_rad_text = (
                f"{dh_rad[r]:.4f}"
                if r < ARM_DOF
                else "-"
            )
            dh_deg_text = (
                f"{dh_deg[r]:.2f}"
                if r < ARM_DOF
                else "-"
            )

            values = [
                str(m["id"]),
                str(m["mode"]),
                str(m["enable"]),
                str(m["err"]),
                f"{m['pos']:.4f}",
                f"{m['vel']:.4f}",
                f"{m['tau']:.4f}",
                str(m["recv"]),
                dh_rad_text,
                dh_deg_text,
            ]

            for c, text in enumerate(values):
                self.table.item(r, c).setText(text)

        can_param = snapshot.get("can_param", [])
        if can_param:
            self.can_param_label.setText(
                "CAN参数："
                + " | ".join(
                    f"{i}:{float(v):.2f}"
                    for i, v in enumerate(can_param)
                )
            )

    def closeEvent(self, event):
        if self.command_running:
            QMessageBox.warning(
                self,
                "提示",
                "当前命令仍在执行，请等待完成后再退出。",
            )
            event.ignore()
            return

        reply = QMessageBox.question(
            self,
            "确认退出",
            "是否退出程序？\n"
            "如果勾选退出失能，程序会先尝试失能全部7个电机。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply != QMessageBox.Yes:
            event.ignore()
            return

        self.timer.stop()
        self.update_controller_options()

        try:
            self.controller.cleanup()
        except Exception as e:
            self.append_log(
                f"[WARN] 退出清理异常: {e}"
            )

        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
