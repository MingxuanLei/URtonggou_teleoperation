#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CTAG2F90D 夹爪开度控制程序。

依赖同目录下的 GripperController.py。

开度定义：
    0.0 = 完全闭合
    1.0 = 最大张开
    0.5 = 约一半开度

默认将夹爪移动到 50% 开度，也可在命令行指定：
    python gripper_opening_control.py --opening 0.3
    python gripper_opening_control.py --opening 0.8
"""

import argparse
import math
import sys
import time
from typing import Optional

from GripperController import GripperController


GRIPPER_PORT = "192.168.3.15:54321"
SLAVE_ID = 1
CONNECTION_TYPE = "tcp"
CONTROL_INTERVAL_S = 0.1

DEFAULT_OPENING = 0.5
DEFAULT_SPEED = 20
DEFAULT_FORCE = 25
DEFAULT_ACCEL = 20
DEFAULT_DECEL = 20

OPENING_TOLERANCE = 0.02
MOVE_TIMEOUT_S = 8.0
FEEDBACK_TIMEOUT_S = 3.0


def finite_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{value!r} 不是有效数字。") from exc
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("参数必须是有限数字。")
    return number


def opening_value(value: str) -> float:
    number = finite_float(value)
    if not 0.0 <= number <= 1.0:
        raise argparse.ArgumentTypeError(
            "开度必须位于0.0～1.0之间：0.0=完全闭合，1.0=最大张开。"
        )
    return number


def ranged_value(value: str, name: str, minimum: float, maximum: float) -> float:
    number = finite_float(value)
    if not minimum <= number <= maximum:
        raise argparse.ArgumentTypeError(
            f"{name} 必须位于 {minimum}～{maximum} 之间。"
        )
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="控制CTAG2F90D夹爪移动到指定开度。"
    )
    parser.add_argument(
        "--opening",
        type=opening_value,
        default=DEFAULT_OPENING,
        help=f"目标开度0.0～1.0，默认{DEFAULT_OPENING}。",
    )
    parser.add_argument(
        "--speed",
        type=lambda v: ranged_value(v, "speed", 0, 100),
        default=DEFAULT_SPEED,
        help=f"目标速度0～100，默认{DEFAULT_SPEED}。",
    )
    parser.add_argument(
        "--force",
        type=lambda v: ranged_value(v, "force", 0, 100),
        default=DEFAULT_FORCE,
        help=f"目标夹持力0～100，默认{DEFAULT_FORCE}。",
    )
    parser.add_argument(
        "--accel",
        type=lambda v: ranged_value(v, "accel", 0, 1000),
        default=DEFAULT_ACCEL,
        help=f"目标加速度0～1000，默认{DEFAULT_ACCEL}。",
    )
    parser.add_argument(
        "--decel",
        type=lambda v: ranged_value(v, "decel", 0, 1000),
        default=DEFAULT_DECEL,
        help=f"目标减速度0～1000，默认{DEFAULT_DECEL}。",
    )
    return parser


def wait_for_initial_feedback(gripper: GripperController, timeout_s: float) -> None:
    """等待后台线程获得第一帧真实反馈。"""
    initial_feedback = gripper.feedback
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        if not gripper.is_running:
            raise RuntimeError("夹爪后台控制线程意外停止。")
        if gripper.feedback is not initial_feedback:
            return
        time.sleep(0.02)

    raise TimeoutError(
        f"在{timeout_s:.1f}秒内没有收到夹爪反馈，请检查54321端口、"
        "Tool Communication Forwarder、RS-485接线及夹爪供电。"
    )


def wait_for_target_opening(
    gripper: GripperController,
    target_opening: float,
    tolerance: float,
    timeout_s: float,
) -> None:
    """根据实际开度反馈等待夹爪到达目标，并提供超时保护。"""
    deadline = time.monotonic() + timeout_s
    last_print_time = 0.0

    while True:
        if not gripper.is_running:
            raise RuntimeError("夹爪后台控制线程意外停止。")

        feedback = gripper.feedback
        actual_opening = float(feedback.open)
        error = abs(actual_opening - target_opening)
        now = time.monotonic()

        if now - last_print_time >= 0.25:
            print(
                "反馈："
                f"open={actual_opening:.3f}, "
                f"position={feedback.position}, "
                f"speed={feedback.speed}, "
                f"current={feedback.current}, "
                f"pos_reached={feedback.pos_reached}"
            )
            last_print_time = now

        if error <= tolerance:
            print(
                f"目标已到达：目标开度={target_opening:.3f}，"
                f"实际开度={actual_opening:.3f}，误差={error:.3f}。"
            )
            return

        if now >= deadline:
            raise TimeoutError(
                f"夹爪在{timeout_s:.1f}秒内未到达目标开度。"
                f"目标={target_opening:.3f}，当前={actual_opening:.3f}。"
            )

        time.sleep(0.02)


def main() -> int:
    args = build_parser().parse_args()
    gripper: Optional[GripperController] = None

    try:
        print("正在连接CTAG2F90D夹爪……")
        print(f"通信地址：{GRIPPER_PORT}")
        print("开度定义：0.0=完全闭合，1.0=最大张开。")
        print(
            f"目标参数：opening={args.opening:.3f}, "
            f"speed={args.speed:.0f}, force={args.force:.0f}, "
            f"accel={args.accel:.0f}, decel={args.decel:.0f}"
        )

        gripper = GripperController(
            port=GRIPPER_PORT,
            slave_id=SLAVE_ID,
            connection_type=CONNECTION_TYPE,
            timeout=0.5,
            debug=False,
        )
        gripper.start(interval=CONTROL_INTERVAL_S)

        print("正在等待夹爪反馈……")
        wait_for_initial_feedback(gripper, FEEDBACK_TIMEOUT_S)
        print(
            f"当前反馈：open={gripper.feedback.open:.3f}, "
            f"position={gripper.feedback.position}"
        )

        print(f"开始移动到目标开度 {args.opening:.3f}……")
        gripper.move(
            position=args.opening,
            speed=args.speed,
            force=args.force,
            accel=args.accel,
            decel=args.decel,
            block=False,
        )

        wait_for_target_opening(
            gripper,
            target_opening=args.opening,
            tolerance=OPENING_TOLERANCE,
            timeout_s=MOVE_TIMEOUT_S,
        )

        print("夹爪开度控制完成。")
        return 0

    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在关闭通信。", file=sys.stderr)
        print(
            "注意：GripperController.stop()只停止通信线程，"
            "不等价于夹爪物理急停。",
            file=sys.stderr,
        )
        return 130

    except Exception as error:
        print(
            f"\n执行失败：{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    finally:
        if gripper is not None:
            try:
                gripper.close()
            except Exception as close_error:
                print(
                    f"关闭夹爪通信时发生异常：{close_error}",
                    file=sys.stderr,
                )
        print("夹爪通信已关闭。")


if __name__ == "__main__":
    raise SystemExit(main())
