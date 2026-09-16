"""
不使能版：将第 7 个工具电机当前位置设置为新的 0 rad 零位。

特点：
1. 只打开 / 初始化 / 启动 CANFD 通道；
2. 不清错；
3. 不使能电机；
4. 不启动 CAN 连续发送线程；
5. 不启动重力补偿线程；
6. 不调用 USBCANFD.set_zero()，避免其后续发送空 PV/MIT/PVT 命令；
7. 只向 ID=7 发送 DMMotor.set_zero_command。

运行示例：
    python set_motor7_zero_no_enable.py --device-index 0
    python set_motor7_zero_no_enable.py --device-index 1 --yes

注意：
- 本脚本设置的是第 7 个工具电机，不会设置 1~6 号机械臂电机。
- 设置零位不是让工具电机运动到 0 rad，而是把当前机械位置定义为新的 0 rad。
- 修改工具电机零位会影响工具电机 PV 目标、遥操作映射、记录和回放中的工具电机角度。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from DMMotor import DMMotor
from USBCANFD import USBCANFD

TARGET_MOTOR_ID = 7


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="No-enable zero setting for tool motor 7."
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="USBCANFD device_index，默认 0。主端/从端请按实际设备索引选择。",
    )
    parser.add_argument(
        "--channel-index",
        type=int,
        default=0,
        help="CAN channel_index，默认 0。",
    )
    parser.add_argument(
        "--timeout-ms",
        type=float,
        default=100.0,
        help="等待电机回复的超时时间，单位 ms，默认 100。",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=1,
        help="set_zero 命令发送次数，默认 1。通信偶发无回复时可设为 2 或 3。",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过交互确认。确认你已经知道风险后再使用。",
    )
    return parser.parse_args()


def confirm_or_exit(args: argparse.Namespace) -> None:
    if args.yes:
        return

    print()
    print("=" * 76)
    print("不使能版：准备将第 7 个工具电机当前位置设置为新的 0 rad")
    print(f"device_index = {args.device_index}")
    print(f"channel_index = {args.channel_index}")
    print(f"motor_id      = {TARGET_MOTOR_ID}")
    print("=" * 76)
    print("本脚本不会清错、不会使能、不会启动连续发送线程。")
    print("请确认：")
    print("  1. 工具电机已经停止运动；")
    print("  2. 没有其他 GUI/脚本正在占用 CANFD；")
    print("  3. 第 7 工具电机当前位置就是你希望定义的 0 rad；")
    print("  4. 你理解该操作会影响工具电机 PV 目标、遥操作映射、记录和回放。")
    print()
    text = input("确认继续请输入 YES：").strip()
    if text != "YES":
        print("已取消设置零位。")
        sys.exit(0)


def open_can(device_index: int, channel_index: int) -> USBCANFD:
    can = USBCANFD(device_index=device_index, channel_index=channel_index)

    print("[1] 打开 CANFD 设备...")
    if not can.open_device():
        raise RuntimeError("打开 CANFD 设备失败")

    print("[2] 初始化 CANFD 通道...")
    if not can.init_device():
        try:
            can.close_device()
        finally:
            pass
        raise RuntimeError("初始化 CANFD 通道失败")

    print("[3] 启动 CANFD 通道...")
    if not can.start_device():
        try:
            can.close_device()
        finally:
            pass
        raise RuntimeError("启动 CANFD 通道失败")

    try:
        can.stop_can()
        can.clearRecvBuffer()
    except Exception as exc:
        print(f"[WARN] 清空接收缓冲区异常: {exc}")

    return can


def get_tool_motor(can: USBCANFD, motor_id: int) -> DMMotor:
    for tool in getattr(can, "tools", []):
        if int(tool.ID) == int(motor_id):
            return tool
    raise RuntimeError(f"未找到工具电机 ID={motor_id}")


def parse_and_print_reply(prefix: str, motor: DMMotor, data: Optional[bytes]) -> bool:
    if data is None:
        print(f"{prefix} 未收到电机回复。")
        return False

    ok = motor.read_motor(data)
    raw = " ".join(f"{b:02X}" for b in data[:8])
    print(f"{prefix} 原始回复: {raw}")

    if not ok:
        print(f"{prefix} 回复解析失败，可能不是目标工具电机反馈。")
        return False

    print(
        f"{prefix} 工具电机 {motor.ID}: "
        f"Position={motor.Position:.6f} rad, "
        f"Velocity={motor.Velocity:.6f}, "
        f"Torque={motor.Torque:.6f}, "
        f"Enable={motor.Enable}, ERR={motor.ERRCODE}, recv={motor.recv_num}"
    )
    return True


def send_zero_no_enable(can: USBCANFD, motor_id: int, timeout_ms: float, retries: int) -> bool:
    motor = get_tool_motor(can, motor_id)
    success = False

    print(f"[4] 不使能，直接向工具电机 {motor_id} 发送 set_zero_command...")
    for attempt in range(1, max(1, int(retries)) + 1):
        print(f"[SEND] 第 {attempt} 次发送 set_zero_command")
        # send_wait 内部只会 stop_can、clearRecvBuffer、发送一次 CANFD、等待回复；
        # 这里没有 enable、没有 clear_error、没有连续发送线程。
        data = can.send_wait(1, motor_id, DMMotor.set_zero_command, float(timeout_ms))
        ok = parse_and_print_reply("[REPLY]", motor, data)
        success = success or ok
        if ok and abs(float(motor.Position)) < 0.05:
            break
        time.sleep(0.05)

    return success


def main() -> int:
    args = parse_args()
    confirm_or_exit(args)

    can: Optional[USBCANFD] = None
    try:
        can = open_can(args.device_index, args.channel_index)
        ok = send_zero_no_enable(
            can=can,
            motor_id=TARGET_MOTOR_ID,
            timeout_ms=args.timeout_ms,
            retries=args.retries,
        )

        print()
        if ok:
            tool = get_tool_motor(can, TARGET_MOTOR_ID)
            pos = float(tool.Position)
            if abs(pos) < 0.05:
                print("[OK] 已发送零位设置命令，且第 7 工具电机反馈 Position 接近 0 rad。")
            else:
                print("[WARN] 已收到第 7 工具电机回复，但反馈 Position 未接近 0 rad。")
                print("[WARN] 若电机处于失能状态不返回更新后的角度，可重新运行 GUI 查看当前位置。")
        else:
            print("[WARN] 已发送零位设置命令，但没有解析到有效回复。")
            print("[WARN] 有些固件在失能状态下可能不回复或不接受 set_zero；请重新打开 GUI 检查零位是否变化。")

        return 0 if ok else 2

    except KeyboardInterrupt:
        print("\n[WARN] 用户中断。")
        return 130
    except Exception as exc:
        print(f"[ERR] 设置第 7 工具电机零位失败: {exc}")
        return 1
    finally:
        if can is not None:
            print("[5] 关闭 CANFD 设备...")
            try:
                can.stop_can()
                can.close_device()
            except Exception as exc:
                print(f"[WARN] 关闭设备异常: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
