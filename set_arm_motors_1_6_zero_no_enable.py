"""
不使能版：将机械臂 1~6 号电机当前位置设置为新的 0 rad 零位。

特点：
1. 只打开 / 初始化 / 启动 CANFD 通道；
2. 不清错；
3. 不使能电机；
4. 不启动 CAN 连续发送线程；
5. 不启动重力补偿线程；
6. 不调用 USBCANFD.set_zero()，避免其后续发送空 PV/MIT/PVT 命令；
7. 只向 1~6 号电机逐个发送 DMMotor.set_zero_command。

运行示例：
    python set_arm_motors_1_6_zero_no_enable.py --device-index 0
    python set_arm_motors_1_6_zero_no_enable.py --device-index 1 --yes

可选：只设置某几个机械臂电机，例如只设置 5、6 号：
    python set_arm_motors_1_6_zero_no_enable.py --device-index 0 --motor-ids 5,6

注意：
- 本脚本设置的是 1~6 号机械臂电机，不设置第 7 个工具电机。
- 设置零位不是让电机运动到 0 rad，而是把当前机械位置定义为新的 0 rad。
- 修改 1~6 号电机零位会影响 DH 角显示、重力补偿、PV 目标转换、遥操作映射、轨迹记录/回放等。
- 请仅在机械臂处于明确标定姿态、并确认需要重定义电机零位时使用。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Sequence

from DMMotor import DMMotor
from USBCANFD import USBCANFD

DEFAULT_TARGET_MOTOR_IDS = [1, 2, 3, 4, 5, 6]


def parse_motor_ids(text: str) -> list[int]:
    ids: list[int] = []
    for part in str(text).replace("，", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            motor_id = int(part)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"非法电机 ID: {part!r}") from exc
        if motor_id < 1 or motor_id > 6:
            raise argparse.ArgumentTypeError("本脚本只允许设置 1~6 号机械臂电机零位")
        ids.append(motor_id)

    # 去重但保持输入顺序
    unique_ids: list[int] = []
    for motor_id in ids:
        if motor_id not in unique_ids:
            unique_ids.append(motor_id)

    if not unique_ids:
        raise argparse.ArgumentTypeError("至少需要指定一个电机 ID")
    return unique_ids


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="No-enable zero setting for arm motors 1~6."
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
        "--motor-ids",
        type=parse_motor_ids,
        default=DEFAULT_TARGET_MOTOR_IDS,
        help="要设置零位的机械臂电机 ID，默认 1,2,3,4,5,6；例如 --motor-ids 5,6。",
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
        help="每个电机 set_zero 命令发送次数，默认 1。通信偶发无回复时可设为 2 或 3。",
    )
    parser.add_argument(
        "--delay-s",
        type=float,
        default=0.08,
        help="不同电机之间的发送间隔，单位 s，默认 0.08。",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过交互确认。确认你已经知道风险后再使用。",
    )
    return parser.parse_args()


def confirm_or_exit(args: argparse.Namespace) -> None:
    motor_ids = list(args.motor_ids)
    if args.yes:
        return

    print()
    print("=" * 86)
    print("不使能版：准备将机械臂电机当前位置设置为新的 0 rad 零位")
    print(f"device_index = {args.device_index}")
    print(f"channel_index = {args.channel_index}")
    print(f"motor_ids     = {motor_ids}")
    print("=" * 86)
    print("本脚本不会清错、不会使能、不会启动连续发送线程。")
    print("请确认：")
    print("  1. 机械臂已经停止运动；")
    print("  2. 没有其他 GUI/脚本正在占用 CANFD；")
    print("  3. 这些电机当前机械位置就是你希望定义的 0 rad；")
    print("  4. 你理解该操作会影响 DH 角、重力补偿、PV 定位、遥操作、记录和回放；")
    print("  5. 如果只是想设置第 7 工具电机零位，请不要使用本脚本。")
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


def parse_and_print_reply(prefix: str, motor: DMMotor, data: Optional[bytes]) -> bool:
    if data is None:
        print(f"{prefix} 未收到电机回复。")
        return False

    ok = motor.read_motor(data)
    raw = " ".join(f"{b:02X}" for b in data[:8])
    print(f"{prefix} 原始回复: {raw}")

    if not ok:
        print(f"{prefix} 回复解析失败，可能不是目标电机反馈。")
        return False

    print(
        f"{prefix} 电机 {motor.ID}: "
        f"Position={motor.Position:.6f} rad, "
        f"Velocity={motor.Velocity:.6f}, "
        f"Torque={motor.Torque:.6f}, "
        f"Enable={motor.Enable}, ERR={motor.ERRCODE}, recv={motor.recv_num}"
    )
    return True


def send_zero_to_one_motor_no_enable(
    can: USBCANFD,
    motor_id: int,
    timeout_ms: float,
    retries: int,
) -> bool:
    if motor_id < 1 or motor_id > can.MOTOR_NUM:
        raise ValueError(f"motor_id={motor_id} 不是 1~{can.MOTOR_NUM} 号机械臂电机")

    motor = can.motors[motor_id - 1]
    success = False

    print()
    print("-" * 78)
    print(f"[SEND] 不使能，直接向机械臂电机 {motor_id} 发送 set_zero_command")

    for attempt in range(1, max(1, int(retries)) + 1):
        print(f"[SEND] 电机 {motor_id}: 第 {attempt} 次发送 set_zero_command")
        # send_wait 内部只会 stop_can、clearRecvBuffer、发送一次 CANFD、等待回复；
        # 这里没有 enable、没有 clear_error、没有连续发送线程。
        data = can.send_wait(1, motor_id, DMMotor.set_zero_command, float(timeout_ms))
        ok = parse_and_print_reply(f"[REPLY M{motor_id}]", motor, data)
        success = success or ok
        if ok and abs(float(motor.Position)) < 0.05:
            break
        time.sleep(0.05)

    return success


def send_zero_no_enable(
    can: USBCANFD,
    motor_ids: Sequence[int],
    timeout_ms: float,
    retries: int,
    delay_s: float,
) -> dict[int, bool]:
    results: dict[int, bool] = {}
    print(f"[4] 准备按顺序设置机械臂电机零位: {list(motor_ids)}")

    for motor_id in motor_ids:
        ok = send_zero_to_one_motor_no_enable(
            can=can,
            motor_id=int(motor_id),
            timeout_ms=timeout_ms,
            retries=retries,
        )
        results[int(motor_id)] = bool(ok)
        time.sleep(max(0.0, float(delay_s)))

    return results


def print_summary(can: USBCANFD, results: dict[int, bool]) -> None:
    print()
    print("=" * 86)
    print("零位设置结果汇总")
    print("=" * 86)
    for motor_id, ok in results.items():
        motor = can.motors[motor_id - 1]
        pos = float(motor.Position)
        if ok and abs(pos) < 0.05:
            status = "OK，反馈 Position 接近 0 rad"
        elif ok:
            status = "WARN，收到回复但 Position 未接近 0 rad"
        else:
            status = "WARN，未解析到有效回复"
        print(
            f"电机 {motor_id}: {status}; "
            f"Position={pos:.6f}, Enable={motor.Enable}, ERR={motor.ERRCODE}"
        )
    print("=" * 86)
    print("如果某些电机处于失能状态不返回更新后的角度，请重新打开 GUI 检查当前位置是否已接近 0。")


def main() -> int:
    args = parse_args()
    confirm_or_exit(args)

    can: Optional[USBCANFD] = None
    try:
        can = open_can(args.device_index, args.channel_index)
        results = send_zero_no_enable(
            can=can,
            motor_ids=args.motor_ids,
            timeout_ms=float(args.timeout_ms),
            retries=int(args.retries),
            delay_s=float(args.delay_s),
        )
        print_summary(can, results)

        # 有回复不代表一定成功写入；但如果无回复，也可能是失能状态不回包。
        # 返回码只反映“是否至少有一个电机给了有效回复”。
        return 0 if all(results.values()) else 2

    except KeyboardInterrupt:
        print("\n[WARN] 用户中断。")
        return 130
    except Exception as exc:
        print(f"[ERR] 设置 1~6 号电机零位失败: {exc}")
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
