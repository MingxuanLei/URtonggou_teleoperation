"""
不使能版：将 1~7 号电机的当前位置设置为新的 0 rad 零位。

特点：
1. 只打开、初始化并启动 CANFD 通道；
2. 不清错、不使能电机；
3. 不启动 CAN 连续发送线程或重力补偿线程；
4. 不调用 USBCANFD.set_zero()，避免随后发送空的 PV/MIT/PVT 命令；
5. 默认依次向 1~7 号电机各发送一次 DMMotor.set_zero_command；
6. 可用 --motor-ids 选择部分电机。

运行示例：
    python set_all_motors_1_7_zero_no_enable.py --device-index 0
    python set_all_motors_1_7_zero_no_enable.py --device-index 1 --yes
    python set_all_motors_1_7_zero_no_enable.py --device-index 0 --motor-ids 1,2,7

注意：
- 设置零位不是让电机运动到 0 rad，而是把当前位置定义为新的 0 rad；
- 修改零位会影响 DH 角、重力补偿、PV 定位、遥操作、轨迹记录和回放；
- 仅在机械臂完全静止、处于明确标定姿态且没有其他程序占用 CANFD 时运行。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Sequence

from DMMotor import DMMotor
from USBCANFD import USBCANFD


DEFAULT_TARGET_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7]


def parse_motor_ids(text: str) -> list[int]:
    """解析逗号分隔的 1~7 号电机 ID，并在保持顺序的同时去重。"""
    motor_ids: list[int] = []

    for part in str(text).replace("，", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            motor_id = int(part)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"非法电机 ID: {part!r}") from exc

        if motor_id < 1 or motor_id > 7:
            raise argparse.ArgumentTypeError("本脚本只允许设置 1~7 号电机零位")
        if motor_id not in motor_ids:
            motor_ids.append(motor_id)

    if not motor_ids:
        raise argparse.ArgumentTypeError("至少需要指定一个电机 ID")
    return motor_ids


def positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"需要正数，实际为: {text!r}") from exc
    if value <= 0.0:
        raise argparse.ArgumentTypeError("数值必须大于 0")
    return value


def positive_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"需要正整数，实际为: {text!r}") from exc
    if value <= 0:
        raise argparse.ArgumentTypeError("数值必须大于 0")
    return value


def nonnegative_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"需要非负数，实际为: {text!r}") from exc
    if value < 0.0:
        raise argparse.ArgumentTypeError("数值不能小于 0")
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="No-enable zero setting for motors 1~7."
    )
    parser.add_argument(
        "--device-index",
        type=int,
        default=0,
        help="USBCANFD device_index，默认 0；主端/从端请按实际设备索引选择。",
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
        help="要设置零位的电机 ID，默认 1,2,3,4,5,6,7。",
    )
    parser.add_argument(
        "--timeout-ms",
        type=positive_float,
        default=100.0,
        help="每次发送后等待电机回复的超时时间，单位 ms，默认 100。",
    )
    parser.add_argument(
        "--retries",
        type=positive_int,
        default=1,
        help="每个电机 set_zero 命令的最多发送次数，默认 1。",
    )
    parser.add_argument(
        "--delay-s",
        type=nonnegative_float,
        default=0.08,
        help="不同电机之间的发送间隔，单位 s，默认 0.08。",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过交互确认；确认已经了解重设零位的风险后再使用。",
    )
    return parser.parse_args()


def confirm_or_exit(args: argparse.Namespace) -> None:
    if args.yes:
        return

    print()
    print("=" * 86)
    print("不使能版：准备将指定电机的当前位置设置为新的 0 rad 零位")
    print(f"device_index = {args.device_index}")
    print(f"channel_index = {args.channel_index}")
    print(f"motor_ids     = {list(args.motor_ids)}")
    print("=" * 86)
    print("本脚本不会清错、不会使能、不会启动连续发送线程或重力补偿线程。")
    print("请确认：")
    print("  1. 1~7 号电机及机械臂已经完全停止运动；")
    print("  2. 没有其他 GUI 或脚本正在占用这台 CANFD 设备；")
    print("  3. 每个目标电机的当前位置都是希望定义的 0 rad；")
    print("  4. 已理解重设零位会影响 DH 角、重力补偿、PV 定位、遥操作、记录和回放。")
    print()

    text = input("确认继续请输入 YES：").strip()
    if text != "YES":
        print("已取消设置零位。")
        raise SystemExit(0)


def open_can(device_index: int, channel_index: int) -> USBCANFD:
    can = USBCANFD(device_index=device_index, channel_index=channel_index)

    print("[1] 打开 CANFD 设备...")
    if not can.open_device():
        raise RuntimeError("打开 CANFD 设备失败")

    print("[2] 初始化 CANFD 通道...")
    if not can.init_device():
        can.close_device()
        raise RuntimeError("初始化 CANFD 通道失败")

    print("[3] 启动 CANFD 通道...")
    if not can.start_device():
        can.close_device()
        raise RuntimeError("启动 CANFD 通道失败")

    # 保证没有连续收发线程运行；后续只使用同步的单次 send_wait。
    can.stop_can()
    if not can.clearRecvBuffer():
        print("[WARN] 清空 CANFD 接收缓冲区失败，仍将尝试发送零位命令。")

    return can


def get_motor(can: USBCANFD, motor_id: int) -> DMMotor:
    if 1 <= motor_id <= can.MOTOR_NUM:
        return can.motors[motor_id - 1]

    for tool in getattr(can, "tools", []):
        if int(tool.ID) == int(motor_id):
            return tool

    raise RuntimeError(f"CAN 控制器中未找到电机 ID={motor_id}")


def parse_and_print_reply(motor: DMMotor, data: Optional[bytes]) -> bool:
    prefix = f"[REPLY M{motor.ID}]"
    if data is None:
        print(f"{prefix} 未收到电机回复。")
        return False

    raw = " ".join(f"{byte:02X}" for byte in data[:8])
    print(f"{prefix} 原始回复: {raw}")

    if not motor.read_motor(data):
        print(f"{prefix} 回复解析失败，可能不是目标电机反馈。")
        return False

    print(
        f"{prefix} Position={motor.Position:.6f} rad, "
        f"Velocity={motor.Velocity:.6f}, Torque={motor.Torque:.6f}, "
        f"Enable={motor.Enable}, ERR={motor.ERRCODE}, recv={motor.recv_num}"
    )
    return True


def send_zero_to_one_motor_no_enable(
    can: USBCANFD,
    motor_id: int,
    timeout_ms: float,
    retries: int,
) -> bool:
    motor = get_motor(can, motor_id)
    success = False

    print()
    print("-" * 78)
    print(f"[SEND] 不使能，直接向电机 {motor_id} 发送 set_zero_command")

    for attempt in range(1, retries + 1):
        print(f"[SEND] 电机 {motor_id}: 第 {attempt} 次发送 set_zero_command")
        # send_wait 只停止连续线程、清接收缓存、发送一次命令并等待回复。
        # 本脚本不会发送 clear_error 或 enable 命令。
        data = can.send_wait(
            1,
            motor_id,
            DMMotor.set_zero_command,
            timeout_ms,
        )
        ok = parse_and_print_reply(motor, data)
        success = success or ok

        if ok and abs(float(motor.Position)) < 0.05:
            break
        if attempt < retries:
            time.sleep(0.05)

    return success


def set_selected_motors_zero(
    can: USBCANFD,
    motor_ids: Sequence[int],
    timeout_ms: float,
    retries: int,
    delay_s: float,
) -> dict[int, bool]:
    results: dict[int, bool] = {}
    ids = [int(motor_id) for motor_id in motor_ids]

    print(f"[4] 准备按顺序设置电机零位: {ids}")
    for index, motor_id in enumerate(ids):
        results[motor_id] = send_zero_to_one_motor_no_enable(
            can=can,
            motor_id=motor_id,
            timeout_ms=timeout_ms,
            retries=retries,
        )
        if index + 1 < len(ids):
            time.sleep(delay_s)

    return results


def print_summary(can: USBCANFD, results: dict[int, bool]) -> None:
    print()
    print("=" * 86)
    print("零位设置结果汇总")
    print("=" * 86)

    for motor_id, reply_ok in results.items():
        motor = get_motor(can, motor_id)
        position = float(motor.Position)
        if reply_ok and abs(position) < 0.05:
            status = "OK，收到有效回复且 Position 接近 0 rad"
        elif reply_ok:
            status = "WARN，收到有效回复但 Position 未接近 0 rad"
        else:
            status = "WARN，未解析到有效回复"

        print(
            f"电机 {motor_id}: {status}; Position={position:.6f}, "
            f"Enable={motor.Enable}, ERR={motor.ERRCODE}"
        )

    print("=" * 86)
    print("注意：某些固件在失能状态下可能不回复，或不会立即返回更新后的角度。")
    print("请重新打开机械臂 GUI，逐个确认 1~7 号电机当前位置是否已接近 0 rad。")


def main() -> int:
    args = parse_args()
    confirm_or_exit(args)

    can: Optional[USBCANFD] = None
    try:
        can = open_can(args.device_index, args.channel_index)
        results = set_selected_motors_zero(
            can=can,
            motor_ids=args.motor_ids,
            timeout_ms=float(args.timeout_ms),
            retries=int(args.retries),
            delay_s=float(args.delay_s),
        )
        print_summary(can, results)

        # 返回码只说明是否为每个目标电机解析到有效回复，不绝对代表零位写入结果。
        return 0 if all(results.values()) else 2

    except KeyboardInterrupt:
        print("\n[WARN] 用户中断。")
        return 130
    except Exception as exc:
        print(f"[ERR] 设置 1~7 号电机零位失败: {exc}")
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
