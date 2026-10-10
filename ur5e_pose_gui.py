# -*- coding: utf-8 -*-
"""
UR5e 夹爪事件识别 / XYZ+Yaw 位姿补偿 / 自主保存 GUI
=========================
文件夹结构（默认）：
    ur5e_pose_gui.py
    teach_records/
        teach_xxx.npz
        shift_outputs/            # 自动生成

运行：python ur5e_pose_gui.py
依赖：pip install numpy scipy matplotlib
GUI：Python 自带 tkinter（部分 Linux 环境需额外安装 python3-tk）

仅进行离线运动学计算，不连接机器人、不下发运动指令。
使用 UR5e 名义 DH 模型的 tool0（法兰）坐标，未包含实际机器人标定及夹爪 TCP。
旋转轴为基坐标系 Z 轴，仅适用于物体绕桌面法线自转；自动旋转中心基于 tool0 XY 的近似估计，需人工校验。
未进行碰撞、关节加速度、实际安全区域和轨迹时序合法性验证；不可直接用于实机回放。
"""
from __future__ import annotations

import os
import tempfile
import queue
import re
import sys
import threading
import traceback
from pathlib import Path
from typing import Callable

import numpy as np
from scipy.optimize import least_squares
from scipy.ndimage import median_filter
from scipy.spatial.transform import Rotation

# 标准 DH，UR5e 名义参数（非出厂专属运动学校准值）
DH_A = np.array([0.0, -0.4250, -0.3922, 0.0, 0.0, 0.0])
DH_D = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])
JOINT_LIMIT = 2 * np.pi  # 仅用于名义数值合法性检查，非现场安全限位


def dh_matrix(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def fk_jacobian(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """base -> tool0 的变换矩阵及几何雅可比（工具0坐标，非夹爪 TCP）。"""
    T = np.eye(4)
    axes = np.empty((6, 3), dtype=float)
    origins = np.empty((6, 3), dtype=float)
    for j in range(6):
        axes[j] = T[:3, 2]
        origins[j] = T[:3, 3]
        T = T @ dh_matrix(float(q[j]), DH_A[j], DH_D[j], DH_ALPHA[j])
    J = np.empty((6, 6), dtype=float)
    for j in range(6):
        J[:3, j] = np.cross(axes[j], T[:3, 3] - origins[j])
        J[3:, j] = axes[j]
    return T, J


def pose_error(target: np.ndarray, actual: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pos = target[:3, 3] - actual[:3, 3]
    rot = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
    return pos, rot


def ik_near(target: np.ndarray, seed: np.ndarray,
            position_tolerance: float = 5e-5,
            rotation_tolerance: float = 5e-4) -> np.ndarray:
    """使用近邻初值的阻尼雅可比 IK；失败时启用 SciPy 优化求解。"""
    q = np.array(seed, dtype=float, copy=True)
    damping = 1e-3
    for _ in range(45):
        current, J = fk_jacobian(q)
        ep, er = pose_error(target, current)
        if np.linalg.norm(ep) < position_tolerance and np.linalg.norm(er) < rotation_tolerance:
            return q
        delta = np.linalg.solve(J.T @ J + damping**2 * np.eye(6), J.T @ np.r_[ep, er])
        mx = float(np.max(np.abs(delta)))
        if mx > 0.18:
            delta *= 0.18 / mx
        q += delta
        if not np.isfinite(q).all() or np.any(np.abs(q) > 2 * JOINT_LIMIT):
            break

    def residual(q_test):
        current, _ = fk_jacobian(q_test)
        ep, er = pose_error(target, current)
        return np.r_[ep, er]

    result = least_squares(
        residual,
        x0=np.clip(seed, -JOINT_LIMIT + 1e-8, JOINT_LIMIT - 1e-8),
        bounds=(-JOINT_LIMIT * np.ones(6), JOINT_LIMIT * np.ones(6)),
        method='trf', max_nfev=100,
        xtol=1e-11, ftol=1e-11, gtol=1e-11,
    )
    ep, er = pose_error(target, fk_jacobian(result.x)[0])
    if np.linalg.norm(ep) >= position_tolerance or np.linalg.norm(er) >= rotation_tolerance:
        raise RuntimeError(
            f'IK 未收敛：位置误差 {np.linalg.norm(ep)*1000:.3f} mm，'
            f'姿态误差 {np.linalg.norm(er):.5f} rad；该偏移可能不可达'
        )
    return result.x


def smoothstep5(s: float) -> float:
    """五次平滑曲线，s=0/1 时值为0/1，且端点的一、二阶导数为0。"""
    s = float(np.clip(s, 0, 1))
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def detect_closing_events(data: dict[str, np.ndarray],
                          source: str = 'auto') -> list[dict]:
    """识别有持续下降幅度的夹爪闭合命令，而非单帧抖动。

    gripper_*_open 的语义：数值越大越张开，数值下降代表闭合。
    事件时间使用与 UR 关节轨迹同一行号及控制频率，避免重复时间戳干扰。
    目标信号有效时优先使用 target，否则尝试实际反馈。
    这只能识别闭合动作，不能证明已经接触或成功抓住物体。
    """
    if source not in ('auto', 'target', 'actual'):
        raise ValueError('夹爪事件数据源必须为 auto、target 或 actual')
    hz = frequency_of(data)
    options = {'target': 'gripper_target_open', 'actual': 'gripper_open'}
    candidates = (('target', 'actual') if source == 'auto' else (source,))
    for label in candidates:
        key = options[label]
        if key not in data:
            continue
        raw = np.asarray(data[key], dtype=float)
        if raw.ndim != 1 or raw.size < 3 or not np.isfinite(raw).all():
            continue
        # 120 ms 中值滤波消除单个数据点的扰动，保留慢速闭合趋势
        window = max(3, int(round(hz * 0.12)) | 1)
        smooth = median_filter(raw, size=window, mode='nearest')
        if float(np.max(smooth)-np.min(smooth)) < 0.12:
            continue
        onset_drop = 0.012      # 相对前一开放峰值下降 1.2% 才可能视为开始闭合
        confirm_drop = 0.12     # 随后最多 4 s 内累计闭合 >= 12% 才确认
        release_rise = 0.15    # 回升 >= 15% 后，才重新允许检测下一次闭合
        look_ahead = max(1, int(round(hz * 4.0)))
        events = []
        armed = True
        peak_value = float(smooth[0])
        trough = peak_value
        last_event_index = -look_ahead
        i = 1
        while i < len(smooth):
            v = float(smooth[i])
            if armed:
                if v > peak_value:
                    peak_value = v
                if peak_value - v >= onset_drop:
                    j = min(len(smooth), i + look_ahead)
                    low = float(np.min(smooth[i:j]))
                    total_drop = peak_value-low
                    if total_drop >= confirm_drop and i-last_event_index >= max(1, int(hz * 0.5)):
                        events.append({
                            'index': i,
                            'time_s': float(i / hz),
                            'level_before': peak_value,
                            'level_min_next4s': low,
                            'confirmed_drop': total_drop,
                            'source': key,
                        })
                        armed = False
                        trough = v
                        last_event_index = i
            else:
                trough = min(trough, v)
                if v-trough >= release_rise:
                    armed = True
                    peak_value = v
            i += 1
        if events or source != 'auto':
            return events
    return []


def compensation_alpha(i: int, n: int, hz: float, mode: str,
                       seconds: float, complete_time: float | None = None) -> float:
    """full: 全程渐变；early: 前N秒；instant: 全程平移；gripper: 闭合前完成补偿。"""
    if mode == 'full':
        return smoothstep5(i / (n - 1))
    if mode == 'early':
        return smoothstep5((i / hz) / seconds) if seconds > 0 else 1.0
    if mode == 'gripper':
        if complete_time is None or complete_time <= 0:
            raise ValueError('夹爪事件补偿缺少有效的完成时间')
        return smoothstep5((i / hz) / complete_time)
    if mode == 'instant':
        return 1.0
    raise ValueError(f'未知补偿模式: {mode}')


def load_record(path: Path) -> dict[str, np.ndarray]:
    """严格检验已有 NPZ 的关键字段，不允许 pickle 加载。"""
    with np.load(path, allow_pickle=False) as npz:
        keys = set(npz.files)
        required = {'ur_q', 'timestamp', 'gripper_open'}
        if missing := (required - keys):
            raise ValueError(f'NPZ 缺少字段：{", ".join(sorted(missing))}')
        data = {key: npz[key].copy() for key in npz.files}

    q = np.asarray(data['ur_q'], dtype=float)
    n = q.shape[0] if q.ndim == 2 else 0
    if q.ndim != 2 or q.shape[1] != 6 or n < 2:
        raise ValueError('ur_q 必须为 (N, 6)，N≥2')
    for name in ('timestamp', 'gripper_open', 'gripper_target_open'):
        if name in data:
            a = np.asarray(data[name])
            if a.ndim != 1 or len(a) != n or not np.isfinite(a).all():
                raise ValueError(f'{name} 必须为长度 N 的有限数字数组')
    if 'ur_target_q' in data:
        q_target = np.asarray(data['ur_target_q'])
        if q_target.shape != (n, 6) or not np.isfinite(q_target).all():
            raise ValueError('ur_target_q 必须为 (N, 6)，数值有限')
    if not np.isfinite(q).all():
        raise ValueError('ur_q 包含 NaN 或 Inf')
    if np.max(np.abs(q)) > JOINT_LIMIT + 1e-7:
        raise ValueError('原轨迹关节角超过 ±2π 名义范围，请人工核查单位或数据')
    if 'rtde_control_frequency_hz' in data:
        hz_data = np.asarray(data['rtde_control_frequency_hz']).ravel()
        if hz_data.size != 1 or not np.isfinite(hz_data[0]) or hz_data[0] <= 0:
            raise ValueError('rtde_control_frequency_hz 无效')
    return data


def frequency_of(data: dict[str, np.ndarray]) -> float:
    if 'rtde_control_frequency_hz' in data:
        return float(data['rtde_control_frequency_hz'].ravel()[0])
    # 只有无法取得控制频率时才从非零时间戳差估计；须人工复核
    dt = np.diff(np.asarray(data['timestamp'], dtype=float))
    valid = dt[dt > 0]
    if valid.size == 0:
        raise ValueError('无法确定采样频率：缺少 rtde_control_frequency_hz 且时间戳无有效差值')
    return 1.0 / float(np.median(valid))



def rotation_z(yaw_rad: float) -> np.ndarray:
    """基坐标系绕 +Z 的主动旋转（逆时针为正，右手定则）。"""
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def compensated_pose(original: np.ndarray, xyz_m: np.ndarray, yaw_deg: float,
                     center_xy_m: np.ndarray, alpha: float) -> np.ndarray:
    """绕物体旧中心旋转，并叠加基坐标系平移。所有量由同一 alpha 渐变。

    p' = c + alpha*xyz + Rz(alpha*yaw)*(p-c)
    R' = Rz(alpha*yaw)*R
    c 是原始物体中心在机器人基坐标系的 XY（Z 不影响绕Z的旋转）。
    """
    c = np.array([center_xy_m[0], center_xy_m[1], 0.0], dtype=float)
    rot = rotation_z(np.deg2rad(float(yaw_deg)) * alpha)
    target = original.copy()
    target[:3, 3] = c + alpha * xyz_m + rot @ (original[:3, 3] - c)
    target[:3, :3] = rot @ original[:3, :3]
    return target


def pivot_from_grasp(record: dict[str, np.ndarray], selected_index: int | None = None
                     ) -> tuple[np.ndarray, int, str]:
    """仅作为物体中心近似：取闭合开始帧的 tool0 XY；准确旋转中心建议人工输入。"""
    q = np.asarray(record['ur_q'], dtype=float)
    events = detect_closing_events(record)
    if selected_index is not None:
        match = next((e for e in events if e['index'] == selected_index), None)
        if match is None:
            raise ValueError('自动中心关联的夹爪闭合事件不存在')
        index = int(match['index'])
        note = f'闭合事件 t={match["time_s"]:.2f}s 的 tool0 XY（近似）'
    elif events:
        index = int(events[0]['index'])
        note = f'首次闭合事件 t={events[0]["time_s"]:.2f}s 的 tool0 XY（近似）'
    else:
        index = len(q)-1
        note = '未检测到闭合事件，使用终点 tool0 XY（粗略近似）'
    T, _ = fk_jacobian(q[index])
    return T[:2, 3].copy(), index, note


def compute_shift(data: dict[str, np.ndarray], offset_m: np.ndarray,
                  ramp_seconds: float,
                  progress: Callable[[int, int], None] | None = None,
                  mode: str = 'full',
                  complete_time: float | None = None,
                  yaw_deg: float = 0.0,
                  pivot_xy_m: np.ndarray | None = None):
    if mode not in ('full', 'early', 'instant', 'gripper'):
        raise ValueError(f'无效的补偿模式: {mode}')
    if not np.isfinite(yaw_deg) or abs(yaw_deg) > 45:
        raise ValueError('绕Z旋转角度必须在 -45°～+45°')
    if pivot_xy_m is None:
        pivot_xy_m = np.zeros(2, dtype=float)
    pivot_xy_m = np.asarray(pivot_xy_m, dtype=float)
    if pivot_xy_m.shape != (2,) or not np.isfinite(pivot_xy_m).all():
        raise ValueError('原始物体旋转中心 X/Y 必须是有效数值')
    q0 = np.asarray(data['ur_q'], dtype=float)
    n = len(q0)
    hz = frequency_of(data)
    ts = np.arange(n, dtype=float) / hz
    q1 = np.empty_like(q0)
    max_pos_error = 0.0
    max_rot_error = 0.0
    last_original = last_solution = last_alpha = None

    for i, q in enumerate(q0):
        alpha = compensation_alpha(i, n, hz, mode, ramp_seconds, complete_time)
        if (last_original is not None and np.array_equal(q, last_original)
                and alpha == last_alpha):
            solution = last_solution
        else:
            original, _ = fk_jacobian(q)
            target = compensated_pose(original, offset_m, yaw_deg, pivot_xy_m, alpha)
            if alpha == 0.0 or (np.max(np.abs(offset_m)) < 1e-14 and abs(yaw_deg) < 1e-14):
                solution = q.copy()
            else:
                seed = q if last_solution is None else last_solution + (q - last_original)
                try:
                    solution = ik_near(target, seed)
                except Exception as exc:
                    # 首次未收敛不等同于几何不可达，改用该点原轨迹关节角再求一次。
                    try:
                        solution = ik_near(target, q)
                    except Exception as retry_exc:
                        raise RuntimeError(
                            f'第 {i+1}/{n} 个轨迹点（t={ts[i]:.3f} s）IK未满足数值精度；'
                            f'首次结果：{exc}；原始关节初值重试：{retry_exc}。'
                            '不能据此断言物理不可达；请勿直接回放。') from retry_exc
            if np.any(np.abs(solution) > JOINT_LIMIT + 1e-7):
                raise RuntimeError(f'第 {i+1} 点的新关节角超出 ±2π 名义范围，未保存结果')
            achieved, _ = fk_jacobian(solution)
            ep, er = pose_error(target, achieved)
            max_pos_error = max(max_pos_error, float(np.linalg.norm(ep)))
            max_rot_error = max(max_rot_error, float(np.linalg.norm(er)))
        q1[i] = solution
        last_original, last_solution, last_alpha = q, solution, alpha
        if progress and (i % 250 == 0 or i == n - 1):
            progress(i + 1, n)

    max_joint_jump = float(np.max(np.abs(np.diff(q1, axis=0))))
    max_joint_speed = float(np.max(np.abs(np.diff(q1, axis=0) * hz)))
    info = {
        'N': n, 'hz': hz,
        'max_pos_error_mm': max_pos_error * 1000,
        'max_rot_error_rad': max_rot_error,
        'max_joint_jump_rad': max_joint_jump,
        'max_joint_speed_rad_s': max_joint_speed,
        'mode': mode, 'yaw_deg': float(yaw_deg), 'rotation_center_xy_m': pivot_xy_m.copy(),
        'start_offset_mm': 1000.0 * float(np.linalg.norm(fk_jacobian(q1[0])[0][:3, 3] - fk_jacobian(q0[0])[0][:3, 3])),
        'end_offset_mm': 1000.0 * float(np.linalg.norm(fk_jacobian(q1[-1])[0][:3, 3] - fk_jacobian(q0[-1])[0][:3, 3])),
    }
    return q1, ts, info


def safe_part(v: float) -> str:
    # 命名使用 cm，精度到 0.001 cm，不与不同偏移量混淆
    if abs(v) < 0.0005:
        return '0'
    return ('p' if v > 0 else 'm') + f'{abs(v):.3f}'.rstrip('0').rstrip('.').replace('.', 'd')


def make_output_base(source: Path, xyz_cm: np.ndarray, folder: Path, yaw_deg: float = 0.0) -> Path:
    tag = '_'.join(axis + safe_part(float(v)) for axis, v in zip('xyz', xyz_cm))
    tag += '_rz' + safe_part(float(yaw_deg))
    stem = re.sub(r'[^\w\-]+', '_', source.stem)
    base = folder / f'{stem}_shift_{tag}'
    candidate = base
    version = 2
    while any(p.exists() for p in (
        candidate.with_suffix('.npz'),
        candidate.with_name(candidate.name + '_original_3d.png'),
        candidate.with_name(candidate.name + '_shifted_3d.png'),
    )):
        candidate = folder / f'{base.name}_v{version}'
        version += 1
    return candidate


def sampled_xyz(q: np.ndarray, indexes: np.ndarray) -> np.ndarray:
    xyz = np.empty((len(indexes), 3), dtype=float)
    for j, i in enumerate(indexes):
        xyz[j] = fk_jacobian(q[i])[0][:3, 3]
    return xyz


def draw_3d(ax, old: np.ndarray, new: np.ndarray, bounds: np.ndarray,
            compensated: bool, orientation_markers: dict | None = None):
    if compensated:
        ax.plot(*old.T, color='#97a3ae', linestyle='--', alpha=0.7,
                linewidth=1.5, label='Original reference')
        points, color, label = new, '#dc7d23', 'Compensated'
        ax.scatter(*old[0], marker='x', s=55, color='#374151', label='Original start')
        ax.scatter(*old[-1], marker='x', s=55, color='#374151', label='Original end')
        ax.plot(np.array([old[-1, 0], new[-1, 0]]),
                np.array([old[-1, 1], new[-1, 1]]),
                np.array([old[-1, 2], new[-1, 2]]),
                color='#9155b5', linewidth=2.0, label='End displacement')
    else:
        points, color, label = old, '#2678b5', 'Original'
    ax.plot(*points.T, color=color, linewidth=1.8, label=label)
    ax.scatter(*points[0], s=38, marker='o', color='#209e78', label='Start')
    ax.scatter(*points[-1], s=35, marker='^', color='#c33b56', label='End')
    # 同步显示法兰 tool0 的 X 轴方向：可看到姿态随物体 Rz 旋转。
    # 箭头代表法兰方向，不代表夹爪指尖方向。
    if orientation_markers:
        names = ('old', 'new') if compensated else ('old',)
        for name in names:
            p, direction = orientation_markers[name]
            arrow_color = '#374151' if name == 'old' else '#dc7d23'
            ax.quiver(*p, *direction, length=0.06, normalize=True,
                      color=arrow_color, linewidth=2.2,
                      label=('Original tool0 X' if name == 'old' else 'New tool0 X'))
    ax.set_xlim(*bounds[0]); ax.set_ylim(*bounds[1]); ax.set_zlim(*bounds[2])
    ax.set_box_aspect(bounds[:, 1] - bounds[:, 0])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.view_init(elev=25, azim=-56)
    ax.set_title('Compensated + original reference' if compensated else 'Original trajectory',
                 fontsize=10)
    ax.legend(loc='upper right', fontsize=7)


def common_bounds(old: np.ndarray, new: np.ndarray) -> np.ndarray:
    combined = np.vstack((old, new))
    lo, hi = np.min(combined, axis=0), np.max(combined, axis=0)
    mid = (lo + hi) / 2
    span = np.maximum((hi - lo) * 1.15, 0.08)
    return np.stack((mid - span / 2, mid + span / 2), axis=1)


def write_image(path: Path, old: np.ndarray, new: np.ndarray,
                bounds: np.ndarray, shifted: bool,
                orientation_markers: dict | None = None):
    # 不依赖 Tk 显示环境，可在后台线程直接保存 PNG
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(7.8, 6.1), dpi=155)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, projection='3d')
    draw_3d(ax, old, new, bounds, compensated=shifted,
            orientation_markers=orientation_markers)
    fig.subplots_adjust(left=0.02, right=0.94, bottom=0.07, top=0.92)
    fig.savefig(path, dpi=155)
    fig.clear()


def generate_preview(source: Path, xyz_cm: np.ndarray, ramp: float,
                     progress: Callable[[str, int, int], None] | None = None,
                     mode: str = 'full',
                     close_event_index: int | None = None,
                     advance_seconds: float = 1.0,
                     yaw_deg: float = 0.0,
                     pivot_mode: str = 'auto',
                     pivot_xy_m: np.ndarray | None = None):
    """只计算，不写入磁盘：返回 NPZ 内容和两张图片共用的三维预览数据。"""
    xyz_cm = np.asarray(xyz_cm, dtype=float)
    if xyz_cm.shape != (3,) or not np.isfinite(xyz_cm).all() or np.any(np.abs(xyz_cm) > 10):
        raise ValueError('X/Y/Z 位移分别必须在 -10 到 +10 cm 范围内')
    if mode not in ('full', 'early', 'instant', 'gripper'):
        raise ValueError('补偿模式无效')
    if not np.isfinite(yaw_deg) or abs(yaw_deg) > 45:
        raise ValueError('绕 Z 轴的旋转角度范围为 -45°～+45°')
    if pivot_mode not in ('auto', 'manual'):
        raise ValueError('旋转中心模式必须为 auto 或 manual')
    if mode == 'early' and (not np.isfinite(ramp) or not (0 <= ramp <= 10)):
        raise ValueError('前 N 秒渐变时间必须在 0 到 10 秒范围内')
    source = Path(source).resolve()
    if not source.is_file() or source.suffix.lower() != '.npz':
        raise FileNotFoundError(f'原始 NPZ 不存在：{source}')
    record = load_record(source)
    q0 = np.asarray(record['ur_q'], dtype=float)
    hz = frequency_of(record)
    events = detect_closing_events(record)
    chosen = None
    complete_time = None
    if mode == 'gripper':
        if not np.isfinite(advance_seconds) or not 0 <= advance_seconds <= 10:
            raise ValueError('闭合前提前完成时间必须在 0~10 秒之间')
        if not events:
            raise ValueError('该文件没有识别到可靠的夹爪闭合事件；请检查夹爪数据或使用手动前N秒模式')
        if close_event_index is None:
            if len(events) > 1:
                raise ValueError('检测到多次夹爪闭合，请在 GUI 中选择哪一次用于抓取')
            chosen = events[0]
        else:
            chosen = next((ev for ev in events if ev['index'] == close_event_index), None)
            if chosen is None:
                raise ValueError('所选择的夹爪闭合事件不属于当前轨迹')
        complete_time = float(chosen['time_s'] - advance_seconds)
        if complete_time < 0.25:
            raise ValueError('完成时刻过早（不足0.25秒），请减小提前时间或选择其他闭合事件')

    if pivot_mode == 'auto':
        # 自动中心为近似估计；基于 tool0 的 XY，而不是视觉/标定后的物体真实中心。
        pivot, pivot_ref_index, pivot_note = pivot_from_grasp(record, close_event_index)
    else:
        pivot = np.asarray(pivot_xy_m, dtype=float)
        if pivot.shape != (2,) or not np.isfinite(pivot).all():
            raise ValueError('请填写原始物体中心 X、Y（机器人基坐标系，单位 m）')
        pivot_ref_index = -1
        pivot_note = '人工输入的原始物体中心 XY（基坐标系）'

    def on_progress(i, n):
        if progress:
            progress('IK 计算中', i, n)

    q1, uniform_ts, stats = compute_shift(
        record, xyz_cm / 100, ramp, on_progress, mode=mode,
        complete_time=complete_time, yaw_deg=yaw_deg, pivot_xy_m=pivot)
    if progress:
        progress('生成预览数据', 0, 1)
    ids = np.linspace(0, len(q0) - 1, num=min(len(q0), 2400), dtype=int)
    old_xyz = sampled_xyz(q0, ids)
    new_xyz = sampled_xyz(q1, ids)
    bounds = common_bounds(old_xyz, new_xyz)
    # 标记闭合事件或最后一帧的 tool0 姿态，绘制末端方向轴。
    marker_i = chosen['index'] if chosen else (events[0]['index'] if events else len(q0)-1)
    marker_old, _ = fk_jacobian(q0[marker_i])
    marker_new, _ = fk_jacobian(q1[marker_i])
    orientation_markers = {
        'old': (marker_old[:3, 3].copy(), marker_old[:3, 0].copy()),
        'new': (marker_new[:3, 3].copy(), marker_new[:3, 0].copy()),
    }

    # 同步写入事件时间与补偿比例，便于后续核对每一个关节轨迹点。
    alpha_curve = np.array([
        compensation_alpha(i, len(q0), hz, mode, ramp, complete_time)
        for i in range(len(q0))], dtype=float)
    # 旧版回放常用字段保持一致；旧主端数据单独存到 source_ 字段。
    output = {
        'timestamp': uniform_ts,
        'ur_q': q1,
        'ur_target_q': q1.copy(),
        'gripper_open': record['gripper_open'].copy(),
        'gripper_target_open': record.get('gripper_target_open', record['gripper_open']).copy(),
        'rtde_control_frequency_hz': np.array([stats['hz']], dtype=float),
        'rtde_receive_frequency_hz': record.get('rtde_receive_frequency_hz', np.array([np.nan])).copy(),
        'original_timestamp': record['timestamp'].copy(),
        'source_ur_q': q0.copy(),
        'source_ur_target_q': record.get('ur_target_q', q0).copy(),
        'source_master_q': record.get('master_q', np.empty((0, 6))).copy(),
        'source_tool7_pos': record.get('tool7_pos', np.empty((0,))).copy(),
        'offset_xyz_m': xyz_cm / 100,
        'yaw_offset_deg': np.array([yaw_deg], dtype=float),
        'yaw_offset_rad': np.array([np.deg2rad(yaw_deg)], dtype=float),
        'rotation_center_xy_m': pivot.copy(),
        'rotation_center_mode': np.array([pivot_mode]),
        'rotation_center_reference_index': np.array([pivot_ref_index], dtype=int),
        'yaw_compensation_deg': alpha_curve * yaw_deg,
        'ramp_seconds': np.array([
            complete_time if mode == 'gripper' else
            (ramp if mode == 'early' else (float(uniform_ts[-1]) if mode == 'full' else 0))
        ], dtype=float),
        'compensation_alpha': alpha_curve,
        'gripper_close_index': np.array([chosen['index'] if chosen else -1], dtype=int),
        'gripper_close_time_s': np.array([chosen['time_s'] if chosen else np.nan], dtype=float),
        'gripper_close_signal': np.array([chosen['source'] if chosen else 'none']),
        'gripper_advance_seconds': np.array([advance_seconds if chosen else np.nan], dtype=float),
        'compensation_complete_time_s': np.array([complete_time if chosen else np.nan], dtype=float),
        'compensation_mode': np.array([mode]),
        'nominal_dh_model': np.array(['UR5e_tool0']),
        'max_fk_position_error_m': np.array([stats['max_pos_error_mm'] / 1000]),
        'max_fk_orientation_error_rad': np.array([stats['max_rot_error_rad']]),
    }
    if progress:
        progress('预览就绪（尚未保存）', 1, 1)
    stats['gripper_close_time_s'] = chosen['time_s'] if chosen else None
    stats['compensation_complete_time_s'] = complete_time
    stats['pivot_note'] = pivot_note
    return {'yaw_deg': float(yaw_deg), 'pivot_xy_m': pivot.copy(),
            'pivot_mode': pivot_mode, 'pivot_note': pivot_note,
            'orientation_markers': orientation_markers,
            'source': source, 'xyz_cm': xyz_cm.copy(), 'ramp': ramp, 'mode': mode,
            'gripper_event': chosen, 'detected_events': events,
            'gripper_time': uniform_ts,
            'gripper_target_curve': record.get('gripper_target_open', record['gripper_open']).copy(),
            'gripper_actual_curve': record['gripper_open'].copy(),
            'alpha_curve': alpha_curve,
            'npz_data': output, 'stats': stats,
            'old_xyz': old_xyz, 'new_xyz': new_xyz, 'bounds': bounds,
            'base': None, 'saved_paths': {}}


def save_selected_outputs(result: dict, output_dir: Path,
                          save_npz: bool, save_png: bool,
                          progress: Callable[[str, int, int], None] | None = None) -> dict:
    """按两个独立选项保存：NPZ 一份，PNG 一组两张；默认不覆盖旧文件。"""
    if not (save_npz or save_png):
        return {'saved': [], 'skipped': [], 'paths': dict(result['saved_paths'])}
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    # 仅在用户第一次按下保存时固定文件名；之后追加保存另一类文件，沿用同一文件名前缀。
    if result['base'] is None:
        result['base'] = make_output_base(result['source'], result['xyz_cm'], output_dir, result.get('yaw_deg', 0.0))
    base = result['base']
    destinations = {
        'npz': base.with_suffix('.npz'),
        'original_png': base.with_name(base.name + '_original_3d.png'),
        'compensated_png': base.with_name(base.name + '_shifted_3d.png'),
    }
    saved, skipped = [], []

    def atomic_save(key: str, writer):
        dest = destinations[key]
        if key in result['saved_paths'] and dest.is_file():
            skipped.append(dest)
            return
        if dest.exists():
            raise FileExistsError(f'目标文件已存在，程序不会覆盖：{dest}')
        fd, tempname = tempfile.mkstemp(
            prefix='.ur5e_pending_', suffix=dest.suffix, dir=output_dir)
        os.close(fd)
        temp = Path(tempname)
        try:
            writer(temp)
            if dest.exists():
                raise FileExistsError(f'目标文件已存在，程序不会覆盖：{dest}')
            os.replace(temp, dest)
            result['saved_paths'][key] = dest
            saved.append(dest)
        finally:
            temp.unlink(missing_ok=True)

    if save_npz:
        if progress:
            progress('保存 NPZ', 0, 1)
        atomic_save('npz', lambda path: np.savez_compressed(path, **result['npz_data']))
        if progress:
            progress('保存 NPZ', 1, 1)
    if save_png:
        if progress:
            progress('保存两张 PNG', 0, 2)
        # 两张轨迹图为一组：先写第一张，再写第二张；失败时回滚本次新建的 PNG。
        created_png_this_time = []
        try:
            count = len(saved)
            atomic_save('original_png', lambda path: write_image(
                path, result['old_xyz'], result['new_xyz'], result['bounds'], shifted=False,
                orientation_markers=result.get('orientation_markers')))
            created_png_this_time.extend(saved[count:])
            if progress:
                progress('保存两张 PNG', 1, 2)
            count = len(saved)
            atomic_save('compensated_png', lambda path: write_image(
                path, result['old_xyz'], result['new_xyz'], result['bounds'], shifted=True,
                orientation_markers=result.get('orientation_markers')))
            created_png_this_time.extend(saved[count:])
            if progress:
                progress('保存两张 PNG', 2, 2)
        except Exception:
            for path in created_png_this_time:
                path.unlink(missing_ok=True)
                result['saved_paths'].pop(
                    'original_png' if path == destinations['original_png'] else 'compensated_png', None)
                if path in saved:
                    saved.remove(path)
            raise
    return {'saved': saved, 'skipped': skipped, 'paths': dict(result['saved_paths'])}


class ShiftGui:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk
        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.script_dir = Path(__file__).resolve().parent
        self.record_dir = self.script_dir / 'teach_records'
        self.output_dir = self.record_dir / 'shift_outputs'
        self.events = queue.Queue()
        self.busy = False
        self.last_result = None
        self.save_npz_var = tk.BooleanVar(value=True)
        self.save_png_var = tk.BooleanVar(value=True)
        self.records = {}
        self.root.title('UR5e XYZ + Rz 示教轨迹修正')
        self.root.geometry('1250x940')
        self.root.minsize(1060, 760)
        self.file_name = tk.StringVar()
        self.dx = tk.StringVar(value='5.0')
        self.dy = tk.StringVar(value='0.0')
        self.dz = tk.StringVar(value='0.0')
        self.yaw = tk.StringVar(value='0.0')
        self.pivot_choice = tk.StringVar(value='auto')
        self.pivot_x = tk.StringVar(value='0.0')
        self.pivot_y = tk.StringVar(value='0.0')
        self.ramp = tk.StringVar(value='2.0')
        self.mode_label = tk.StringVar(value='夹爪闭合自动补偿：抓取前完成偏移')
        self.advance = tk.StringVar(value='1.0')
        self.event_label = tk.StringVar()
        self.close_events = []
        self.event_options = {}
        self.summary = tk.StringVar(value='请选择示教轨迹文件。')
        self.status = tk.StringVar(value='就绪（不连接 UR5e）')
        self.result_label = tk.StringVar(value='生成轨迹并查看预览后，可自主决定保存哪些文件；输出目录为 teach_records/shift_outputs/')

        style = ttk.Style()
        if 'clam' in style.theme_names(): style.theme_use('clam')
        self.build_widgets()
        self.refresh_files()
        self.root.after(100, self.poll_events)

    def build_widgets(self):
        tk, ttk = self.tk, self.ttk
        main = ttk.Frame(self.root, padding=14)
        main.pack(fill='both', expand=True)
        ttk.Label(main, text='UR5e 示教轨迹离线补偿', font=('Microsoft YaHei UI', 17, 'bold')).pack(anchor='w')
        ttk.Label(main, text='从 teach_records 选择 NPZ，设置基坐标系 XYZ + 绕Z旋转、原物体中心；先预览，再决定是否保存（不控制真实机器人）。',
                  foreground='#4e6474').pack(anchor='w', pady=(2, 12))

        choice = ttk.LabelFrame(main, text='1 选择原始轨迹', padding=10)
        choice.pack(fill='x')
        line = ttk.Frame(choice); line.pack(fill='x')
        ttk.Label(line, text='轨迹文件').pack(side='left', padx=(0, 8))
        self.combo = ttk.Combobox(line, textvariable=self.file_name, state='readonly', width=48)
        self.combo.pack(side='left', fill='x', expand=True)
        self.combo.bind('<<ComboboxSelected>>', lambda e: self.selected_changed())
        self.refresh_button = ttk.Button(line, text='刷新列表', command=self.refresh_files)
        self.refresh_button.pack(side='left', padx=(10, 0))
        ttk.Label(choice, text=f'默认目录：{self.record_dir}', foreground='#677782').pack(anchor='w', pady=(7, 2))
        ttk.Label(choice, textvariable=self.summary, foreground='#314e69').pack(anchor='w')

        setting = ttk.LabelFrame(main, text='2 设置目标物体位姿变化（基坐标系）', padding=10)
        setting.pack(fill='x', pady=(10, 0))
        row = ttk.Frame(setting); row.pack(fill='x')
        self.spins = []
        for label, value in [('X', self.dx), ('Y', self.dy), ('Z', self.dz)]:
            group = ttk.Frame(row); group.pack(side='left', padx=(0, 22))
            ttk.Label(group, text=f'{label}：').pack(side='left')
            spin = ttk.Spinbox(group, from_=-10.0, to=10.0, increment=0.5,
                               textvariable=value, width=9, justify='center')
            spin.pack(side='left')
            ttk.Label(group, text='cm').pack(side='left', padx=(4, 0))
            self.spins.append(spin)
        ttk.Label(row, text='范围：-10 ～ +10 cm（可手动输入小数）', foreground='#677782').pack(side='left')
        # 新增姿态与旋转中心：物体绕自身旧中心作平面 yaw 旋转。
        rotation_row = ttk.Frame(setting)
        rotation_row.pack(fill='x', pady=(9, 0))
        ttk.Label(rotation_row, text='绕基坐标系 Z 轴旋转 Rz：').pack(side='left')
        self.yaw_spin = ttk.Spinbox(rotation_row, from_=-45.0, to=45.0,
                                    increment=1.0, textvariable=self.yaw,
                                    width=8, justify='center')
        self.yaw_spin.pack(side='left')
        ttk.Label(rotation_row, text='°（-45 ～ +45，逆时针为正，从Z正方向俯视）',
                  foreground='#677782').pack(side='left', padx=(7, 0))

        pivot_row = ttk.Frame(setting)
        pivot_row.pack(fill='x', pady=(8, 0))
        ttk.Label(pivot_row, text='物体原始旋转中心：').pack(side='left')
        ttk.Radiobutton(pivot_row, text='自动估计（闭合时 tool0 XY，近似）',
                        variable=self.pivot_choice, value='auto',
                        command=self.on_pivot_changed).pack(side='left', padx=(8, 12))
        ttk.Radiobutton(pivot_row, text='手动输入物体中心 XY',
                        variable=self.pivot_choice, value='manual',
                        command=self.on_pivot_changed).pack(side='left', padx=(0, 12))
        self.pivot_entry_frame = ttk.Frame(pivot_row)
        self.pivot_entry_frame.pack(side='left')
        ttk.Label(self.pivot_entry_frame, text='X:').pack(side='left')
        self.pivot_x_spin = ttk.Spinbox(self.pivot_entry_frame, from_=-2.0, to=2.0,
                                         increment=0.01, textvariable=self.pivot_x,
                                         width=8, justify='center')
        self.pivot_x_spin.pack(side='left')
        ttk.Label(self.pivot_entry_frame, text='Y:').pack(side='left', padx=(7, 0))
        self.pivot_y_spin = ttk.Spinbox(self.pivot_entry_frame, from_=-2.0, to=2.0,
                                         increment=0.01, textvariable=self.pivot_y,
                                         width=8, justify='center')
        self.pivot_y_spin.pack(side='left')
        ttk.Label(self.pivot_entry_frame, text='m').pack(side='left', padx=(4, 0))
        ttk.Label(setting, text='旋转中心指原始示教时的物体中心：自动值仅为法兰XY近似，真实抓取请优先用标定/测量的物体中心。',
                  foreground='#9a571c').pack(anchor='w', pady=(3, 0))
        self.on_pivot_changed()
        under = ttk.Frame(setting); under.pack(fill='x', pady=(10, 0))
        ttk.Label(under, text='补偿方式：').pack(side='left')
        self.mode_labels = {
            '全程渐变：起点不变、终点完整偏移': 'full',
            '前 N 秒渐变：起点不变、N秒后完整偏移': 'early',
            '立即平移：所有点偏移（包括起点）': 'instant',
            '夹爪闭合自动补偿：抓取前完成偏移': 'gripper',
        }
        self.mode_combo = ttk.Combobox(
            under, textvariable=self.mode_label, state='readonly', width=38,
            values=list(self.mode_labels))
        self.mode_combo.pack(side='left', padx=(0, 12))
        self.mode_combo.bind('<<ComboboxSelected>>', lambda e: self.on_mode_changed())
        self.ramp_label = ttk.Label(under, text='提前完成时间：')
        self.ramp_label.pack(side='left')
        self.ramp_spin = ttk.Spinbox(under, from_=0.1, to=10, increment=0.5,
                                     textvariable=self.ramp, width=7, justify='center')
        self.ramp_spin.pack(side='left')
        ttk.Label(under, text='秒（仅“前 N 秒渐变”有效）',
                  foreground='#677782').pack(side='left', padx=(7, 0))
        self.grip_row = ttk.Frame(setting)
        self.grip_row.pack(fill='x', pady=(8, 0))
        ttk.Label(self.grip_row, text='闭合事件：').pack(side='left')
        self.event_combo = ttk.Combobox(self.grip_row, textvariable=self.event_label,
                                        state='readonly', width=42)
        self.event_combo.pack(side='left', padx=(4, 15))
        self.event_combo.bind('<<ComboboxSelected>>', lambda e: self.update_event_note())
        ttk.Label(self.grip_row, text='提前完成：').pack(side='left')
        self.advance_spin = ttk.Spinbox(self.grip_row, from_=0, to=10, increment=0.5,
                                        textvariable=self.advance, width=7, justify='center')
        self.advance_spin.pack(side='left')
        ttk.Label(self.grip_row, text='秒').pack(side='left', padx=(4, 10))
        self.event_note = tk.StringVar(value='将提前于夹爪开始闭合完成完整位置补偿')
        ttk.Label(self.grip_row, textvariable=self.event_note,
                  foreground='#416277').pack(side='left')
        self.advance.trace_add('write', lambda *_: self.update_event_note())
        self.mode_hint = ttk.Label(setting,
                  text='提示：夹爪闭合是命令事件，非接触检测；抓取后仍保持 XYZ+Rz 补偿，固定放置点也会改变。',
                  foreground='#7a5c33')
        self.mode_hint.pack(anchor='w', pady=(7, 0))
        self.on_mode_changed()

        actions = ttk.Frame(main); actions.pack(fill='x', pady=(11, 8))
        self.run_button = ttk.Button(actions, text='生成轨迹并预览（暂不保存）', command=self.start)
        self.run_button.pack(side='left')
        self.folder_button = ttk.Button(actions, text='打开输出文件夹', command=self.open_folder)
        self.folder_button.pack(side='left', padx=(10, 0))
        ttk.Label(actions, textvariable=self.status, foreground='#175b85').pack(side='right')
        save_frame = ttk.LabelFrame(main, text='3 结果保存（计算完成后可自主选择）', padding=9)
        save_frame.pack(fill='x', pady=(3, 9))
        save_row = ttk.Frame(save_frame)
        save_row.pack(fill='x')
        self.save_npz_check = ttk.Checkbutton(save_row, text='保存修正后的 NPZ',
                                             variable=self.save_npz_var, state='disabled')
        self.save_npz_check.pack(side='left', padx=(0, 22))
        self.save_png_check = ttk.Checkbutton(save_row, text='保存两张三维轨迹图（PNG）',
                                             variable=self.save_png_var, state='disabled')
        self.save_png_check.pack(side='left', padx=(0, 22))
        self.save_button = ttk.Button(save_row, text='保存所选结果',
                                      command=self.start_save, state='disabled')
        self.save_button.pack(side='left')
        ttk.Label(save_frame, text=f'保存到：{self.output_dir}（生成预览不会自动写文件）',
                  foreground='#677782').pack(anchor='w', pady=(7, 0))

        self.progress_bar = ttk.Progressbar(main, mode='determinate', maximum=100)
        self.progress_bar.pack(fill='x', pady=(0, 6))
        ttk.Label(main, textvariable=self.result_label, foreground='#3f6760', wraplength=1100).pack(anchor='w')

        notebook = ttk.Notebook(main)
        notebook.pack(fill='both', expand=True, pady=(10, 0))
        self.preview_tab = ttk.Frame(notebook)
        self.log_tab = ttk.Frame(notebook)
        self.gripper_tab = ttk.Frame(notebook)
        notebook.add(self.preview_tab, text='三维轨迹预览（两张图）')
        notebook.add(self.gripper_tab, text='夹爪事件与补偿比例')
        notebook.add(self.log_tab, text='运行记录')
        self.gripper_canvas = None
        self.preview_note = ttk.Label(self.preview_tab, text='计算完成后，这里会显示原始轨迹和补偿后的轨迹。',
                                      foreground='#6a7881')
        self.preview_note.pack(pady=30)
        self.canvas = None
        self.log_widget = tk.Text(self.log_tab, height=12, wrap='word', state='disabled')
        self.log_widget.pack(side='left', fill='both', expand=True)
        bar = ttk.Scrollbar(self.log_tab, command=self.log_widget.yview)
        bar.pack(side='right', fill='y')
        self.log_widget.config(yscrollcommand=bar.set)
        ttk.Label(main, text='安全提示：仅供离线实验；使用名义 DH 法兰 tool0；自动旋转中心为近似值，未校准实机运动学/TCP/碰撞/加速度。',
                  foreground='#9a571c', wraplength=1100).pack(anchor='w', pady=(7, 0))

    def log(self, s):
        self.log_widget.config(state='normal')
        self.log_widget.insert('end', s + '\n')
        self.log_widget.see('end')
        self.log_widget.config(state='disabled')

    def refresh_files(self):
        if self.busy: return
        if not self.record_dir.is_dir():
            self.combo['values'] = []
            self.records = {}
            self.file_name.set('')
            self.summary.set('未找到 teach_records 文件夹。请将它放在 GUI 脚本同级目录。')
            return
        paths = sorted(self.record_dir.glob('*.npz'), key=lambda p: p.stat().st_mtime, reverse=True)
        self.records = {p.name: p for p in paths}
        names = list(self.records)
        prior = self.file_name.get()
        self.combo['values'] = names
        self.file_name.set(prior if prior in self.records else (names[0] if names else ''))
        self.selected_changed()
        self.log(f'扫描完成：在 teach_records 找到 {len(names)} 个 NPZ 文件。')

    def selected_changed(self):
        path = self.records.get(self.file_name.get())
        if not path:
            self.close_events = []
            self.event_options = {}
            self.event_combo['values'] = []
            self.event_label.set('')
            self.summary.set('文件夹中尚无 NPZ 文件。')
            return
        try:
            with np.load(path, allow_pickle=False) as d:
                if 'ur_q' not in d:
                    raise ValueError('缺少 ur_q')
                q = d['ur_q']
                hz = float(d['rtde_control_frequency_hz'].ravel()[0]) if 'rtde_control_frequency_hz' in d else None
                msg = f'轨迹点：{len(q)}  |  UR关节维度：{q.shape}  |  频率：{hz:g} Hz' if hz else f'轨迹点：{len(q)}  |  UR关节维度：{q.shape}'
                ev_data = {'gripper_open': d['gripper_open'].copy()} if 'gripper_open' in d else {}
                if 'gripper_target_open' in d:
                    ev_data['gripper_target_open'] = d['gripper_target_open'].copy()
                if 'rtde_control_frequency_hz' in d:
                    ev_data['rtde_control_frequency_hz'] = d['rtde_control_frequency_hz'].copy()
                elif 'timestamp' in d:
                    ev_data['timestamp'] = d['timestamp'].copy()
                self.close_events = detect_closing_events(ev_data) if ev_data else []
                msg += f'  |  闭合事件：{len(self.close_events)} 次'
                self.summary.set(msg)
                self.event_options = {
                    f"闭合 {i + 1}：{ev['time_s']:.2f} s（{ev['source']}）": ev['index']
                    for i, ev in enumerate(self.close_events)
                }
                names = list(self.event_options)
                self.event_combo['values'] = names
                self.event_label.set(names[0] if names else '')
                self.update_event_note()
        except Exception as e:
            self.close_events = []
            self.event_options = {}
            self.event_combo['values'] = []
            self.event_label.set('')
            self.summary.set(f'文件可能无法读取：{e}')

    def on_mode_changed(self):
        mode = self.mode_labels.get(self.mode_label.get(), 'full')
        self.ramp_spin.config(state='normal' if mode == 'early' else 'disabled')
        if mode == 'gripper':
            if not self.grip_row.winfo_manager():
                self.grip_row.pack(fill='x', pady=(8, 0), before=self.mode_hint)
        else:
            self.grip_row.pack_forget()

    def on_pivot_changed(self):
        manual = self.pivot_choice.get() == 'manual'
        for spin in (self.pivot_x_spin, self.pivot_y_spin):
            spin.config(state='normal' if manual else 'disabled')

    def update_event_note(self):
        selected = self.event_options.get(self.event_label.get())
        event = next((v for v in self.close_events if v['index'] == selected), None)
        if event is None:
            self.event_note.set('未检测到可靠闭合，请尝试手动前N秒模式')
            return
        try:
            advance = float(self.advance.get())
            if not np.isfinite(advance):
                raise ValueError
            completion = event['time_s'] - advance
            self.event_note.set(f'预计 {completion:.2f} s 完成')
        except ValueError:
            self.event_note.set('请输入有效的提前秒数')

    def start(self):
        from tkinter import messagebox
        if self.busy: return
        src = self.records.get(self.file_name.get())
        if not src:
            messagebox.showwarning('请选择文件', '请先在列表中选择一份 teach_records 下的 NPZ 轨迹。')
            return
        try:
            values = np.array([float(self.dx.get()), float(self.dy.get()), float(self.dz.get())])
            mode = self.mode_labels.get(self.mode_label.get())
            ramp = float(self.ramp.get())
            yaw = float(self.yaw.get())
            pivot_mode = self.pivot_choice.get()
            pivot_xy = np.array([float(self.pivot_x.get()), float(self.pivot_y.get())]) if pivot_mode == 'manual' else None
            if not np.isfinite(yaw) or not -45 <= yaw <= 45:
                raise ValueError('绕 Z 轴的旋转角度必须在 -45°～+45°')
            if pivot_mode not in ('auto', 'manual'):
                raise ValueError('旋转中心类型无效')
            if pivot_xy is not None and (not np.isfinite(pivot_xy).all() or np.any(np.abs(pivot_xy) > 2.0)):
                raise ValueError('物体中心 X/Y 需为基坐标系下 ±2.0 m 内的有效数值')
            if not np.isfinite(values).all() or np.any(np.abs(values) > 10):
                raise ValueError('X、Y、Z 必须分别在 -10～+10 cm 之间')
            if mode not in ('full', 'early', 'instant', 'gripper'):
                raise ValueError('请选择补偿方式')
            event_idx = self.event_options.get(self.event_label.get())
            advance = float(self.advance.get()) if mode == 'gripper' else 1.0
            if mode == 'gripper':
                if event_idx is None:
                    raise ValueError('无法识别有效的夹爪闭合事件，请检查数据或改用前N秒模式')
                if not np.isfinite(advance) or not 0 <= advance <= 10:
                    raise ValueError('提前完成时间必须在 0~10 秒之间')
                event = next((ev for ev in self.close_events if ev['index'] == event_idx), None)
                if event is None or event['time_s']-advance < 0.25:
                    raise ValueError('提前量过大，补偿完成时刻不得早于0.25秒')
            if mode == 'early' and (not np.isfinite(ramp) or not 0 < ramp <= 10):
                raise ValueError('前N秒渐变时间应在 (0, 10] 秒')
        except ValueError as e:
            messagebox.showerror('输入有误', f'请检查参数：{e}')
            return
        self.busy = True
        self.last_result = None
        self.save_button.config(state='disabled')
        self.save_npz_check.config(state='disabled')
        self.save_png_check.config(state='disabled')
        self.run_button.config(state='disabled')
        self.refresh_button.config(state='disabled')
        self.combo.config(state='disabled')
        self.progress_bar['value'] = 0
        self.status.set('计算中…')
        self.result_label.set('正在离线计算，结果仅保存在内存中，暂不写入 NPZ 或 PNG。')
        self.log(f'开始：{src.name}，XYZ = {values.tolist()} cm，Yaw={yaw:.2f}°，旋转中心={pivot_mode}，补偿方式：{self.mode_label.get()}')

        def worker():
            try:
                def cb(stage, done, total):
                    self.events.put(('progress', stage, done, total))
                result = generate_preview(src, values, ramp, cb, mode=mode,
                                          close_event_index=event_idx, advance_seconds=advance,
                                          yaw_deg=yaw, pivot_mode=pivot_mode, pivot_xy_m=pivot_xy)
                self.events.put(('success', result))
            except Exception as e:
                self.events.put(('error', str(e), traceback.format_exc()))
        threading.Thread(target=worker, daemon=True).start()

    def start_save(self):
        from tkinter import messagebox
        if self.busy or self.last_result is None:
            return
        want_npz = bool(self.save_npz_var.get())
        want_png = bool(self.save_png_var.get())
        if not (want_npz or want_png):
            messagebox.showwarning('请选择保存内容', '请至少勾选保存 NPZ 或保存两张 PNG。\n不保存可以直接继续预览或处理下一条轨迹。')
            return
        self.busy = True
        self.run_button.config(state='disabled')
        self.refresh_button.config(state='disabled')
        self.combo.config(state='disabled')
        self.save_button.config(state='disabled')
        self.save_npz_check.config(state='disabled')
        self.save_png_check.config(state='disabled')
        self.status.set('正在保存所选文件…')
        self.log(f'保存选择：NPZ={want_npz}，两张PNG={want_png}')
        result = self.last_result

        def worker():
            try:
                def cb(stage, done, total):
                    self.events.put(('progress', stage, done, total))
                outcome = save_selected_outputs(
                    result, self.output_dir, want_npz, want_png, cb)
                self.events.put(('saved', outcome))
            except Exception as e:
                self.events.put(('save_error', str(e), traceback.format_exc()))
        threading.Thread(target=worker, daemon=True).start()

    def poll_events(self):
        from tkinter import messagebox
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == 'progress':
                    _, stage, done, total = event
                    self.status.set(f'{stage}（{done}/{total}）')
                    if stage == 'IK 计算中':
                        self.progress_bar['value'] = 85 * done / total
                    elif stage == '生成预览数据':
                        self.progress_bar['value'] = 90
                    elif stage == '预览就绪（尚未保存）':
                        self.progress_bar['value'] = 100
                elif event[0] == 'success':
                    self.finish_busy()
                    self.last_result = event[1]
                    info = self.last_result['stats']
                    self.status.set('轨迹已生成，等待保存选择')
                    self.progress_bar['value'] = 100
                    self.result_label.set(
                        f"已生成内存预览（尚未保存）：{self.last_result['source'].name}，"
                        f"XYZ = {self.last_result['xyz_cm'].tolist()} cm，Yaw={self.last_result['yaw_deg']:.2f}°。"
                        '勾选需要保存的类型，然后点击“保存所选结果”。')
                    self.log('轨迹生成成功；目前没有保存任何 NPZ 或 PNG 文件。')
                    self.log(f"旋转中心 XY={self.last_result['pivot_xy_m'].tolist()} m；{self.last_result['pivot_note']}")
                    self.log(f"完整补偿的绕Z角度={self.last_result['yaw_deg']:.3f}°（逆时针为正）")
                    self.log(f"N={info['N']}，频率={info['hz']:.2f}Hz，FK位置误差上限={info['max_pos_error_mm']:.4f}mm，"
                             f"峰值关节差={info['max_joint_jump_rad']:.4f}rad，峰值离散速度={info['max_joint_speed_rad_s']:.3f}rad/s")
                    self.log(f"首点位移={info['start_offset_mm']:.4f}mm，末点位移={info['end_offset_mm']:.4f}mm（数值模型）")
                    if self.last_result['gripper_event'] is not None:
                        ev = self.last_result['gripper_event']
                        self.log(f"检测到夹爪闭合：{ev['time_s']:.3f}s（索引{ev['index']}，{ev['source']}）；"
                                 f"在 {info['compensation_complete_time_s']:.3f}s 完成XYZ+Yaw补偿")
                    if info['max_joint_jump_rad'] > 0.08:
                        self.log('【警告】相邻轨迹点关节角差超过 0.08rad；不得直接用于实机回放！')
                    try:
                        self.show_preview(self.last_result)
                        self.show_gripper_plot(self.last_result)
                    except Exception as e:
                        self.log(f'GUI 预览失败（可尝试保存两张 PNG 查看）：{e}')
                    self.enable_save_controls()
                    messagebox.showinfo('轨迹生成成功',
                                        '新的关节轨迹已计算完成，三维图已准备好。\n\n'
                                        '目前没有自动保存文件。\n'
                                        '请在“结果保存”区域分别选择 NPZ 和/或两张 PNG，再点击“保存所选结果”。\n\n'
                                        '注意：还没有进行实机回放安全校验。')
                elif event[0] == 'saved':
                    self.finish_busy()
                    self.enable_save_controls()
                    outcome = event[1]
                    self.status.set('已完成保存')
                    for p in outcome['saved']:
                        self.log(f'已保存：{p}')
                    for p in outcome['skipped']:
                        self.log(f'已存在、未重复保存：{p}')
                    self.result_label.set(
                        f"当前预览：{self.last_result['source'].name}；"
                        f"已保存 {len(self.last_result['saved_paths'])}/3 个文件。"
                        f"目录：{self.output_dir}")
                    if outcome['saved']:
                        messagebox.showinfo('保存成功',
                                            '已保存：\n' + '\n'.join(p.name for p in outcome['saved']) +
                                            f'\n\n所在文件夹：{self.output_dir}')
                    else:
                        messagebox.showinfo('无需重复保存', '选中的文件之前已经保存过，未覆盖文件。')
                elif event[0] == 'save_error':
                    self.finish_busy()
                    self.enable_save_controls()
                    self.status.set('保存失败，可重新尝试')
                    self.log('保存失败：' + event[1])
                    self.log(event[2])
                    messagebox.showerror('保存失败', event[1])
                elif event[0] == 'error':
                    self.finish_busy()
                    self.status.set('处理失败')
                    self.result_label.set('未生成可用的修正轨迹。请检查运行记录。')
                    self.log('处理失败：' + event[1])
                    self.log(event[2])
                    messagebox.showerror('离线修正失败', event[1])
        except queue.Empty:
            pass
        self.root.after(120, self.poll_events)

    def enable_save_controls(self):
        if self.last_result is not None and not self.busy:
            self.save_button.config(state='normal')
            self.save_npz_check.config(state='normal')
            self.save_png_check.config(state='normal')

    def finish_busy(self):
        self.busy = False
        self.run_button.config(state='normal')
        self.refresh_button.config(state='normal')
        self.combo.config(state='readonly')

    def show_preview(self, result):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        if self.canvas:
            self.canvas.get_tk_widget().destroy()
            self.canvas = None
        self.preview_note.pack_forget()
        fig = Figure(figsize=(10.7, 4.8), dpi=95)
        old, new, bounds = result['old_xyz'], result['new_xyz'], result['bounds']
        for j, compensated in enumerate((False, True)):
            ax = fig.add_subplot(1, 2, j + 1, projection='3d')
            draw_3d(ax, old, new, bounds, compensated=compensated,
                    orientation_markers=result.get('orientation_markers'))
        fig.subplots_adjust(left=0, right=0.98, bottom=0.08, top=0.94, wspace=0.06)
        self.canvas = FigureCanvasTkAgg(fig, master=self.preview_tab)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

    def show_gripper_plot(self, result):
        """单独展示夹爪开合与补偿比例，让自动事件识别可核对。"""
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        if self.gripper_canvas:
            self.gripper_canvas.get_tk_widget().destroy()
            self.gripper_canvas = None
        fig = Figure(figsize=(10.7, 4.8), dpi=95)
        ax = fig.add_subplot(111)
        ts = result['gripper_time']
        ax.plot(ts, result['gripper_target_curve'], color='#2678b5',
                linewidth=1.25, label='Gripper target open')
        ax.plot(ts, result['gripper_actual_curve'], color='#26986c',
                linewidth=1.25, alpha=.8, label='Gripper actual open')
        if result['gripper_event'] is not None:
            ev = result['gripper_event']
            ax.axvline(ev['time_s'], color='#c33b56', linestyle='--',
                       label=f"Close onset {ev['time_s']:.2f}s")
            ax.axvline(result['stats']['compensation_complete_time_s'],
                       color='#d28b27', linestyle=':', linewidth=2,
                       label=f"Compensation done {result['stats']['compensation_complete_time_s']:.2f}s")
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Open fraction (larger = more open)')
        ax.grid(alpha=0.2)
        ax2 = ax.twinx()
        ax2.plot(ts, result['alpha_curve'], color='#9a69ca', alpha=.75,
                 linewidth=1.5, label='Compensation ratio')
        ax2.set_ylabel(f"Compensation ratio (Yaw = {result.get('yaw_deg', 0.0):+.1f} deg at 100%)")
        ax2.set_ylim(-.08, 1.08)
        ax.legend(loc='upper left', fontsize=8)
        ax2.legend(loc='upper right', fontsize=8)
        fig.subplots_adjust(left=0.07, right=0.92, bottom=0.14, top=0.95)
        self.gripper_canvas = FigureCanvasTkAgg(fig, master=self.gripper_tab)
        self.gripper_canvas.draw()
        self.gripper_canvas.get_tk_widget().pack(fill='both', expand=True)

    def open_folder(self):
        from tkinter import messagebox
        folder = self.output_dir if self.output_dir.is_dir() else self.record_dir
        if not folder.is_dir():
            messagebox.showwarning('目录不存在', f'找不到文件夹：\n{folder}')
            return
        try:
            if sys.platform == 'win32':
                os.startfile(str(folder))
            elif sys.platform == 'darwin':
                import subprocess
                subprocess.Popen(['open', str(folder)])
            else:
                import subprocess
                subprocess.Popen(['xdg-open', str(folder)])
        except Exception as e:
            messagebox.showerror('打开失败', str(e))


def main():
    import tkinter as tk
    root = tk.Tk()
    ShiftGui(root)
    root.mainloop()


if __name__ == '__main__':
    main()
