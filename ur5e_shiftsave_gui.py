# -*- coding: utf-8 -*-
"""
UR5e 离线示教轨迹平移 / 自主保存 GUI
=========================
文件夹结构（默认）：
    ur5e_shiftsave_gui.py
    teach_records/
        teach_xxx.npz
        shift_outputs/            # 自动生成

运行：python ur5e_shiftsave_gui.py
依赖：pip install numpy scipy matplotlib
GUI：Python 自带 tkinter（部分 Linux 环境需额外安装 python3-tk）

仅进行离线运动学计算，不连接机器人、不下发运动指令。
使用 UR5e 名义 DH 模型的 tool0（法兰）坐标，未包含实际机器人标定及夹爪 TCP。
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


def blend_alpha(t: float, seconds: float) -> float:
    if seconds <= 0:
        return 1.0
    s = float(np.clip(t / seconds, 0, 1))
    return 10 * s**3 - 15 * s**4 + 6 * s**5


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


def compute_shift(data: dict[str, np.ndarray], offset_m: np.ndarray,
                  ramp_seconds: float,
                  progress: Callable[[int, int], None] | None = None):
    q0 = np.asarray(data['ur_q'], dtype=float)
    n = len(q0)
    hz = frequency_of(data)
    ts = np.arange(n, dtype=float) / hz
    q1 = np.empty_like(q0)
    max_pos_error = 0.0
    max_rot_error = 0.0
    last_original = last_solution = last_offset = None

    for i, q in enumerate(q0):
        delta = offset_m * blend_alpha(ts[i], ramp_seconds)
        if (last_original is not None and np.array_equal(q, last_original)
                and np.array_equal(delta, last_offset)):
            solution = last_solution
        else:
            target, _ = fk_jacobian(q)
            target[:3, 3] += delta
            if np.max(np.abs(delta)) < 1e-14:
                solution = q.copy()
            else:
                seed = q if last_solution is None else last_solution + (q - last_original)
                try:
                    solution = ik_near(target, seed)
                except Exception as exc:
                    raise RuntimeError(f'第 {i+1}/{n} 个轨迹点（t={ts[i]:.3f} s）：{exc}') from exc
            if np.any(np.abs(solution) > JOINT_LIMIT + 1e-7):
                raise RuntimeError(f'第 {i+1} 点的新关节角超出 ±2π 名义范围，未保存结果')
            achieved, _ = fk_jacobian(solution)
            ep, er = pose_error(target, achieved)
            max_pos_error = max(max_pos_error, float(np.linalg.norm(ep)))
            max_rot_error = max(max_rot_error, float(np.linalg.norm(er)))
        q1[i] = solution
        last_original, last_solution, last_offset = q, solution, delta
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
    }
    return q1, ts, info


def safe_part(v: float) -> str:
    # 命名使用 cm，精度到 0.001 cm，不与不同偏移量混淆
    if abs(v) < 0.0005:
        return '0'
    return ('p' if v > 0 else 'm') + f'{abs(v):.3f}'.rstrip('0').rstrip('.').replace('.', 'd')


def make_output_base(source: Path, xyz_cm: np.ndarray, folder: Path) -> Path:
    tag = '_'.join(axis + safe_part(float(v)) for axis, v in zip('xyz', xyz_cm))
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
            compensated: bool):
    if compensated:
        ax.plot(*old.T, color='#97a3ae', linestyle='--', alpha=0.7,
                linewidth=1.5, label='Original reference')
        points, color, label = new, '#dc7d23', 'Compensated'
        ax.plot(np.array([old[-1, 0], new[-1, 0]]),
                np.array([old[-1, 1], new[-1, 1]]),
                np.array([old[-1, 2], new[-1, 2]]),
                color='#9155b5', linewidth=2.0, label='End displacement')
    else:
        points, color, label = old, '#2678b5', 'Original'
    ax.plot(*points.T, color=color, linewidth=1.8, label=label)
    ax.scatter(*points[0], s=28, marker='o', color='#209e78', label='Start')
    ax.scatter(*points[-1], s=35, marker='^', color='#c33b56', label='End')
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
                bounds: np.ndarray, shifted: bool):
    # 不依赖 Tk 显示环境，可在后台线程直接保存 PNG
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(7.8, 6.1), dpi=155)
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, projection='3d')
    draw_3d(ax, old, new, bounds, compensated=shifted)
    fig.subplots_adjust(left=0.02, right=0.94, bottom=0.07, top=0.92)
    fig.savefig(path, dpi=155)
    fig.clear()


def generate_preview(source: Path, xyz_cm: np.ndarray, ramp: float,
                     progress: Callable[[str, int, int], None] | None = None):
    """只计算，不写入磁盘：返回 NPZ 内容和两张图片共用的三维预览数据。"""
    xyz_cm = np.asarray(xyz_cm, dtype=float)
    if xyz_cm.shape != (3,) or not np.isfinite(xyz_cm).all() or np.any(np.abs(xyz_cm) > 10):
        raise ValueError('X/Y/Z 位移分别必须在 -10 到 +10 cm 范围内')
    if not np.isfinite(ramp) or not (0 <= ramp <= 10):
        raise ValueError('渐变时间必须在 0 到 10 秒范围内')
    source = Path(source).resolve()
    if not source.is_file() or source.suffix.lower() != '.npz':
        raise FileNotFoundError(f'原始 NPZ 不存在：{source}')
    record = load_record(source)
    q0 = np.asarray(record['ur_q'], dtype=float)

    def on_progress(i, n):
        if progress:
            progress('IK 计算中', i, n)

    q1, uniform_ts, stats = compute_shift(record, xyz_cm / 100, ramp, on_progress)
    if progress:
        progress('生成预览数据', 0, 1)
    ids = np.linspace(0, len(q0) - 1, num=min(len(q0), 2400), dtype=int)
    old_xyz = sampled_xyz(q0, ids)
    new_xyz = sampled_xyz(q1, ids)
    bounds = common_bounds(old_xyz, new_xyz)

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
        'ramp_seconds': np.array([ramp], dtype=float),
        'nominal_dh_model': np.array(['UR5e_tool0']),
        'max_fk_position_error_m': np.array([stats['max_pos_error_mm'] / 1000]),
        'max_fk_orientation_error_rad': np.array([stats['max_rot_error_rad']]),
    }
    if progress:
        progress('预览就绪（尚未保存）', 1, 1)
    return {'source': source, 'xyz_cm': xyz_cm.copy(), 'ramp': ramp,
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
        result['base'] = make_output_base(result['source'], result['xyz_cm'], output_dir)
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
                path, result['old_xyz'], result['new_xyz'], result['bounds'], shifted=False))
            created_png_this_time.extend(saved[count:])
            if progress:
                progress('保存两张 PNG', 1, 2)
            count = len(saved)
            atomic_save('compensated_png', lambda path: write_image(
                path, result['old_xyz'], result['new_xyz'], result['bounds'], shifted=True))
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
        self.root.title('UR5e 离线轨迹修正')
        self.root.geometry('1180x850')
        self.root.minsize(940, 670)
        self.file_name = tk.StringVar()
        self.dx = tk.StringVar(value='5.0')
        self.dy = tk.StringVar(value='0.0')
        self.dz = tk.StringVar(value='0.0')
        self.ramp = tk.StringVar(value='2.0')
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
        ttk.Label(main, text='从 teach_records 选择 NPZ，设置基坐标系 XYZ 偏移；先生成并预览，再决定是否保存（不会控制真实机器人）。',
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

        setting = ttk.LabelFrame(main, text='2 设置末端平移量（单位：cm，机器人基坐标系）', padding=10)
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
        under = ttk.Frame(setting); under.pack(fill='x', pady=(10, 0))
        ttk.Label(under, text='起步渐变：').pack(side='left')
        self.ramp_spin = ttk.Spinbox(under, from_=0, to=10, increment=0.5,
                                     textvariable=self.ramp, width=9, justify='center')
        self.ramp_spin.pack(side='left')
        ttk.Label(under, text='秒   （默认 2 秒逐渐加入补偿；0 秒表示整条轨迹直接平移，起点也变化）',
                  foreground='#677782').pack(side='left', padx=(7, 0))

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
        notebook.add(self.preview_tab, text='三维轨迹预览（两张图）')
        notebook.add(self.log_tab, text='运行记录')
        self.preview_note = ttk.Label(self.preview_tab, text='计算完成后，这里会显示原始轨迹和补偿后的轨迹。',
                                      foreground='#6a7881')
        self.preview_note.pack(pady=30)
        self.canvas = None
        self.log_widget = tk.Text(self.log_tab, height=12, wrap='word', state='disabled')
        self.log_widget.pack(side='left', fill='both', expand=True)
        bar = ttk.Scrollbar(self.log_tab, command=self.log_widget.yview)
        bar.pack(side='right', fill='y')
        self.log_widget.config(yscrollcommand=bar.set)
        ttk.Label(main, text='安全提示：仅供离线实验；使用名义 DH 法兰 tool0，不包含实际夹爪 TCP、机器人标定、碰撞与加速度校验。',
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
            self.summary.set('文件夹中尚无 NPZ 文件。')
            return
        try:
            with np.load(path, allow_pickle=False) as d:
                if 'ur_q' not in d:
                    raise ValueError('缺少 ur_q')
                q = d['ur_q']
                hz = float(d['rtde_control_frequency_hz'].ravel()[0]) if 'rtde_control_frequency_hz' in d else None
                msg = f'轨迹点：{len(q)}  |  UR关节维度：{q.shape}  |  频率：{hz:g} Hz' if hz else f'轨迹点：{len(q)}  |  UR关节维度：{q.shape}'
                self.summary.set(msg)
        except Exception as e:
            self.summary.set(f'文件可能无法读取：{e}')

    def start(self):
        from tkinter import messagebox
        if self.busy: return
        src = self.records.get(self.file_name.get())
        if not src:
            messagebox.showwarning('请选择文件', '请先在列表中选择一份 teach_records 下的 NPZ 轨迹。')
            return
        try:
            values = np.array([float(self.dx.get()), float(self.dy.get()), float(self.dz.get())])
            ramp = float(self.ramp.get())
            if not np.isfinite(values).all() or np.any(np.abs(values) > 10):
                raise ValueError('X、Y、Z 必须分别在 -10～+10 cm 之间')
            if not np.isfinite(ramp) or not 0 <= ramp <= 10:
                raise ValueError('渐变时间必须在 0～10 秒之间')
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
        self.log(f'开始：{src.name}，XYZ = {values.tolist()} cm，渐变 {ramp:g} s')

        def worker():
            try:
                def cb(stage, done, total):
                    self.events.put(('progress', stage, done, total))
                result = generate_preview(src, values, ramp, cb)
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
                        f"XYZ = {self.last_result['xyz_cm'].tolist()} cm。"
                        '勾选需要保存的类型，然后点击“保存所选结果”。')
                    self.log('轨迹生成成功；目前没有保存任何 NPZ 或 PNG 文件。')
                    self.log(f"N={info['N']}，频率={info['hz']:.2f}Hz，FK位置误差上限={info['max_pos_error_mm']:.4f}mm，"
                             f"峰值关节差={info['max_joint_jump_rad']:.4f}rad，峰值离散速度={info['max_joint_speed_rad_s']:.3f}rad/s")
                    if info['max_joint_jump_rad'] > 0.08:
                        self.log('【警告】相邻轨迹点关节角差超过 0.08rad；不得直接用于实机回放！')
                    try:
                        self.show_preview(self.last_result)
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
            draw_3d(ax, old, new, bounds, compensated=compensated)
        fig.subplots_adjust(left=0, right=0.98, bottom=0.08, top=0.94, wspace=0.06)
        self.canvas = FigureCanvasTkAgg(fig, master=self.preview_tab)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

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
