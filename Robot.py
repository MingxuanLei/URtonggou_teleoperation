"""Python robot model ported from Robot_UR.cs for the UR-isomorphic DM arm.

The kinematics, Jacobian and gravity-compensation calculations follow the
original C# implementation.  The kinematic/dynamic model follows Robot_UR.cs.  ``dh2motor`` is kept as
an explicit coordinate-conversion helper; safety limits are intentionally
handled outside this mathematical model.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple, List

import numpy as np


class Robot:
    DOF = 6

    def __init__(self) -> None:
        # Same direction mapping as Robot_UR.cs.
        self.ratio = np.array([1.0, 1.0, -1.0, 1.0, 1.0, 1.0], dtype=float)
        self.Fh = np.zeros(6, dtype=float)

        # UR-isomorphic master-arm geometric parameters from the C# source.
        self.a2 = -0.210
        self.a3 = -0.200
        self.d4 = 0.062
        self.d5 = 0.073
        self.d6 = 0.036

        self._q = np.zeros(6, dtype=float)
        self._pos = np.zeros(3, dtype=float)
        self._rpy = np.zeros(3, dtype=float)
        self._trans = np.zeros((4, 4), dtype=float)
        self._rot = np.zeros((3, 3), dtype=float)
        self._jacob0 = np.zeros((6, 6), dtype=float)
        self._jacob6 = np.zeros((6, 6), dtype=float)

        self._tau_g = np.zeros(6, dtype=float)
        self._tau_fh = np.zeros(6, dtype=float)
        self._tau = np.zeros(6, dtype=float)
        self._safe = False

        # [mass, COM_x, COM_y, COM_z], copied from Robot_UR.cs.
        mr = np.array(
            [
                [0.548728, 0.0, -0.002649, -0.003134],
                [0.750278, -0.164665, 0.0, 0.061074],
                [0.653139, -0.151553, 0.0, -0.002837],
                [0.431678, 0.0, -0.004718, -0.001152],
                [0.431678, 0.0, 0.004718, -0.001152],
                [0.277642, -0.000290, 0.009891, 0.037114],
            ],
            dtype=float,
        )
        self._m = mr[:, 0].copy()
        self._r = [mr[i, 1:4].copy() for i in range(6)]
        self._g0 = np.array([0.0, 0.0, -9.80665], dtype=float)

        # Keep rotation matrices initialized so G_Tool is always readable.
        self._R06 = np.eye(3, dtype=float)

        self._safe = True
        self.set_robot()

    # ------------------------------------------------------------------
    # Public properties matching the C# API
    # ------------------------------------------------------------------
    @property
    def Angle(self) -> List[float]:
        return self._q.copy().tolist()

    @Angle.setter
    def Angle(self, value: Sequence[float]) -> None:
        try:
            values = np.asarray(value, dtype=float).reshape(-1)
        except Exception:
            self._safe = False
            return
        if values.size != self.DOF:
            self._safe = False
            return
        self._q = np.array([self.angle_clip_pnpi(v) for v in values], dtype=float)
        self._safe = bool(np.all(np.isfinite(self._q)))

    @property
    def Position(self) -> List[float]:
        return self._pos.copy().tolist()

    @property
    def RPY(self) -> List[float]:
        return self._rpy.copy().tolist()

    @property
    def TransMatrix(self) -> np.ndarray:
        return self._trans.copy()

    @property
    def RotMatrix(self) -> np.ndarray:
        return self._rot.copy()

    @property
    def Jacob0(self) -> np.ndarray:
        return self._jacob0.copy()

    @property
    def Jacob6(self) -> np.ndarray:
        return self._jacob6.copy()

    @property
    def Tau_G(self) -> List[float]:
        return self._tau_g.copy().tolist()

    @property
    def Tau_Fh(self) -> List[float]:
        return self._tau_fh.copy().tolist()

    @property
    def Tau(self) -> List[float]:
        return self._tau.copy().tolist()

    @property
    def Tau_G_Motor(self) -> List[float]:
        return (self._tau_g / self.ratio).tolist()

    @property
    def Tau_Fh_Motor(self) -> List[float]:
        return (self._tau_fh / self.ratio).tolist()

    @property
    def G_Tool(self) -> List[float]:
        return (self._R06 @ self._g0).tolist()

    # ------------------------------------------------------------------
    # Motor <-> DH angle conversion
    # ------------------------------------------------------------------
    def motor2dh(self, motors: Sequence[object]) -> List[float]:
        """Convert the first six motor feedback positions to DH joint angles.

        This is a direct port of Robot_UR.cs::motor2dh().
        """
        if len(motors) < self.DOF:
            raise ValueError("motor2dh requires at least 6 motors")
        dh = []
        for i in range(self.DOF):
            pos = float(motors[i].Position)
            dh.append(self.angle_clip_pnpi(pos / float(self.ratio[i])))
        return dh

    def dh2motor(
        self,
        motors: Sequence[object],
        target_dh_q: Sequence[float],
    ) -> Tuple[bool, List[float], List[bool]]:
        """Convert DH joint targets to raw motor-position targets.

        IMPORTANT
        ---------
        This function performs *coordinate conversion only*.  It deliberately
        does NOT enforce mechanical/software safety limits.

        The original C# Robot_UR.cs only defines the direction mapping
        ``motor2dh = motor_position / ratio``.  Therefore the inverse mapping
        is simply:

            motor_position = dh_angle * ratio

        Safety checks belong to the controller/UI layer, where their meaning
        is explicit (DH soft limit, maximum step, velocity limit, feedback
        validity, protocol encoding range, etc.).

        Returns
        -------
        ok:
            True when six finite DH values were supplied.
        motor_targets:
            Six raw motor-position targets in radians.
        valid:
            Per-joint input-valid flags.  These are NOT mechanical-limit flags.
        """
        if len(motors) < self.DOF or len(target_dh_q) != self.DOF:
            return False, [], [False] * self.DOF

        targets: List[float] = []
        valid: List[bool] = []

        for i in range(self.DOF):
            try:
                q_dh = float(target_dh_q[i])
            except Exception:
                targets.append(float("nan"))
                valid.append(False)
                continue

            if not math.isfinite(q_dh):
                targets.append(float("nan"))
                valid.append(False)
                continue

            targets.append(q_dh * float(self.ratio[i]))
            valid.append(True)

        return all(valid), targets, valid

    # ------------------------------------------------------------------
    # Kinematics, Jacobian, gravity and external-force mapping
    # ------------------------------------------------------------------
    def set_robot(self) -> bool:
        if not self._safe:
            print("无解！")
            return False

        q = self._q
        s = np.sin(q)
        c = np.cos(q)

        T10 = np.array(
            [[c[0], -s[0], 0, 0], [s[0], c[0], 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        T21 = np.array(
            [[c[1], -s[1], 0, 0], [0, 0, -1, 0], [s[1], c[1], 0, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        T32 = np.array(
            [[c[2], -s[2], 0, self.a2], [s[2], c[2], 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        T43 = np.array(
            [[c[3], -s[3], 0, self.a3], [s[3], c[3], 0, 0], [0, 0, 1, self.d4], [0, 0, 0, 1]],
            dtype=float,
        )
        T54 = np.array(
            [[c[4], -s[4], 0, 0], [0, 0, -1, -self.d5], [s[4], c[4], 0, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        T65 = np.array(
            [[c[5], -s[5], 0, 0], [0, 0, 1, self.d6], [-s[5], -c[5], 0, 0], [0, 0, 0, 1]],
            dtype=float,
        )

        T20 = T10 @ T21
        T30 = T20 @ T32
        T40 = T30 @ T43
        T50 = T40 @ T54
        T60 = T50 @ T65

        T61 = T21 @ T32 @ T43 @ T54 @ T65
        T62 = T32 @ T43 @ T54 @ T65
        T63 = T43 @ T54 @ T65
        T64 = T54 @ T65

        self._trans = T60.copy()
        self._pos = self.trans2pos(T60)
        self._rot = self.trans2rot(T60)
        # The C# code calls rot2rpy(trans); rot2rpy reads only the top-left 3x3.
        self._rpy = self.rot2rpy(T60)

        R10, R21, R32 = T10[:3, :3], T21[:3, :3], T32[:3, :3]
        R43, R54, R65 = T43[:3, :3], T54[:3, :3], T65[:3, :3]
        R20, R30, R40 = T20[:3, :3], T30[:3, :3], T40[:3, :3]
        R50, R60 = T50[:3, :3], T60[:3, :3]

        R01, R02, R03 = R10.T, R20.T, R30.T
        R04, R05, R06 = R40.T, R50.T, R60.T
        self._R06 = R06.copy()

        P10, P21, P32 = T10[:3, 3], T21[:3, 3], T32[:3, 3]
        P43, P54, P65 = T43[:3, 3], T54[:3, 3], T65[:3, 3]
        _ = (P10, P21, P32, P43, P54)  # retained conceptually from C# source

        Z1, Z2, Z3 = T10[:3, 2], T20[:3, 2], T30[:3, 2]
        Z4, Z5, Z6 = T40[:3, 2], T50[:3, 2], T60[:3, 2]

        P61, P62 = T61[:3, 3], T62[:3, 3]
        P63, P64 = T63[:3, 3], T64[:3, 3]
        P66 = np.zeros(3, dtype=float)

        J1 = np.concatenate((np.cross(Z1, R10 @ P61), Z1))
        J2 = np.concatenate((np.cross(Z2, R20 @ P62), Z2))
        J3 = np.concatenate((np.cross(Z3, R30 @ P63), Z3))
        J4 = np.concatenate((np.cross(Z4, R40 @ P64), Z4))
        J5 = np.concatenate((np.cross(Z5, R50 @ P65), Z5))
        J6 = np.concatenate((np.cross(Z6, R60 @ P66), Z6))

        Ja0 = np.column_stack((J1, J2, J3, J4, J5, J6))
        zero3 = np.zeros((3, 3), dtype=float)
        block_R06 = np.block([[R06, zero3], [zero3, R06]])
        Ja6 = block_R06 @ Ja0

        self._jacob0 = Ja0.copy()
        self._jacob6 = Ja6.copy()

        G1 = R01 @ self._g0 * self._m[0]
        G2 = R02 @ self._g0 * self._m[1]
        G3 = R03 @ self._g0 * self._m[2]
        G4 = R04 @ self._g0 * self._m[3]
        G5 = R05 @ self._g0 * self._m[4]
        G6 = R06 @ self._g0 * self._m[5]

        F6 = -G6
        F5 = R65 @ F6 - G5
        F4 = R54 @ F5 - G4
        F3 = R43 @ F4 - G3
        F2 = R32 @ F3 - G2
        F1 = R21 @ F2 - G1
        _ = F1  # computed in the original recursive Newton-Euler chain

        r1, r2, r3, r4, r5, r6 = self._r
        M6 = -np.cross(r6, G6)
        M5 = R65 @ M6 + np.cross(P65, R65 @ F6) - np.cross(r5, G5)
        M4 = R54 @ M5 + np.cross(P54, R54 @ F5) - np.cross(r4, G4)
        M3 = R43 @ M4 + np.cross(P43, R43 @ F4) - np.cross(r3, G3)
        M2 = R32 @ M3 + np.cross(P32, R32 @ F3) - np.cross(r2, G2)
        M1 = R21 @ M2 + np.cross(P21, R21 @ F2) - np.cross(r1, G1)

        self._tau_g = np.array([M1[2], M2[2], M3[2], M4[2], M5[2], M6[2]], dtype=float)
        fh = np.asarray(self.Fh, dtype=float).reshape(-1)
        if fh.size != 6 or not np.all(np.isfinite(fh)):
            self._safe = False
            print("无解！")
            return False
        self._tau_fh = Ja0.T @ fh
        self._tau = self._tau_g + self._tau_fh
        return True

    # ------------------------------------------------------------------
    # Angle and transform helpers ported from Robot_UR.cs
    # ------------------------------------------------------------------
    @staticmethod
    def _islegal(a: float) -> bool:
        return math.isfinite(float(a))

    @staticmethod
    def angle_clip_pnpi(q: float) -> float:
        q = float(q)
        if not Robot._islegal(q):
            return float("nan")
        # Matches C#: output interval (-pi, pi].
        while q > math.pi or q <= -math.pi:
            if q > math.pi:
                q -= 2.0 * math.pi
                continue
            if q <= -math.pi:
                q += 2.0 * math.pi
        return q

    @staticmethod
    def angle_clip_02pi(q: float) -> float:
        q = float(q)
        while q >= 2.0 * math.pi or q < 0.0:
            if q >= 2.0 * math.pi:
                q -= 2.0 * math.pi
                continue
            if q < 0.0:
                q += 2.0 * math.pi
        return q

    @staticmethod
    def minor_arc(angle1: float, angle2: float) -> float:
        return abs(Robot.minor_arc_dir(angle1, angle2))

    @staticmethod
    def minor_arc_dir(start, target):
        if np.isscalar(start) and np.isscalar(target):
            return Robot.angle_clip_pnpi(2.0 * math.pi - (float(start) - float(target)))
        s = np.asarray(start, dtype=float).reshape(-1)
        t = np.asarray(target, dtype=float).reshape(-1)
        if s.size != t.size:
            raise ValueError("start and target must have the same length")
        return [Robot.minor_arc_dir(a, b) for a, b in zip(s, t)]

    @staticmethod
    def rotpos2trans(Rot: Sequence[Sequence[float]], Pos: Sequence[float]) -> np.ndarray:
        R = np.asarray(Rot, dtype=float)
        p = np.asarray(Pos, dtype=float).reshape(-1)
        if R.shape != (3, 3) or p.size != 3:
            raise ValueError("Rot must be 3x3 and Pos must contain 3 values")
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = p
        return T

    @staticmethod
    def trans2rot(T: Sequence[Sequence[float]]) -> np.ndarray:
        arr = np.asarray(T, dtype=float)
        if arr.shape[0] < 3 or arr.shape[1] < 3:
            raise ValueError("T must be at least 3x3")
        return arr[:3, :3].copy()

    @staticmethod
    def trans2pos(T: Sequence[Sequence[float]]) -> np.ndarray:
        arr = np.asarray(T, dtype=float)
        if arr.shape[0] < 3 or arr.shape[1] < 4:
            raise ValueError("T must be at least 3x4")
        return arr[:3, 3].copy()

    @staticmethod
    def rpy2rot(RPY: Sequence[float]) -> np.ndarray:
        a, b, c = [float(x) for x in RPY]
        sinA, cosA = math.sin(a), math.cos(a)
        sinB, cosB = math.sin(b), math.cos(b)
        sinC, cosC = math.sin(c), math.cos(c)
        return np.array(
            [
                [cosB * cosC, cosC * sinA * sinB - cosA * sinC, sinA * sinC + cosA * cosC * sinB],
                [cosB * sinC, cosA * cosC + sinA * sinB * sinC, cosA * sinB * sinC - cosC * sinA],
                [-sinB, cosB * sinA, cosA * cosB],
            ],
            dtype=float,
        )

    @staticmethod
    def rot_x(a: float) -> np.ndarray:
        ca, sa = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]], dtype=float)

    @staticmethod
    def rot_y(b: float) -> np.ndarray:
        cb, sb = math.cos(b), math.sin(b)
        return np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]], dtype=float)

    @staticmethod
    def rot_z(c: float) -> np.ndarray:
        cc, sc = math.cos(c), math.sin(c)
        return np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]], dtype=float)

    @staticmethod
    def rot2rpy(R: Sequence[Sequence[float]]) -> np.ndarray:
        arr = np.asarray(R, dtype=float)
        if arr.shape[0] < 3 or arr.shape[1] < 3:
            raise ValueError("R must be at least 3x3")

        if abs(arr[2, 0] - 1.0) < 1.0e-15:
            a = 0.0
            b = -math.pi / 2.0
            c = math.atan2(-arr[0, 1], -arr[0, 2])
        elif abs(arr[2, 0] + 1.0) < 1.0e-15:
            a = 0.0
            b = math.pi / 2.0
            c = -math.atan2(arr[0, 1], arr[0, 2])
        else:
            a = math.atan2(arr[2, 1], arr[2, 2])
            c = math.atan2(arr[1, 0], arr[0, 0])
            cosC = math.cos(c)
            sinC = math.sin(c)
            if abs(cosC) > abs(sinC):
                b = math.atan2(-arr[2, 0], arr[0, 0] / cosC)
            else:
                b = math.atan2(-arr[2, 0], arr[1, 0] / sinC)

        return np.array([a, b, c], dtype=float)


