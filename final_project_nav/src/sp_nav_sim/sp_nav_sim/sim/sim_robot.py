import math
import threading
from typing import Optional, Tuple

import numpy as np

from sp_nav_sim.sim.sim_map import (
    CHASSIS_HALF,
    SimMapMeta,
    clamp_norm,
    in_collision_world,
    oriented_square_mtv,
    segment_in_collision,
    estimate_obstacle_normal,
    find_nearest_free_world,
)
from sp_nav_sim.sim._dyn import VelocityFilter

class SimRobot:

    def __init__(
        self,
        robot_id: int,
        init_pos: np.ndarray,
        v_max: float,
        a_max: float,
        robot_radius: float,
        map_occ: np.ndarray,
        meta: SimMapMeta,
        map_w: int,
        map_h: int,
        tf_child_frame_id: str = 'base_link',
        sim_hz: float = 30.0,
        scan_speed: float = 1.0,
    ):
        self.robot_id = robot_id
        self.v_max = v_max
        self.a_max = a_max
        self.robot_radius = robot_radius
        self.chassis_half = CHASSIS_HALF
        self.map_occ = map_occ
        self.meta = meta
        self.map_w = map_w
        self.map_h = map_h
        self.tf_child_frame_id = tf_child_frame_id
        self.dt = 1.0 / max(sim_hz, 1.0)
        self.scan_speed = scan_speed

        self._vf = VelocityFilter(self.dt)

        self.pos = init_pos.copy().astype(float)
        self.vel = np.array([0.0, 0.0], dtype=float)
        self.yaw = 0.0
        self.target = self.pos.copy()
        self.dragging = False

        self.gimbal_mode: int = 2   #不知道为什么自转会影响导航
        self.gimbal_big_yaw_deg: float = 0.0
        self.gimbal_yaw_lower_deg: float = -180.0
        self.gimbal_yaw_upper_deg: float = 180.0
        self._yaw_scan_dir: float = 1.0

        self._cmd_vel_vx_bl: float = 0.0
        self._cmd_vel_vy_bl: float = 0.0
        self._cmd_vel_last_time_ns: Optional[int] = None
        self._cmd_vel_timeout_ns: int = int(5.0 * 1e9)

        self._chassis_stop: bool = False
        self._hp_disabled: bool = False

        self.chassis_yaw: float = 0.0
        self.chassis_mode: int = 0
        self.chassis_rotate_velocity: float = 3.0
        self.chassis_wz: float = 0.0
        self._wall_contact: bool = False
        self.peers = []

        self.z_vel_override = 0.0

        self.lock = threading.Lock()

        self._fix_initial_collision()

    def _sync_vf_vel(self):
        self._vf.set_vel(float(self.vel[0]), float(self.vel[1]))

    def _clear_motion(self):
        self.vel = np.array([0.0, 0.0], dtype=float)
        self._vf.reset()

    def _plant_step_bl(self, vx_bl: float, vy_bl: float) -> np.ndarray:
        vx, vy = self._vf.step(
            vx_bl, vy_bl, self.yaw, self.chassis_yaw, self.chassis_wz,
            self.v_max, self.a_max)
        return np.array([vx, vy], dtype=float)

    def _hits_peer(self, pos: np.ndarray) -> bool:
        half = self.chassis_half
        for peer in self.peers:
            if oriented_square_mtv(
                pos, self.chassis_yaw, half,
                peer.pos, peer.chassis_yaw, peer.chassis_half,
            ) is not None:
                return True
        return False

    def _peer_push_normal(self, pos: np.ndarray) -> Optional[np.ndarray]:
        best = None
        best_len = 0.0
        half = self.chassis_half
        for peer in self.peers:
            mtv = oriented_square_mtv(
                pos, self.chassis_yaw, half,
                peer.pos, peer.chassis_yaw, peer.chassis_half,
            )
            if mtv is None:
                continue
            length = float(np.linalg.norm(mtv))
            if length > best_len:
                best_len = length
                best = mtv
        if best is None or best_len < 1e-9:
            return None
        return best / best_len

    def _in_collision(self, pos: np.ndarray) -> bool:
        if in_collision_world(
            pos, self.map_occ, self.meta, self.map_w, self.map_h,
            self.chassis_half, self.chassis_yaw,
        ):
            return True
        return self._hits_peer(pos)

    def _seg_collision(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        if segment_in_collision(
            p0, p1, self.map_occ, self.meta, self.map_w, self.map_h,
            self.chassis_half, self.chassis_yaw,
        ):
            return True
        dist = float(np.linalg.norm(p1 - p0))
        if dist < 1e-9:
            return self._hits_peer(p1)
        step = max(float(self.meta.resolution) * 0.5, 0.02)
        n = int(math.ceil(dist / step))
        for i in range(1, n + 1):
            a = i / float(n)
            if self._hits_peer((1.0 - a) * p0 + a * p1):
                return True
        return False

    def _obstacle_normal(self, pos: np.ndarray) -> Optional[np.ndarray]:
        return estimate_obstacle_normal(
            pos, self.map_occ, self.meta, self.map_w, self.map_h, self.chassis_half,
        )

    def _free_normal(self, pos: np.ndarray) -> Optional[np.ndarray]:
        normals = []
        if in_collision_world(
            pos, self.map_occ, self.meta, self.map_w, self.map_h,
            self.chassis_half, self.chassis_yaw,
        ):
            n = self._obstacle_normal(pos)
            if n is not None:
                into = in_collision_world(
                    pos + n * 0.05, self.map_occ, self.meta, self.map_w, self.map_h,
                    self.chassis_half, self.chassis_yaw,
                )
                out = in_collision_world(
                    pos - n * 0.05, self.map_occ, self.meta, self.map_w, self.map_h,
                    self.chassis_half, self.chassis_yaw,
                )
                if into and not out:
                    n = -n
                normals.append(n)
        peer_n = self._peer_push_normal(pos)
        if peer_n is not None:
            normals.append(peer_n)
        if not normals:
            return None
        nsum = np.sum(np.stack(normals, axis=0), axis=0)
        length = float(np.linalg.norm(nsum))
        if length < 1e-9:
            return normals[0]
        return nsum / length

    def _separate_to_surface(self):

        if not self._in_collision(self.pos):
            return
        n = self._free_normal(self.pos)
        dirs = []
        if n is not None:
            dirs.append(n)
        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            dirs.append(np.array([math.cos(rad), math.sin(rad)], dtype=float))
        step = max(float(self.meta.resolution), 0.01)
        for direction in dirs:
            pos = self.pos.copy()
            for _ in range(int(0.80 / step) + 1):
                pos = pos + direction * step
                if not self._in_collision(pos):
                    self.pos = pos
                    return

    def _apply_velocity(self, vel_new: np.ndarray) -> bool:
        vel_new = clamp_norm(vel_new, self.v_max)
        pos_new = self.pos + vel_new * self.dt
        if not self._seg_collision(self.pos, pos_new):
            self.pos = pos_new
            self.vel = vel_new
            return False

        n = self._free_normal(self.pos)
        if n is None:
            n = self._free_normal(pos_new)
        if n is None:
            self.vel = np.zeros(2, dtype=float)
            self._sync_vf_vel()
            return True

        vn = float(np.dot(vel_new, n))
        tangent = vel_new - vn * n
        candidates = [vel_new] if vn >= 0.0 else [tangent]
        if vn >= 0.0 and float(np.linalg.norm(tangent)) > 1e-4:
            candidates.append(tangent)
        for vel_try in candidates:
            vel_try = clamp_norm(vel_try, self.v_max)
            if float(np.linalg.norm(vel_try)) < 1e-4:
                continue
            for scale in (1.0, 0.6, 0.3, 0.15):
                v_step = vel_try * scale
                pos_try = self.pos + v_step * self.dt
                if not self._seg_collision(self.pos, pos_try):
                    self.pos = pos_try
                    self.vel = v_step
                    self._sync_vf_vel()
                    return True
        self.vel = np.zeros(2, dtype=float)
        self._sync_vf_vel()
        return True

    def _fix_initial_collision(self):
        if self._in_collision(self.pos):
            ok, new_pos = find_nearest_free_world(
                self.pos, self.map_occ, self.meta,
                self.map_w, self.map_h, self.chassis_half, self.chassis_yaw,
                max_radius_m=2.0,
            )
            if ok:
                self.pos = new_pos
                self.target = self.pos.copy()

    def set_cmd_vel_timeout(self, timeout_s: float):
        self._cmd_vel_timeout_ns = int(timeout_s * 1e9)

    def on_cmd_vel(self, vx_bl: float, vy_bl: float, now_ns: int):
        with self.lock:
            self._cmd_vel_vx_bl = vx_bl
            self._cmd_vel_vy_bl = vy_bl
            self._cmd_vel_last_time_ns = now_ns

    def set_chassis_stop(self, stop: bool):
        with self.lock:
            self._chassis_stop = stop
            if stop:
                self.chassis_wz = 0.0

    def set_hp_enabled(self, enabled: bool):
        with self.lock:
            self._hp_disabled = not enabled

    def set_z_vel(self, val: float):
        with self.lock:
            self.z_vel_override = val

    def is_cmd_vel_active(self, now_ns: int) -> bool:
        if self._cmd_vel_last_time_ns is None:
            return False
        return (now_ns - self._cmd_vel_last_time_ns) < self._cmd_vel_timeout_ns

    def on_gimbal_control(
        self,
        mode: int,
        big_yaw_deg: float,
        yaw_lower_deg: float,
        yaw_upper_deg: float,
    ):
        with self.lock:
            if self.gimbal_mode != mode:
                self._yaw_scan_dir = 1.0
            self.gimbal_mode = mode
            self.gimbal_big_yaw_deg = big_yaw_deg
            self.gimbal_yaw_lower_deg = yaw_lower_deg
            self.gimbal_yaw_upper_deg = yaw_upper_deg

    @staticmethod
    def _normalize_angle_rad(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _reciprocate_scan_(
        self,
        current_rad: float,
        lower_deg: float,
        upper_deg: float,
        scan_dir_attr: str,
    ) -> float:
        lo = math.radians(lower_deg)
        hi = math.radians(upper_deg)
        if lo > hi:
            lo, hi = hi, lo
        scan_dir = getattr(self, scan_dir_attr)
        value = current_rad + scan_dir * self.scan_speed * self.dt
        if value >= hi:
            value = hi
            setattr(self, scan_dir_attr, -1.0)
        elif value <= lo:
            value = lo
            setattr(self, scan_dir_attr, 1.0)
        return value

    def _update_gimbal_from_control(self):

        mode = self.gimbal_mode
        if mode == 0:
            self.yaw += self.scan_speed * self.dt
            self.yaw = self._normalize_angle_rad(self.yaw)
        elif mode == 1:
            self.yaw = self._reciprocate_scan_(
                self.yaw,
                self.gimbal_yaw_lower_deg,
                self.gimbal_yaw_upper_deg,
                '_yaw_scan_dir',
            )
        elif mode == 2:
            pass
        elif mode == 3:
            self.yaw = math.radians(self.gimbal_big_yaw_deg)

    def _compute_chassis_wz(self, now_sec: float) -> float:
        if self._chassis_stop or self._hp_disabled:
            return 0.0
        base_vel = float(self.chassis_rotate_velocity)
        mode = int(self.chassis_mode)
        if mode == 1:
            yaw_err = self._normalize_angle_rad(self.yaw - self.chassis_yaw)
            rotate_speed = 3.0
            if abs(yaw_err) < rotate_speed * self.dt:
                return yaw_err / max(self.dt, 1e-6)
            return math.copysign(rotate_speed, yaw_err)
        if mode == 2:
            if base_vel == 0.0:
                base_vel = 3.0
            return base_vel * (1.0 + 0.8 * math.sin(now_sec * 2.0))
        return base_vel

    def step(self, now_ns: int):
        with self.lock:
            self._step_locked(now_ns)

    def _step_locked(self, now_ns: int):
        now_sec = now_ns * 1e-9

        self.chassis_wz = 0.0 if self._wall_contact else self._compute_chassis_wz(now_sec)

        if self._hp_disabled or self._chassis_stop:
            self._clear_motion()
            self.chassis_wz = 0.0
            self._update_gimbal_from_control()
            return

        self._separate_to_surface()

        use_cmd_vel = self.is_cmd_vel_active(now_ns)

        if use_cmd_vel:
            self.target = self.pos.copy()
            vel_new = self._plant_step_bl(self._cmd_vel_vx_bl, self._cmd_vel_vy_bl)
        else:
            to_goal = self.target - self.pos
            dist = float(np.linalg.norm(to_goal))
            if dist < 0.05:
                self._clear_motion()
                vel_new = self.vel.copy()
            else:
                dir_vec = to_goal / max(dist, 1e-9)
                v_stop = dist / max(self._vf.stopping_lag(), self.dt)
                speed = min(self.v_max, v_stop)
                v_map = dir_vec * speed
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                vx_bl = c * v_map[0] + s * v_map[1]
                vy_bl = -s * v_map[0] + c * v_map[1]
                vel_new = self._plant_step_bl(float(vx_bl), float(vy_bl))

        self._wall_contact = self._apply_velocity(vel_new)
        if not self._wall_contact:
            new_yaw = self._normalize_angle_rad(self.chassis_yaw + self.chassis_wz * self.dt)
            old_yaw = self.chassis_yaw
            self.chassis_yaw = new_yaw
            if self._in_collision(self.pos):
                self.chassis_yaw = old_yaw
                self.chassis_wz = 0.0
                self._wall_contact = True
        self._update_gimbal_from_control()

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray, float]:
        with self.lock:
            return self.pos.copy(), self.vel.copy(), float(self.yaw)

    def get_z_vel_override(self) -> float:
        with self.lock:
            return self.z_vel_override

    def set_target(self, target: np.ndarray):
        with self.lock:
            self.target = np.asarray(target, dtype=float).copy()

            self._cmd_vel_last_time_ns = None

    def set_dragging(self, flag: bool):
        with self.lock:
            self.dragging = flag

    def get_dragging(self) -> bool:
        with self.lock:
            return self.dragging

    def get_pos_for_drag_check(self) -> np.ndarray:
        with self.lock:
            return self.pos.copy()
