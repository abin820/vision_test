import math
import numpy as np
import pygame
import threading
from typing import List, Optional, Tuple

from sp_nav_sim.sim.sim_map import (
    SimMapMeta,
    world_to_pixel,
    pixel_to_world,
)
from sp_nav_sim.sim.sim_robot import SimRobot

_ROBOT_COLORS = [
    ((200, 50,  50),  (255, 100, 100), (0, 200,  50)),
    ((50,  80, 200),  (80, 150, 255),  (0, 200, 200)),
    ((50, 180,  50),  (100, 255, 100), (200, 200, 0)),
    ((180, 100,  0),  (255, 180,  50), (200, 100, 200)),
]

def _robot_color(idx: int):
    return _ROBOT_COLORS[idx % len(_ROBOT_COLORS)]

class SimGui:

    def __init__(
        self,
        robots: List[SimRobot],
        map_rgb: np.ndarray,
        meta: SimMapMeta,
        map_w: int,
        map_h: int,
        win_w: int = 1600,
        win_h: int = 1000,
        sim_hz: float = 30.0,
    ):
        self.robots = robots
        self.map_rgb = map_rgb
        self.meta = meta
        self.map_w = map_w
        self.map_h = map_h
        self.win_w = win_w
        self.win_h = win_h
        self.sim_hz = sim_hz
        self.map_res = getattr(self.meta, 'resolution', 0.05)

        pygame.init()
        self.screen = pygame.display.set_mode((win_w, win_h))
        pygame.display.set_caption("SP Nav Sim — multi-robot sim")
        self.clock = pygame.time.Clock()

        sx = win_w / float(map_w)
        sy = win_h / float(map_h)
        self.window_scale = max(0.05, min(sx, sy))

        self.map_surface = self._make_map_surface(map_rgb, self.window_scale)
        self.map_rect = self.map_surface.get_rect()
        screen_rect = self.screen.get_rect()
        self.map_offset = (
            (screen_rect.width  - self.map_rect.width)  // 2,
            (screen_rect.height - self.map_rect.height) // 2,
        )

        self._right_selected: Optional[int] = None
        self._right_dragging: bool = False
        self._sentry_motion_button_rect = pygame.Rect(12, 12, 200, 34)

        try:
            self._font = pygame.font.SysFont('monospace', 14)
        except Exception:
            self._font = None

    @staticmethod
    def _make_map_surface(rgb_img: np.ndarray, scale: float) -> pygame.Surface:
        surf = pygame.surfarray.make_surface(np.transpose(rgb_img, (1, 0, 2)))
        if abs(scale - 1.0) > 1e-6:
            new_size = (
                int(rgb_img.shape[1] * scale),
                int(rgb_img.shape[0] * scale),
            )
            surf = pygame.transform.smoothscale(surf, new_size)
        return surf

    def _world_to_screen(self, xy_m: np.ndarray) -> Tuple[int, int]:
        px = world_to_pixel(xy_m, self.meta, self.map_h) * self.window_scale
        return (
            int(px[0] + self.map_offset[0]),
            int(px[1] + self.map_offset[1]),
        )

    def _mouse_to_world(self, mx: int, my: int) -> Optional[np.ndarray]:
        mx_map = mx - self.map_offset[0]
        my_map = my - self.map_offset[1]
        if (mx_map < 0 or my_map < 0
                or mx_map >= self.map_rect.width
                or my_map >= self.map_rect.height):
            return None
        px = mx_map / self.window_scale
        py = my_map / self.window_scale
        return pixel_to_world(np.array([px, py], dtype=float), self.meta, self.map_h)

    def _pick_radius(self) -> float:
        return max(1.0, 20.0 / self.window_scale * self.meta.resolution)

    def _nearest_extra_robot(self, world_pos: np.ndarray) -> Optional[int]:
        if len(self.robots) <= 1:
            return None
        best_idx, best_dist = None, float('inf')
        for i, robot in enumerate(self.robots[1:], start=1):
            pos = robot.get_pos_for_drag_check()
            d = float(np.linalg.norm(world_pos - pos))
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _handle_sentry_motion_button_click(self, event) -> bool:
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return False
        if not self._sentry_motion_button_rect.collidepoint(event.pos):
            return False

        node = getattr(self, '_node_ref', None)
        if node is not None and hasattr(node, 'toggle_sentry_motion_allowed'):
            node.toggle_sentry_motion_allowed()
        return True

    def handle_events(self, sentry_motion_blocked: bool):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

            if self._handle_sentry_motion_button_click(event):
                continue

            if event.type == pygame.KEYDOWN and event.key == pygame.K_z:
                self.robots[0].set_z_vel(-1.3)
            elif event.type == pygame.KEYUP and event.key == pygame.K_z:
                self.robots[0].set_z_vel(0.0)

            if len(self.robots) > 0:
                sentry = self.robots[0]

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if not sentry_motion_blocked:
                        target = self._mouse_to_world(*pygame.mouse.get_pos())
                        if target is not None:
                            sentry.set_target(target)
                            pos = sentry.get_pos_for_drag_check()
                            if np.linalg.norm(target - pos) <= self._pick_radius():
                                sentry.set_dragging(True)

                if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    sentry.set_dragging(False)

                if event.type == pygame.MOUSEMOTION and sentry.get_dragging():
                    if not sentry_motion_blocked:
                        target = self._mouse_to_world(*pygame.mouse.get_pos())
                        if target is not None:
                            sentry.set_target(target)
                    else:
                        sentry.set_dragging(False)

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                target = self._mouse_to_world(*pygame.mouse.get_pos())
                if target is not None:
                    idx = self._nearest_extra_robot(target)
                    if idx is not None:
                        robot = self.robots[idx]
                        robot.set_target(target)
                        pos = robot.get_pos_for_drag_check()
                        self._right_selected = idx
                        if np.linalg.norm(target - pos) <= self._pick_radius():
                            self._right_dragging = True
                        else:
                            self._right_dragging = False

            if event.type == pygame.MOUSEBUTTONUP and event.button == 3:
                self._right_dragging = False

            if event.type == pygame.MOUSEMOTION and self._right_dragging:
                if self._right_selected is not None:
                    target = self._mouse_to_world(*pygame.mouse.get_pos())
                    if target is not None:
                        self.robots[self._right_selected].set_target(target)

    def _draw_sentry_motion_button(self):
        node = getattr(self, '_node_ref', None)
        allowed = (
            bool(node.is_sentry_motion_gui_allowed())
            if node and hasattr(node, 'is_sentry_motion_gui_allowed')
            else True
        )
        bg_color = (40, 150, 70) if allowed else (180, 55, 55)
        border_color = (230, 255, 230) if allowed else (255, 220, 220)
        text_color = (255, 255, 255)
        text = f"Sentry Move: {'ALLOW' if allowed else 'BLOCK'}"

        pygame.draw.rect(self.screen, bg_color, self._sentry_motion_button_rect, border_radius=6)
        pygame.draw.rect(self.screen, border_color, self._sentry_motion_button_rect, width=2, border_radius=6)
        if self._font:
            label = self._font.render(text, True, text_color)
            label_rect = label.get_rect(center=self._sentry_motion_button_rect.center)
            self.screen.blit(label, label_rect)

    def draw(self):
        self.screen.fill((30, 30, 30))
        self.screen.blit(self.map_surface, self.map_offset)

        for idx, robot in enumerate(self.robots):
            pos, vel, yaw = robot.snapshot()
            body_color, arrow_color, cross_color = _robot_color(idx)

            with robot.lock:
                target = robot.target.copy()
            tx, ty = self._world_to_screen(target)
            pygame.draw.line(self.screen, cross_color, (tx - 8, ty), (tx + 8, ty), 2)
            pygame.draw.line(self.screen, cross_color, (tx, ty - 8), (tx, ty + 8), 2)

            x, y = self._world_to_screen(pos)
            rad_px = max(2, int((robot.robot_radius / self.meta.resolution) * self.window_scale))

            chassis_yaw = getattr(robot, 'chassis_yaw', yaw)
            is_main = (robot == self.robots[0])
            self._get_rotated_rect(self.screen, (0, 255, 255) if is_main else body_color,
                                pos[0], pos[1], yaw, override_yaw=chassis_yaw)

            pygame.draw.circle(self.screen, body_color, (x, y), rad_px, 0)
            if idx > 0 and idx == self._right_selected:
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), rad_px + 4, 2)

            arrow_len = 30
            ex = int(x + math.cos(yaw) * arrow_len)
            ey = int(y - math.sin(yaw) * arrow_len)
            pygame.draw.line(self.screen, arrow_color, (x, y), (ex, ey), 2)

            spd = float(np.linalg.norm(vel))
            if spd > 1e-3:
                dv = vel / spd
                vex = int(x + dv[0] * 20)
                vey = int(y - dv[1] * 20)
                pygame.draw.line(self.screen, (220, 220, 80), (x, y), (vex, vey), 1)

            if self._font:
                label = self._font.render(f"R{robot.robot_id}", True, (240, 240, 240))
                self.screen.blit(label, (x + rad_px + 2, y - 8))

        self._draw_sentry_motion_button()

        pygame.display.flip()

    def tick(self, sentry_motion_blocked: bool, now_ns: int):
        self.handle_events(sentry_motion_blocked)
        for robot in self.robots:
            robot.step(now_ns)
        self.draw()
        self.clock.tick(int(self.sim_hz))

    def _get_rotated_rect(self, surf, color, x, y, yaw, override_yaw=None):
        side = 0.50
        dx2 = side / 2.0
        dy2 = side / 2.0

        yaw_use = yaw
        if override_yaw is not None:
             yaw_use = override_yaw

        p_local = np.array([
            [dx2, dy2],
            [-dx2, dy2],
            [-dx2, -dy2],
            [dx2, -dy2],
            [dx2, dy2],
        ])
        R = np.array([
            [ math.cos(yaw_use), -math.sin(yaw_use)],
            [ math.sin(yaw_use),  math.cos(yaw_use)],
        ])
        p_rot = p_local @ R

        pts_screen = []
        for point in p_rot:
            cx, cy = self._world_to_screen(np.array([x + point[0], y + point[1]]))
            pts_screen.append((cx, cy))

        pygame.draw.lines(surf, color, False, pts_screen, 2)
