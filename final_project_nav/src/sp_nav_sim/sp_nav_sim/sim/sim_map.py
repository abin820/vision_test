import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from PIL import Image

CHASSIS_HALF = 0.25

@dataclass
class SimMapMeta:
    resolution: float
    origin: np.ndarray
    negate: int
    occupied_thresh: float
    free_thresh: float

def clamp_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-9 or n <= max_norm:
        return v
    return v * (max_norm / n)

def read_text_with_fallback(path: str) -> str:
    for enc in ("utf-8", "utf-8-sig", "utf-16", "utf-16le", "utf-16be", "gbk"):
        try:
            with open(path, "r", encoding=enc) as f:
                return f.read()
        except UnicodeDecodeError:
            continue
    with open(path, "rb") as f:
        raw = f.read()
    return raw.decode("utf-8", errors="ignore")

def load_sim_map(yaml_path: str, treat_unknown_as_occupied: bool = True):
    import yaml as _yaml
    text = read_text_with_fallback(yaml_path)
    y = _yaml.safe_load(text)

    image_path = y['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    resolution = float(y['resolution'])
    origin = np.array(y.get('origin', [0.0, 0.0, 0.0]), dtype=float)
    negate = int(y.get('negate', 0))
    occupied_thresh = float(y.get('occupied_thresh', 0.65))
    free_thresh = float(y.get('free_thresh', 0.196))

    meta = SimMapMeta(
        resolution=resolution,
        origin=origin,
        negate=negate,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
    )

    img = Image.open(image_path).convert('L')
    raw = np.array(img, dtype=np.uint8)

    if negate == 0:
        p_occ = (255.0 - raw.astype(np.float32)) / 255.0
    else:
        p_occ = raw.astype(np.float32) / 255.0

    occ = p_occ >= occupied_thresh
    free = p_occ <= free_thresh
    unknown = ~(occ | free)

    if treat_unknown_as_occupied:
        occ_mask = (occ | unknown).astype(np.bool_)
    else:
        occ_mask = occ.astype(np.bool_)

    disp_gray = raw if negate == 0 else (255 - raw)
    disp_rgb = np.stack([disp_gray, disp_gray, disp_gray], axis=-1)

    return disp_rgb, occ_mask, meta, image_path

def world_to_pixel(xy_m: np.ndarray, meta: SimMapMeta, map_h: int) -> np.ndarray:
    x, y = float(xy_m[0]), float(xy_m[1])
    ox, oy, _ = meta.origin
    px = (x - ox) / meta.resolution
    py = (y - oy) / meta.resolution
    py_img = (map_h - 1) - py
    return np.array([px, py_img], dtype=float)

def pixel_to_world(px_py: np.ndarray, meta: SimMapMeta, map_h: int) -> np.ndarray:
    px, py_img = float(px_py[0]), float(px_py[1])
    ox, oy, _ = meta.origin
    py = (map_h - 1) - py_img
    x = ox + px * meta.resolution
    y = oy + py * meta.resolution
    return np.array([x, y], dtype=float)

def world_to_pixel_int(xy_m: np.ndarray, meta: SimMapMeta, map_h: int) -> Tuple[int, int]:
    p = world_to_pixel(xy_m, meta, map_h)
    return int(round(p[0])), int(round(p[1]))

def pixel_in_bounds(px: int, py: int, map_w: int, map_h: int) -> bool:
    return 0 <= px < map_w and 0 <= py < map_h

def _square_proj_radius(yaw: float, half: float, axis: np.ndarray) -> float:
    c, s = math.cos(yaw), math.sin(yaw)
    ax, ay = float(axis[0]), float(axis[1])
    return half * (abs(c * ax + s * ay) + abs(-s * ax + c * ay))

def oriented_square_mtv(
    c0: np.ndarray,
    yaw0: float,
    h0: float,
    c1: np.ndarray,
    yaw1: float,
    h1: float,
) -> Optional[np.ndarray]:
    """把方框 0 推出方框 1 的最短平移；不相交则返回 None。"""
    delta = np.asarray(c1, dtype=float) - np.asarray(c0, dtype=float)
    best_overlap = float('inf')
    best_axis = None
    for yaw in (yaw0, yaw1):
        c, s = math.cos(yaw), math.sin(yaw)
        for axis in (np.array([c, s]), np.array([-s, c])):
            dist = abs(float(np.dot(delta, axis)))
            overlap = _square_proj_radius(yaw0, h0, axis) + _square_proj_radius(yaw1, h1, axis) - dist
            if overlap <= 0.0:
                return None
            if overlap < best_overlap:
                best_overlap = overlap
                best_axis = -axis if float(np.dot(delta, axis)) > 0.0 else axis.copy()
    if best_axis is None:
        return None
    return best_axis * best_overlap

def in_collision_world(
    xy_m: np.ndarray,
    map_occ: np.ndarray,
    meta: SimMapMeta,
    map_w: int,
    map_h: int,
    half_size: float = CHASSIS_HALF,
    yaw: float = 0.0,
) -> bool:
    cx, cy = float(xy_m[0]), float(xy_m[1])
    c, s = math.cos(yaw), math.sin(yaw)
    aabb = half_size * (abs(c) + abs(s))
    corners = (
        (cx + aabb, cy + aabb),
        (cx + aabb, cy - aabb),
        (cx - aabb, cy + aabb),
        (cx - aabb, cy - aabb),
    )
    pxs, pys = [], []
    for x, y in corners:
        px, py = world_to_pixel_int(np.array([x, y], dtype=float), meta, map_h)
        pxs.append(px)
        pys.append(py)
    for py in range(min(pys), max(pys) + 1):
        for px in range(min(pxs), max(pxs) + 1):
            if not pixel_in_bounds(px, py, map_w, map_h):
                return True
            w = pixel_to_world(np.array([px + 0.5, py + 0.5], dtype=float), meta, map_h)
            dx, dy = float(w[0]) - cx, float(w[1]) - cy
            lx = c * dx + s * dy
            ly = -s * dx + c * dy
            if max(abs(lx), abs(ly)) <= half_size + 1e-9 and map_occ[py, px]:
                return True
    return False

def segment_in_collision(
    p0: np.ndarray,
    p1: np.ndarray,
    map_occ: np.ndarray,
    meta: SimMapMeta,
    map_w: int,
    map_h: int,
    half_size: float = CHASSIS_HALF,
    yaw: float = 0.0,
) -> bool:
    dist = float(np.linalg.norm(p1 - p0))
    if dist < 1e-9:
        return in_collision_world(p1, map_occ, meta, map_w, map_h, half_size, yaw)
    step = max(meta.resolution * 0.5, 0.02)
    n = int(math.ceil(dist / step))
    for i in range(1, n + 1):
        a = i / float(n)
        p = (1 - a) * p0 + a * p1
        if in_collision_world(p, map_occ, meta, map_w, map_h, half_size, yaw):
            return True
    return False

def find_nearest_free_world(
    start_xy: np.ndarray,
    map_occ: np.ndarray,
    meta: SimMapMeta,
    map_w: int,
    map_h: int,
    half_size: float = CHASSIS_HALF,
    yaw: float = 0.0,
    max_radius_m: float = 2.0,
) -> Tuple[bool, np.ndarray]:
    start_px = world_to_pixel(start_xy, meta, map_h)
    sx, sy = int(round(start_px[0])), int(round(start_px[1]))
    if not pixel_in_bounds(sx, sy, map_w, map_h):
        return False, start_xy
    max_r_px = int(max_radius_m / meta.resolution)
    for r in range(1, max_r_px + 1):
        for dx in range(-r, r + 1):
            for dy in (-r, r):
                x, y = sx + dx, sy + dy
                if pixel_in_bounds(x, y, map_w, map_h) and not map_occ[y, x]:
                    cand = pixel_to_world(np.array([x, y], dtype=float), meta, map_h)
                    if not in_collision_world(cand, map_occ, meta, map_w, map_h, half_size, yaw):
                        return True, cand
        for dy in range(-r + 1, r):
            for dx in (-r, r):
                x, y = sx + dx, sy + dy
                if pixel_in_bounds(x, y, map_w, map_h) and not map_occ[y, x]:
                    cand = pixel_to_world(np.array([x, y], dtype=float), meta, map_h)
                    if not in_collision_world(cand, map_occ, meta, map_w, map_h, half_size, yaw):
                        return True, cand
    return False, start_xy

def estimate_obstacle_normal(
    xy_m: np.ndarray,
    map_occ: np.ndarray,
    meta: SimMapMeta,
    map_w: int,
    map_h: int,
    half_size: float = CHASSIS_HALF,
) -> Optional[np.ndarray]:
    px_f = world_to_pixel(xy_m, meta, map_h)
    cx, cy = int(round(px_f[0])), int(round(px_f[1]))
    if not pixel_in_bounds(cx, cy, map_w, map_h):
        return None

    win = max(2, int(math.ceil(half_size / meta.resolution)) + 2)
    x0, x1 = max(0, cx - win), min(map_w - 1, cx + win)
    y0, y1 = max(0, cy - win), min(map_h - 1, cy + win)

    patch = map_occ[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
    if patch.size < 9:
        return None

    gy, gx = np.gradient(patch)

    yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
    dx = (xx - cx).astype(np.float32)
    dy = (yy - cy).astype(np.float32)
    r2 = dx * dx + dy * dy
    w = 1.0 / (1.0 + r2)

    gpx = float(np.sum(gx * w))
    gpy = float(np.sum(gy * w))

    g = np.array([gpx, gpy], dtype=float)
    ng = float(np.linalg.norm(g))
    if ng < 1e-6:
        return None

    g = -g / ng
    n_world = np.array([g[0], -g[1]], dtype=float)
    nn = float(np.linalg.norm(n_world))
    if nn < 1e-9:
        return None
    return n_world / nn
