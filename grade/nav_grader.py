#!/usr/bin/env python3
"""ROS 2 navigation grader: arrival accuracy, elapsed time and path tracking."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


Point = Tuple[float, float]


@dataclass
class Sample:
    elapsed: float
    x: float
    y: float
    goal_distance: float
    path_error: Optional[float] = None
    speed: Optional[float] = None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def segment_distance(px: float, py: float, a: Point, b: Point) -> float:
    """Return the Euclidean distance from P to the finite segment AB."""
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    if denom <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    u = clamp(((px - ax) * vx + (py - ay) * vy) / denom, 0.0, 1.0)
    return math.hypot(px - (ax + u * vx), py - (ay + u * vy))


def polyline_distance(px: float, py: float, path: Sequence[Point]) -> Optional[float]:
    if not path:
        return None
    if len(path) == 1:
        return math.hypot(px - path[0][0], py - path[0][1])
    return min(segment_distance(px, py, path[i], path[i + 1]) for i in range(len(path) - 1))


def lerp_score(value: float, full_at: float, zero_at: float, maximum: float) -> float:
    if value <= full_at:
        return maximum
    if value >= zero_at:
        return 0.0
    return maximum * (zero_at - value) / (zero_at - full_at)


def polyline_length(points: Sequence[Point]) -> float:
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def percentile(values: Sequence[float], ratio: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = clamp(ratio, 0.0, 1.0) * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


class NavigationGrader(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("navigation_grader")
        self.args = args
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, qos)
        self.path_sub = self.create_subscription(Path, args.path_topic, self._on_path, qos)
        self.odom_sub = self.create_subscription(
            Odometry, args.odom_topic, self._on_odometry, qos_profile_sensor_data
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.started = False
        self.finished = False
        self.arrived = False
        self.failure_reason: Optional[str] = None
        self.start_mono = 0.0
        self.finish_mono = 0.0
        self.entry_mono: Optional[float] = None
        self.samples: List[Sample] = []
        self.reference_path: List[Point] = []
        self.path_received_elapsed: Optional[float] = None
        self.last_pose: Optional[Point] = None
        self.latest_speed: Optional[float] = None
        self.latest_speed_mono = 0.0
        self.last_tf_warning = 0.0
        self.last_progress_mono = 0.0
        self.goal_subscriber_count = 0
        self.timer = self.create_timer(1.0 / args.sample_rate, self._sample)

    def _on_odometry(self, msg: Odometry) -> None:
        velocity = msg.twist.twist.linear
        self.latest_speed = math.hypot(velocity.x, velocity.y)
        self.latest_speed_mono = time.monotonic()

    def _on_path(self, msg: Path) -> None:
        # 首条有效路径即本次评测的固定参考路径，避免后续重规划改写历史基准。
        if not self.started or self.reference_path or not msg.poses:
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) >= 2:
            self.reference_path = points
            self.path_received_elapsed = time.monotonic() - self.start_mono
            # 路径通常比目标晚几十毫秒到达，补算这段时间内已经采到的位姿。
            for sample in self.samples:
                sample.path_error = polyline_distance(sample.x, sample.y, points)
            self.get_logger().info(f"收到全局参考路径：{len(points)} 个点")

    def lookup_pose(self) -> Optional[Point]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, Time()
            )
            return (float(tf.transform.translation.x), float(tf.transform.translation.y))
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warning >= 2.0:
                self.get_logger().warning(
                    f"等待 TF {self.args.map_frame} -> {self.args.base_frame}: {exc}"
                )
                self.last_tf_warning = now
            return None

    def ready(self) -> bool:
        self.goal_subscriber_count = self.goal_pub.get_subscription_count()
        velocity_ready = (
            self.latest_speed is not None
            and time.monotonic() - self.latest_speed_mono <= self.args.velocity_stale_timeout
        )
        return (
            self.goal_subscriber_count >= self.args.min_goal_subscribers
            and self.lookup_pose() is not None
            and velocity_ready
        )

    def publish_goal_once(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.map_frame
        msg.pose.position.x = self.args.goal_x
        msg.pose.position.y = self.args.goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.started = True
        self.start_mono = time.monotonic()
        self.last_progress_mono = self.start_mono
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"目标已发布一次：({self.args.goal_x:.3f}, {self.args.goal_y:.3f})；开始计时"
        )

    def _sample(self) -> None:
        if not self.started or self.finished:
            return
        now = time.monotonic()
        pose = self.lookup_pose()
        if pose is None:
            if now - self.start_mono >= self.args.timeout:
                self.finish_mono = now
                self.failure_reason = "robot_tf_lost"
                self.finished = True
            return

        x, y = pose
        self.last_pose = pose
        elapsed = now - self.start_mono
        goal_distance = math.hypot(x - self.args.goal_x, y - self.args.goal_y)
        error = polyline_distance(x, y, self.reference_path)
        speed_is_fresh = (
            self.latest_speed is not None
            and now - self.latest_speed_mono <= self.args.velocity_stale_timeout
        )
        speed = self.latest_speed if speed_is_fresh else None
        self.samples.append(Sample(elapsed, x, y, goal_distance, error, speed))

        if not self.reference_path and elapsed >= self.args.path_timeout:
            self.finish_mono = now
            self.failure_reason = "no_global_path"
            self.finished = True
            self.get_logger().error(
                f"目标发布后 {elapsed:.1f}s 仍未收到有效 {self.args.path_topic}；"
                "导航未确认处理目标，本次终止且不会重复发布目标。"
            )
            return

        if now - self.last_progress_mono >= self.args.progress_interval:
            error_text = "等待路径" if error is None else f"路径误差={error:.3f}m"
            speed_text = "速度=N/A" if speed is None else f"速度={speed:.3f}m/s"
            self.get_logger().info(
                f"进度 | t={elapsed:6.1f}s | 位姿=({x:6.2f}, {y:6.2f}) | "
                f"距目标={goal_distance:5.2f}m | {speed_text} | {error_text}"
            )
            self.last_progress_mono = now

        arrival_conditions_met = (
            goal_distance <= self.args.arrival_tolerance
            and speed is not None
            and speed <= self.args.arrival_speed_threshold
        )
        if arrival_conditions_met:
            if self.entry_mono is None:
                self.entry_mono = now
            if now - self.entry_mono >= self.args.hold_time:
                self.arrived = True
                # 防抖的停留时间不计入导航用时。
                self.finish_mono = self.entry_mono
                self.finished = True
                return
        else:
            self.entry_mono = None

        if elapsed >= self.args.timeout:
            self.finish_mono = now
            self.failure_reason = "navigation_timeout"
            self.finished = True


def color_for_error(error: Optional[float], zero_score_error: float) -> str:
    if error is None:
        return "#94a3b8"
    ratio = clamp(error / max(zero_score_error, 1.0e-9), 0.0, 1.0)
    red = int(30 + 220 * ratio)
    green = int(185 - 125 * ratio)
    blue = int(85 - 35 * ratio)
    return f"#{red:02x}{green:02x}{blue:02x}"


def make_svg(
    path: Sequence[Point], samples: Sequence[Sample], args: argparse.Namespace, result: dict
) -> str:
    width, height = 1100, 760
    left, right, top = 75.0, 35.0, 55.0
    plot_w, trajectory_h = width - left - right, 460.0
    error_top, error_h = 585.0, 120.0
    all_points = list(path) + [(s.x, s.y) for s in samples] + [(args.goal_x, args.goal_y)]
    if not all_points:
        all_points = [(0.0, 0.0), (1.0, 1.0)]
    xs, ys = [p[0] for p in all_points], [p[1] for p in all_points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    pad = max(0.5, 0.05 * max(x_max - x_min, y_max - y_min, 1.0))
    x_min, x_max = x_min - pad, x_max + pad
    y_min, y_max = y_min - pad, y_max + pad
    scale = min(plot_w / (x_max - x_min), trajectory_h / (y_max - y_min))

    def sx(x: float) -> float:
        return left + (x - x_min) * scale

    def sy(y: float) -> float:
        return top + trajectory_h - (y - y_min) * scale

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<style>text{font-family:Arial,"Noto Sans CJK SC",sans-serif;fill:#172033}.small{font-size:13px}.label{font-size:15px}.title{font-size:23px;font-weight:700}.grid{stroke:#d9e1ea;stroke-width:1}.axis{stroke:#64748b;stroke-width:1.5}</style>',
        '<text x="75" y="32" class="title">导航评分：参考路径、实际轨迹与跟踪误差</text>',
        f'<text x="{width - 35}" y="30" text-anchor="end" class="label">总分 {result["score_total"]:.2f} / 70</text>',
    ]

    for i in range(6):
        gx = left + plot_w * i / 5
        gy = top + trajectory_h * i / 5
        xv = x_min + (x_max - x_min) * i / 5
        yv = y_max - (y_max - y_min) * i / 5
        parts += [
            f'<line x1="{gx:.1f}" y1="{top}" x2="{gx:.1f}" y2="{top + trajectory_h}" class="grid"/>',
            f'<text x="{gx:.1f}" y="{top + trajectory_h + 21}" text-anchor="middle" class="small">{xv:.1f}</text>',
            f'<line x1="{left}" y1="{gy:.1f}" x2="{left + plot_w}" y2="{gy:.1f}" class="grid"/>',
            f'<text x="{left - 10}" y="{gy + 5:.1f}" text-anchor="end" class="small">{yv:.1f}</text>',
        ]

    if path:
        path_pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in path)
        parts.append(
            f'<polyline points="{path_pts}" fill="none" stroke="#2563eb" stroke-width="4" opacity="0.8"/>'
        )
    for a, b in zip(samples, samples[1:]):
        color = color_for_error(b.path_error, args.track_zero_error)
        parts.append(
            f'<line x1="{sx(a.x):.1f}" y1="{sy(a.y):.1f}" x2="{sx(b.x):.1f}" y2="{sy(b.y):.1f}" stroke="{color}" stroke-width="3"/>'
        )
    if samples:
        parts.append(
            f'<circle cx="{sx(samples[0].x):.1f}" cy="{sy(samples[0].y):.1f}" r="6" fill="#10b981"/>'
        )
    parts += [
        f'<circle cx="{sx(args.goal_x):.1f}" cy="{sy(args.goal_y):.1f}" r="8" fill="#dc2626"/>',
        f'<circle cx="{sx(args.goal_x):.1f}" cy="{sy(args.goal_y):.1f}" r="{args.arrival_tolerance * scale:.1f}" fill="none" stroke="#dc2626" stroke-width="2" stroke-dasharray="6 5"/>',
        '<line x1="770" y1="66" x2="810" y2="66" stroke="#2563eb" stroke-width="4"/><text x="818" y="71" class="small">全局参考路径</text>',
        '<line x1="770" y1="88" x2="810" y2="88" stroke="#20b955" stroke-width="3"/><text x="818" y="93" class="small">实际轨迹（绿好，红差）</text>',
        '<circle cx="790" cy="110" r="6" fill="#dc2626"/><text x="818" y="115" class="small">目标点 / 到达容差圈</text>',
        f'<text x="{left + plot_w / 2:.1f}" y="{top + trajectory_h + 43}" text-anchor="middle" class="label">map x (m)</text>',
        f'<text x="22" y="{top + trajectory_h / 2:.1f}" text-anchor="middle" transform="rotate(-90 22 {top + trajectory_h / 2:.1f})" class="label">map y (m)</text>',
    ]

    max_t = max((s.elapsed for s in samples), default=1.0)
    max_e = max(
        [args.track_zero_error * 1.15]
        + [s.path_error for s in samples if s.path_error is not None]
    )
    full_y = error_top + error_h * (1 - args.track_full_error / max_e)
    zero_y = error_top + error_h * (1 - args.track_zero_error / max_e)
    rmse_label = result.get("tracking_rmse_m")
    rmse_label = "N/A" if rmse_label is None else f"{rmse_label:.3f}"
    parts += [
        f'<rect x="{left}" y="{full_y:.1f}" width="{plot_w}" height="{error_top + error_h - full_y:.1f}" fill="#dcfce7" opacity="0.55"/>',
        f'<line x1="{left}" y1="{error_top + error_h}" x2="{left + plot_w}" y2="{error_top + error_h}" class="axis"/>',
        f'<line x1="{left}" y1="{error_top}" x2="{left}" y2="{error_top + error_h}" class="axis"/>',
        f'<line x1="{left}" y1="{full_y:.1f}" x2="{left + plot_w}" y2="{full_y:.1f}" stroke="#16a34a" stroke-dasharray="5 5"/>',
        f'<line x1="{left}" y1="{zero_y:.1f}" x2="{left + plot_w}" y2="{zero_y:.1f}" stroke="#dc2626" stroke-dasharray="5 5"/>',
        f'<text x="{left + 8}" y="{error_top + 17}" class="small">跟踪误差 (m)，RMSE={rmse_label}</text>',
        f'<text x="{left + plot_w - 5}" y="{full_y - 4:.1f}" text-anchor="end" class="small">满分线 {args.track_full_error:.2f}m</text>',
        f'<text x="{left + plot_w - 5}" y="{zero_y - 4:.1f}" text-anchor="end" class="small">零分线 {args.track_zero_error:.2f}m</text>',
        f'<text x="{left - 10}" y="{error_top + 5}" text-anchor="end" class="small">{max_e:.2f}</text>',
        f'<text x="{left - 10}" y="{error_top + error_h + 5}" text-anchor="end" class="small">0</text>',
        f'<text x="{left + plot_w}" y="{error_top + error_h + 21}" text-anchor="end" class="small">{max_t:.1f} s</text>',
    ]
    valid_samples = [s for s in samples if s.path_error is not None]
    if valid_samples:
        err_pts = " ".join(
            f"{left + plot_w * s.elapsed / max_t:.1f},{error_top + error_h * (1 - min(s.path_error or 0.0, max_e) / max_e):.1f}"
            for s in valid_samples
        )
        parts.append(
            f'<polyline points="{err_pts}" fill="none" stroke="#f97316" stroke-width="2"/>'
        )
    parts.append("</svg>")
    return "\n".join(parts)


def calculate_result(grader: NavigationGrader, args: argparse.Namespace) -> dict:
    final = grader.last_pose
    final_distance = (
        math.hypot(final[0] - args.goal_x, final[1] - args.goal_y)
        if final is not None
        else None
    )
    elapsed = max(0.0, grader.finish_mono - grader.start_mono)
    errors = [s.path_error for s in grader.samples if s.path_error is not None]
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None
    mean_error = sum(errors) / len(errors) if errors else None
    max_error = max(errors) if errors else None
    p95_error = percentile(errors, 0.95)
    within_full = (
        100.0 * sum(error <= args.track_full_error for error in errors) / len(errors)
        if errors
        else None
    )
    reference_length = polyline_length(grader.reference_path)
    actual_points = [(sample.x, sample.y) for sample in grader.samples]
    actual_length = polyline_length(actual_points)
    average_speed = actual_length / elapsed if elapsed > 1.0e-9 else 0.0
    length_ratio = (
        actual_length / reference_length if reference_length > 1.0e-9 else None
    )

    accuracy_score = (
        lerp_score(
            final_distance,
            args.accuracy_full_distance,
            args.accuracy_zero_distance,
            25.0,
        )
        if final_distance is not None
        else 0.0
    )
    time_score = (
        lerp_score(elapsed, args.time_full, args.time_zero, 25.0)
        if grader.arrived
        else 0.0
    )
    tracking_score = (
        lerp_score(rmse, args.track_full_error, args.track_zero_error, 20.0)
        if rmse is not None
        else 0.0
    )
    status = "arrived" if grader.arrived else (grader.failure_reason or "interrupted")
    return {
        "status": status,
        "failure_reason": grader.failure_reason,
        "goal": {"x": args.goal_x, "y": args.goal_y, "frame": args.map_frame},
        "goal_publish_count": 1,
        "goal_subscriber_count": grader.goal_subscriber_count,
        "planning_latency_s": grader.path_received_elapsed,
        "final_pose": ({"x": final[0], "y": final[1]} if final is not None else None),
        "final_distance_m": final_distance,
        "final_speed_mps": (grader.samples[-1].speed if grader.samples else None),
        "elapsed_s": elapsed,
        "tracking_rmse_m": rmse,
        "tracking_mean_m": mean_error,
        "tracking_p95_m": p95_error,
        "tracking_max_m": max_error,
        "tracking_within_full_threshold_percent": within_full,
        "reference_path_length_m": reference_length,
        "actual_trajectory_length_m": actual_length,
        "trajectory_to_reference_length_ratio": length_ratio,
        "average_speed_mps": average_speed,
        "sample_count": len(grader.samples),
        "tracking_sample_count": len(errors),
        "reference_path_points": len(grader.reference_path),
        "score_accuracy": round(accuracy_score, 2),
        "score_time": round(time_score, 2),
        "score_tracking": round(tracking_score, 2),
        "score_total": round(accuracy_score + time_score + tracking_score, 2),
        "maximum_score": 70,
        "thresholds": {
            "arrival_tolerance_m": args.arrival_tolerance,
            "arrival_speed_threshold_mps": args.arrival_speed_threshold,
            "arrival_hold_s": args.hold_time,
            "accuracy_full_at_m": args.accuracy_full_distance,
            "accuracy_zero_at_m": args.accuracy_zero_distance,
            "time_full_at_s": args.time_full,
            "time_zero_at_s": args.time_zero,
            "tracking_full_at_rmse_m": args.track_full_error,
            "tracking_zero_at_rmse_m": args.track_zero_error,
        },
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    }


def save_outputs(grader: NavigationGrader, args: argparse.Namespace, result: dict) -> None:
    os.makedirs(args.output_dir, exist_ok=True)
    json_path = os.path.join(args.output_dir, "result.json")
    csv_path = os.path.join(args.output_dir, "trajectory.csv")
    svg_path = os.path.join(args.output_dir, "tracking_error.svg")
    report_path = os.path.join(args.output_dir, "report.txt")
    with open(json_path, "w", encoding="utf-8") as fp:
        json.dump(result, fp, ensure_ascii=False, indent=2)
        fp.write("\n")
    with open(csv_path, "w", encoding="utf-8", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            ["elapsed_s", "x_m", "y_m", "goal_distance_m", "speed_mps", "path_error_m"]
        )
        for s in grader.samples:
            writer.writerow(
                [
                    f"{s.elapsed:.6f}",
                    f"{s.x:.6f}",
                    f"{s.y:.6f}",
                    f"{s.goal_distance:.6f}",
                    "" if s.speed is None else f"{s.speed:.6f}",
                    "" if s.path_error is None else f"{s.path_error:.6f}",
                ]
            )
    with open(svg_path, "w", encoding="utf-8") as fp:
        fp.write(make_svg(grader.reference_path, grader.samples, args, result))
        fp.write("\n")
    with open(report_path, "w", encoding="utf-8") as fp:
        fp.write(make_text_report(result, args))


def metric(value: Optional[float], unit: str = "", digits: int = 3) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}{unit}"


def make_text_report(result: dict, args: argparse.Namespace) -> str:
    status_names = {
        "arrived": "到达",
        "no_global_path": "未收到全局路径确认",
        "navigation_timeout": "导航超时",
        "robot_tf_lost": "机器人 TF 丢失",
        "interrupted": "人工中止",
    }
    lines = [
        "================ 导航评分报告 ================",
        f"状态：{status_names.get(result['status'], result['status'])}",
        f"目标：({args.goal_x:.3f}, {args.goal_y:.3f})  frame={args.map_frame}",
        f"目标发布：1 次；发布时已发现 {result['goal_subscriber_count']} 个订阅者",
        f"规划确认耗时：{metric(result['planning_latency_s'], ' s')}",
        "",
        "评分标准（区间内线性扣分）：",
        f"  准确到达 25分：≤ {args.accuracy_full_distance:.2f}m 满分；≥ {args.accuracy_zero_distance:.2f}m 零分",
        f"  时间     25分：≤ {args.time_full:.1f}s 满分；≥ {args.time_zero:.1f}s 零分；未到达为零分",
        f"  跟踪精度 20分：RMSE ≤ {args.track_full_error:.2f}m 满分；≥ {args.track_zero_error:.2f}m 零分",
        "",
        "分项结果：",
        f"  准确到达：{result['score_accuracy']:6.2f} / 25   最终距离={metric(result['final_distance_m'], ' m')}",
        f"  时间：    {result['score_time']:6.2f} / 25   用时={metric(result['elapsed_s'], ' s', 2)}",
        f"  跟踪精度：{result['score_tracking']:6.2f} / 20   RMSE={metric(result['tracking_rmse_m'], ' m')}",
        f"  总分：    {result['score_total']:6.2f} / 70",
        "",
        "辅助判分数据：",
        f"  判定到达时速度：{metric(result['final_speed_mps'], ' m/s')}",
        f"  跟踪误差 mean / P95 / max：{metric(result['tracking_mean_m'], ' m')} / {metric(result['tracking_p95_m'], ' m')} / {metric(result['tracking_max_m'], ' m')}",
        f"  满分误差线内采样占比：{metric(result['tracking_within_full_threshold_percent'], '%', 1)}",
        f"  参考路径 / 实际轨迹长度：{metric(result['reference_path_length_m'], ' m')} / {metric(result['actual_trajectory_length_m'], ' m')}",
        f"  实际/参考长度比：{metric(result['trajectory_to_reference_length_ratio'], '', 3)}",
        f"  平均速度：{metric(result['average_speed_mps'], ' m/s')}",
        f"  有效跟踪采样：{result['tracking_sample_count']} / {result['sample_count']}",
        "================================================",
        "",
    ]
    return "\n".join(lines)


def print_scoring_standard(args: argparse.Namespace) -> None:
    print("\n========== 本次评分标准（总计 70 分）==========")
    print(
        f"1. 准确到达 25分 | 最终距离 ≤ {args.accuracy_full_distance:.2f}m：25分；"
        f"{args.accuracy_full_distance:.2f}–{args.accuracy_zero_distance:.2f}m：线性扣分；"
        f"≥ {args.accuracy_zero_distance:.2f}m：0分"
    )
    print(
        f"2. 时间     25分 | 用时 ≤ {args.time_full:.1f}s：25分；"
        f"{args.time_full:.1f}–{args.time_zero:.1f}s：线性扣分；"
        f"≥ {args.time_zero:.1f}s 或未到达：0分"
    )
    print(
        f"3. 跟踪精度 20分 | RMSE ≤ {args.track_full_error:.2f}m：20分；"
        f"{args.track_full_error:.2f}–{args.track_zero_error:.2f}m：线性扣分；"
        f"≥ {args.track_zero_error:.2f}m：0分"
    )
    print(
        f"到达判定：距目标 ≤ {args.arrival_tolerance:.2f}m 且平移速度 ≤ "
        f"{args.arrival_speed_threshold:.2f}m/s，并连续保持 {args.hold_time:.2f}s；"
        "计时取首次同时满足距离和速度条件的时刻。"
    )
    print("跟踪误差：采样位姿到首条有效全局参考路径折线的最短距离。")
    print("计时起点：调用 /goal_pose 一次发布的瞬间，规划耗时包含在内。")
    print("================================================\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="导航作业 70 分自动评分器")
    parser.add_argument("--goal-x", type=float, default=14.1)
    parser.add_argument("--goal-y", type=float, default=14.1)
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--path-topic", default="/global_path")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=150.0)
    parser.add_argument("--arrival-tolerance", type=float, default=0.20)
    parser.add_argument("--arrival-speed-threshold", type=float, default=0.05)
    parser.add_argument("--velocity-stale-timeout", type=float, default=1.0)
    parser.add_argument("--hold-time", type=float, default=0.50)
    parser.add_argument("--ready-timeout", type=float, default=15.0)
    parser.add_argument("--path-timeout", type=float, default=10.0)
    parser.add_argument("--progress-interval", type=float, default=2.0)
    parser.add_argument("--min-goal-subscribers", type=int, default=2)
    parser.add_argument("--accuracy-full-distance", type=float, default=0.20)
    parser.add_argument("--accuracy-zero-distance", type=float, default=1.00)
    parser.add_argument("--time-full", type=float, default=45.0)
    parser.add_argument("--time-zero", type=float, default=120.0)
    parser.add_argument("--track-full-error", type=float, default=0.10)
    parser.add_argument("--track-zero-error", type=float, default=0.50)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    if (
        args.sample_rate <= 0
        or args.timeout <= 0
        or args.path_timeout <= 0
        or args.progress_interval <= 0
        or args.arrival_speed_threshold < 0
        or args.velocity_stale_timeout <= 0
        or args.hold_time < 0
    ):
        parser.error("采样率、超时和进度间隔必须为正数，速度阈值和 hold-time 不能为负数")
    if args.min_goal_subscribers < 1:
        parser.error("min-goal-subscribers 至少为 1")
    if args.accuracy_zero_distance <= args.accuracy_full_distance:
        parser.error("accuracy-zero-distance 必须大于 accuracy-full-distance")
    if args.time_zero <= args.time_full or args.track_zero_error <= args.track_full_error:
        parser.error("零分阈值必须大于满分阈值")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    grader = NavigationGrader(args)
    interrupted = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        print_scoring_standard(args)
        print(
            f"[评分器] 等待至少 {args.min_goal_subscribers} 个 {args.goal_topic} 订阅者，"
            f"并检查 {args.map_frame}->{args.base_frame} TF 与 {args.odom_topic} 速度……"
        )
        deadline = time.monotonic() + args.ready_timeout
        while rclpy.ok() and not interrupted and time.monotonic() < deadline:
            rclpy.spin_once(grader, timeout_sec=0.1)
            if grader.ready():
                break
        else:
            print(
                f"[评分器] 错误：等待目标订阅者、机器人 TF 或里程计速度超时；"
                f"当前发现 {grader.goal_subscriber_count} 个订阅者，目标没有发送。",
                file=sys.stderr,
            )
            return 3

        initial_pose = grader.lookup_pose()
        pose_text = (
            "暂时不可用"
            if initial_pose is None
            else f"({initial_pose[0]:.3f}, {initial_pose[1]:.3f})"
        )
        grader.get_logger().info(
            f"发送条件已满足：发现 {grader.goal_subscriber_count} 个目标订阅者；"
            f"初始位姿={pose_text}；初始速度={grader.latest_speed:.3f}m/s"
        )
        grader.publish_goal_once()
        while rclpy.ok() and not interrupted and not grader.finished:
            rclpy.spin_once(grader, timeout_sec=0.1)

        if interrupted:
            grader.finish_mono = time.monotonic()
            grader.finished = True
        result = calculate_result(grader, args)
        save_outputs(grader, args, result)

        print("\n" + make_text_report(result, args), end="")
        print(f'结果目录   : {os.path.abspath(args.output_dir)}')
        print("  - result.json")
        print("  - trajectory.csv")
        print("  - report.txt")
        print("  - tracking_error.svg（轨迹颜色 + 跟踪误差曲线 + 满分/零分阈值线）")
        return 0 if grader.arrived else 1
    finally:
        grader.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
