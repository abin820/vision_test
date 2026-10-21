import math
import os
import threading
from typing import List

import numpy as np
import pygame

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from robot_msg.msg import (
    ChassisModeMsg,
    GimbalControlMsg,
    RobotKinematics,
    RobotKinematicsArray,
)
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from sp_nav_sim.sim.sim_map import (
    load_sim_map,
    pixel_to_world,
)
from sp_nav_sim.sim.sim_robot import SimRobot
from sp_nav_sim.sim.sim_gui import SimGui


def _resolve_package_uri(raw: str) -> str:
    if not raw or raw.startswith('/'):
        return raw
    rest = raw
    prefix = 'package://'
    if raw.startswith(prefix):
        rest = raw[len(prefix):]
    slash = rest.find('/')
    if slash < 0:
        return raw
    pkg = rest[:slash]
    rel = rest[slash + 1:]
    return os.path.join(get_package_share_directory(pkg), rel)


def _param_missing_msg(node_name: str, param: str) -> str:
    return (
        f"[参数缺失] 节点 '{node_name}' 缺少参数 '{param}'，"
        "请在 sim_robot.yaml 的 ros__parameters 中填写。"
    )


class SimRobotNode(Node):
    def _require_value(self, name: str):
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException as exc:
            raise RuntimeError(_param_missing_msg(self.get_name(), name)) from exc

    def __init__(self):
        super().__init__('sim_robot_node')

        self.declare_parameter('map_yaml', Parameter.Type.STRING)
        self.declare_parameter('topic', Parameter.Type.STRING)
        self.declare_parameter('frame_id', Parameter.Type.STRING)

        self.declare_parameter('publish_hz', Parameter.Type.DOUBLE)
        self.declare_parameter('pub_hz', Parameter.Type.DOUBLE)
        self.declare_parameter('odom_hz', Parameter.Type.DOUBLE)

        self.declare_parameter('v_max', Parameter.Type.DOUBLE)
        self.declare_parameter('a_max', Parameter.Type.DOUBLE)
        self.declare_parameter('robot_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('margin', Parameter.Type.DOUBLE)
        self.declare_parameter('d_safe', Parameter.Type.DOUBLE)
        self.declare_parameter('w', Parameter.Type.DOUBLE)

        self.declare_parameter('window_width', Parameter.Type.INTEGER)
        self.declare_parameter('window_height', Parameter.Type.INTEGER)

        self.declare_parameter('treat_unknown_as_occupied', Parameter.Type.BOOL)

        self.declare_parameter('pos_noise_std', Parameter.Type.DOUBLE)
        self.declare_parameter('seed', Parameter.Type.INTEGER)

        self.declare_parameter('robot_id', Parameter.Type.INTEGER)
        self.declare_parameter('init_x', Parameter.Type.DOUBLE)
        self.declare_parameter('init_y', Parameter.Type.DOUBLE)
        self.declare_parameter('tf_child_frame_id', Parameter.Type.STRING)
        self.declare_parameter('cmd_vel_topic', Parameter.Type.STRING)
        self.declare_parameter('cmd_vel_timeout', Parameter.Type.DOUBLE)
        self.declare_parameter('odom_topic', Parameter.Type.STRING)
        self.declare_parameter('odom_frame_id', Parameter.Type.STRING)
        self.declare_parameter('local_costmap_topic', Parameter.Type.STRING)
        self.declare_parameter('local_costmap_width_m', Parameter.Type.DOUBLE)
        self.declare_parameter('local_costmap_height_m', Parameter.Type.DOUBLE)
        self.declare_parameter('gimbal_topic', Parameter.Type.STRING)
        self.declare_parameter('scan_speed', Parameter.Type.DOUBLE)
        self.declare_parameter('chassis_topic', Parameter.Type.STRING)

        self.declare_parameter('robot_ids', Parameter.Type.STRING)
        self.declare_parameter('init_xs', Parameter.Type.STRING)
        self.declare_parameter('init_ys', Parameter.Type.STRING)
        self.declare_parameter('tf_child_frame_ids', Parameter.Type.STRING)

        map_yaml = _resolve_package_uri(str(self._require_value('map_yaml')))
        if not map_yaml:
            raise RuntimeError(_param_missing_msg(self.get_name(), 'map_yaml'))

        self.topic    = self._require_value('topic')
        self.frame_id = self._require_value('frame_id')

        self.sim_hz  = float(self._require_value('publish_hz'))
        self.pub_hz  = float(self._require_value('pub_hz'))
        self.odom_hz = float(self._require_value('odom_hz'))

        v_max        = float(self._require_value('v_max'))
        a_max        = float(self._require_value('a_max'))
        robot_radius = float(self._require_value('robot_radius'))
        self.margin = float(self._require_value('margin'))
        self.d_safe = float(self._require_value('d_safe'))
        self.w_pen = float(self._require_value('w'))

        win_w  = int(self._require_value('window_width'))
        win_h  = int(self._require_value('window_height'))

        treat_unknown = bool(self._require_value('treat_unknown_as_occupied'))

        self.pos_noise_std = float(self._require_value('pos_noise_std'))
        seed = int(self._require_value('seed'))
        if seed == 0:
            seed = int(self.get_clock().now().nanoseconds % (2 ** 31 - 1))
        self.rng = np.random.default_rng(seed)

        sentry_id       = int(self._require_value('robot_id'))
        sentry_x        = float(self._require_value('init_x'))
        sentry_y        = float(self._require_value('init_y'))
        sentry_tf       = str(self._require_value('tf_child_frame_id'))
        cmd_vel_topic   = str(self._require_value('cmd_vel_topic'))
        cmd_vel_timeout = float(self._require_value('cmd_vel_timeout'))
        self.odom_topic = str(self._require_value('odom_topic'))
        self.odom_frame = str(self._require_value('odom_frame_id'))
        self.local_costmap_topic = str(self._require_value('local_costmap_topic'))
        self.local_costmap_width_m = float(self._require_value('local_costmap_width_m'))
        self.local_costmap_height_m = float(self._require_value('local_costmap_height_m'))
        gimbal_topic    = str(self._require_value('gimbal_topic'))
        scan_speed      = float(self._require_value('scan_speed'))
        chassis_topic   = str(self._require_value('chassis_topic'))

        extra_ids  = [int(x)   for x in str(self._require_value('robot_ids')).split(',')          if x.strip()]
        extra_xs   = [float(x) for x in str(self._require_value('init_xs')).split(',')            if x.strip()]
        extra_ys   = [float(x) for x in str(self._require_value('init_ys')).split(',')            if x.strip()]
        extra_tfs  = [x.strip() for x in str(self._require_value('tf_child_frame_ids')).split(',') if x.strip()]

        map_rgb, map_occ, meta, map_path = load_sim_map(map_yaml, treat_unknown)
        map_h, map_w = map_rgb.shape[:2]
        self.map_w = map_w
        self.map_h = map_h
        self.map_occ = map_occ
        self.map_meta = meta
        self.map_res = float(meta.resolution)
        self.robot_radius = robot_radius
        self.local_costmap_w = max(1, int(round(self.local_costmap_width_m / self.map_res)))
        self.local_costmap_h = max(1, int(round(self.local_costmap_height_m / self.map_res)))

        default_pos = pixel_to_world(
            np.array([map_w / 2.0, map_h / 2.0]), meta, map_h
        )

        common_kwargs = dict(
            v_max=v_max, a_max=a_max, robot_radius=robot_radius,
            map_occ=map_occ, meta=meta, map_w=map_w, map_h=map_h,
            sim_hz=self.sim_hz, scan_speed=scan_speed,
        )

        sentry_pos = (
            np.array([sentry_x, sentry_y], dtype=float)
            if not (math.isnan(sentry_x) or math.isnan(sentry_y))
            else default_pos.copy()
        )
        self.sentry = SimRobot(
            robot_id=sentry_id,
            init_pos=sentry_pos,
            tf_child_frame_id=sentry_tf,
            **common_kwargs,
        )
        self.sentry.set_cmd_vel_timeout(cmd_vel_timeout)
        self._sentry_motion_gui_allowed = True
        sentry_pos0, _, _ = self.sentry.snapshot()
        self.odom_origin_ = np.array(sentry_pos0, dtype=float)

        self.extra_robots: List[SimRobot] = []
        for i, rid in enumerate(extra_ids):
            rx   = extra_xs[i]  if i < len(extra_xs)  else default_pos[0] + 2.0 * (i + 1)
            ry   = extra_ys[i]  if i < len(extra_ys)  else default_pos[1]
            rtf  = extra_tfs[i] if i < len(extra_tfs) else f'base_link_{i + 2}'
            pos_i = (
                np.array([rx, ry], dtype=float)
                if not (math.isnan(rx) or math.isnan(ry))
                else default_pos.copy() + np.array([2.0 * (i + 1), 0.0])
            )
            robot = SimRobot(
                robot_id=rid, init_pos=pos_i,
                tf_child_frame_id=rtf,
                **common_kwargs,
            )
            self.extra_robots.append(robot)

        self.all_robots: List[SimRobot] = [self.sentry] + self.extra_robots
        for robot in self.all_robots:
            robot.peers = [other for other in self.all_robots if other is not robot]

        self.gui = SimGui(
            robots=self.all_robots,
            map_rgb=map_rgb,
            meta=meta,
            map_w=map_w,
            map_h=map_h,
            win_w=win_w,
            win_h=win_h,
            sim_hz=self.sim_hz,
        )
        self.gui._node_ref = self

        self.tf_broadcaster        = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_map_to_lidar_odom()

        self.robots_pub = self.create_publisher(RobotKinematicsArray, self.topic, 10)
        self.odom_pub   = self.create_publisher(Odometry, self.odom_topic, 10)
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, self.local_costmap_topic, 10)

        self.local_costmap_enabled = True
        self.create_service(SetBool, '/set_local_costmap_enable', self._on_set_local_costmap_enable)

        pub_dt  = 1.0 / max(self.pub_hz,  0.1)
        odom_dt = 1.0 / max(self.odom_hz, 1.0)
        self.create_timer(pub_dt,  self._on_pub_timer)
        self.create_timer(odom_dt, self._on_odom_timer)

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(GimbalControlMsg, gimbal_topic, self._on_gimbal, 10)
        self.create_subscription(ChassisModeMsg, chassis_topic, self._on_chassis_mode, 10)

        self.get_logger().info(
            f"SimRobotNode ready | map={map_path} | "
            f"robots={[r.robot_id for r in self.all_robots]} | "
            f"sim={self.sim_hz}Hz pub={self.pub_hz}Hz odom={self.odom_hz}Hz"
        )
        self.get_logger().info(
            f"map->{self.odom_frame} origin=({self.odom_origin_[0]:.3f}, "
            f"{self.odom_origin_[1]:.3f}) | start: base_link ≡ {self.odom_frame}"
        )
        self.get_logger().info(
            f"gimbal_topic={gimbal_topic} scan_speed={scan_speed} rad/s | "
            f"chassis_topic={chassis_topic}"
        )
        self.get_logger().info(
            f"local_costmap_topic={self.local_costmap_topic} frame={self.frame_id} "
            f"window={self.local_costmap_width_m:.2f}x{self.local_costmap_height_m:.2f}m"
        )

    def _publish_static_map_to_lidar_odom(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id  = self.odom_frame
        t.transform.translation.x = float(self.odom_origin_[0])
        t.transform.translation.y = float(self.odom_origin_[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _on_cmd_vel(self, msg: Twist):
        now_ns = self.get_clock().now().nanoseconds
        self.sentry.on_cmd_vel(float(msg.linear.x), float(msg.linear.y), now_ns)

    def _on_gimbal(self, msg: GimbalControlMsg):
        self.sentry.on_gimbal_control(
            int(msg.mode),
            float(msg.big_yaw),
            float(msg.yaw_lower_limit),
            float(msg.yaw_upper_limit),
        )

    def _on_chassis_mode(self, msg: ChassisModeMsg):
        self.sentry.set_chassis_stop(bool(msg.is_stop))
        self.sentry.chassis_mode = getattr(msg, 'mode', 0)
        if not bool(msg.is_stop):
            self.sentry.chassis_rotate_velocity = float(msg.rotate_velocity)
        else:
            self.sentry.chassis_rotate_velocity = 0.0

    def _on_set_local_costmap_enable(self, request, response):
        self.local_costmap_enabled = request.data
        response.success = True
        return response

    def is_sentry_motion_gui_allowed(self) -> bool:
        return self._sentry_motion_gui_allowed

    def is_sentry_motion_allowed(self) -> bool:
        return self._sentry_motion_gui_allowed

    def toggle_sentry_motion_allowed(self) -> bool:
        self._sentry_motion_gui_allowed = not self._sentry_motion_gui_allowed
        self.sentry.set_hp_enabled(self.is_sentry_motion_allowed())
        self.get_logger().info(
            f"Sentry motion GUI {'ALLOW' if self._sentry_motion_gui_allowed else 'BLOCK'}"
        )
        return self._sentry_motion_gui_allowed

    def _on_pub_timer(self):
        now = self.get_clock().now().to_msg()
        arr = RobotKinematicsArray()
        arr.header = Header()
        arr.header.stamp    = now
        arr.header.frame_id = self.frame_id

        kinematic_list = []
        for robot in self.all_robots:
            pos, vel, yaw = robot.snapshot()
            nx, ny = self.rng.normal(0.0, self.pos_noise_std, size=2)

            r = RobotKinematics()
            r.id = int(robot.robot_id)

            pose = PoseWithCovariance()
            pose.pose.position.x = float(pos[0] + nx)
            pose.pose.position.y = float(pos[1] + ny)
            pose.pose.position.z = 0.0
            half_yaw = yaw * 0.5
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            cov = [0.0] * 36
            cov[0] = self.pos_noise_std ** 2
            cov[7] = self.pos_noise_std ** 2
            pose.covariance = cov
            r.pose = pose

            tw = TwistWithCovariance()
            tw.twist.linear.x = float(vel[0])
            tw.twist.linear.y = float(vel[1])
            tw.twist.linear.z = 0.0
            tw.covariance = [0.0] * 36
            r.twist = tw

            kinematic_list.append(r)

        arr.robots = kinematic_list
        self.robots_pub.publish(arr)
        self._publish_local_costmap(now)

    def _publish_local_costmap(self, stamp):
        sentry_pos, _, _ = self.sentry.snapshot()
        res = self.map_res
        map_x0, map_y0, map_x1, map_y1 = self._map_world_bounds()

        raw_ox = float(sentry_pos[0] - 0.5 * self.local_costmap_w * res)
        raw_oy = float(sentry_pos[1] - 0.5 * self.local_costmap_h * res)
        raw_x1 = raw_ox + self.local_costmap_w * res
        raw_y1 = raw_oy + self.local_costmap_h * res

        ox = max(raw_ox, map_x0)
        oy = max(raw_oy, map_y0)
        x1 = min(raw_x1, map_x1)
        y1 = min(raw_y1, map_y1)
        ox = max(map_x0, math.floor(ox / res + 1e-9) * res)
        oy = max(map_y0, math.floor(oy / res + 1e-9) * res)
        w = int(math.floor((x1 - ox) / res + 1e-9))
        h = int(math.floor((y1 - oy) / res + 1e-9))
        if w <= 0 or h <= 0:
            return

        local = OccupancyGrid()
        local.header.stamp = stamp
        local.header.frame_id = self.frame_id
        local.info.resolution = res
        local.info.width = w
        local.info.height = h
        local.info.origin.position.x = ox
        local.info.origin.position.y = oy
        local.info.origin.position.z = 0.0
        local.info.origin.orientation.w = 1.0

        obstacle = self._sample_obstacle_mask(ox, oy, w, h)
        if self.local_costmap_enabled:
            for robot in self.extra_robots:
                pos, _, _ = robot.snapshot()
                self._stamp_robot_disk(obstacle, ox, oy, w, h, pos)

        costs = self._obstacle_mask_to_costs(obstacle)
        local.data = costs.reshape(-1).tolist()
        self.local_costmap_pub.publish(local)

    def _map_world_bounds(self):
        ox, oy, _ = self.map_meta.origin
        return (
            float(ox),
            float(oy),
            float(ox + self.map_w * self.map_res),
            float(oy + self.map_h * self.map_res),
        )

    def _sample_obstacle_mask(
        self, origin_x: float, origin_y: float, grid_w: int, grid_h: int
    ) -> np.ndarray:
        from sp_nav_sim.sim.sim_map import world_to_pixel_int, pixel_in_bounds

        mask = np.zeros((grid_h, grid_w), dtype=np.bool_)
        for row in range(grid_h):
            wy = origin_y + (row + 0.5) * self.map_res
            for col in range(grid_w):
                wx = origin_x + (col + 0.5) * self.map_res
                px, py = world_to_pixel_int(np.array([wx, wy]), self.map_meta, self.map_h)
                if pixel_in_bounds(px, py, self.map_w, self.map_h) and self.map_occ[py, px]:
                    mask[row, col] = True
        return mask

    def _stamp_robot_disk(
        self,
        mask: np.ndarray,
        origin_x: float,
        origin_y: float,
        grid_w: int,
        grid_h: int,
        pos: np.ndarray,
    ):
        radius_cells = max(1, int(math.ceil(self.robot_radius / self.map_res)))
        col_center = int((float(pos[0]) - origin_x) / self.map_res)
        row_center = int((float(pos[1]) - origin_y) / self.map_res)
        col0 = max(0, col_center - radius_cells)
        col1 = min(grid_w - 1, col_center + radius_cells)
        row0 = max(0, row_center - radius_cells)
        row1 = min(grid_h - 1, row_center + radius_cells)
        radius_sq = self.robot_radius * self.robot_radius
        for row in range(row0, row1 + 1):
            cy = origin_y + (row + 0.5) * self.map_res
            dy = cy - float(pos[1])
            for col in range(col0, col1 + 1):
                cx = origin_x + (col + 0.5) * self.map_res
                dx = cx - float(pos[0])
                if dx * dx + dy * dy <= radius_sq:
                    mask[row, col] = True

    def _obstacle_mask_to_costs(self, obstacle: np.ndarray) -> np.ndarray:
        free_u8 = np.where(obstacle, 0, 255).astype(np.uint8)
        try:
            import cv2
            dist_px = cv2.distanceTransform(free_u8, cv2.DIST_L2, 5)
        except ImportError:
            dist_px = self._edt_numpy(obstacle)

        dist_m = dist_px.astype(np.float32) * float(self.map_res)
        lethal = float(self.robot_radius + self.margin)
        costs = np.zeros(obstacle.shape, dtype=np.int8)

        clearance = dist_m - lethal
        lethal_mask = clearance < 0.0
        free_mask = dist_m >= self.d_safe
        soft_mask = ~(lethal_mask | free_mask)

        costs[lethal_mask] = 100
        costs[free_mask] = 0
        if np.any(soft_mask):
            diff = self.d_safe - dist_m[soft_mask]
            penalty = self.w_pen * diff * diff
            scaled = 100.0 * (1.0 - np.exp(-penalty))
            costs[soft_mask] = np.clip(np.rint(scaled), 0, 99).astype(np.int8)
        return costs

    @staticmethod
    def _edt_numpy(obstacle: np.ndarray) -> np.ndarray:
        h, w = obstacle.shape
        dist = np.full((h, w), 1e9, dtype=np.float32)
        ys, xs = np.nonzero(obstacle)
        if ys.size == 0:
            return np.full((h, w), 1e6, dtype=np.float32)
        for row in range(h):
            for col in range(w):
                if obstacle[row, col]:
                    dist[row, col] = 0.0
                else:
                    dy = ys.astype(np.float32) - row
                    dx = xs.astype(np.float32) - col
                    dist[row, col] = float(np.sqrt(np.min(dy * dy + dx * dx)))
        return dist

    def _on_odom_timer(self):
        now = self.get_clock().now().to_msg()

        pos, vel, yaw = self.sentry.snapshot()
        pos_odom = pos - self.odom_origin_
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.sentry.tf_child_frame_id
        odom.pose.pose.position.x = float(pos_odom[0])
        odom.pose.pose.position.y = float(pos_odom[1])
        odom.pose.pose.position.z = 0.0
        half_yaw = yaw * 0.5
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.covariance    = [0.0] * 36
        odom.pose.covariance[0] = self.pos_noise_std ** 2
        odom.pose.covariance[7] = self.pos_noise_std ** 2
        odom.twist.twist.linear.x  = float(vel[0])
        odom.twist.twist.linear.y  = float(vel[1])

        z_v_override = self.sentry.get_z_vel_override()
        if z_v_override != 0.0:
            odom.twist.twist.linear.z  = z_v_override
        else:
            odom.twist.twist.linear.z  = 0.0

        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance = [0.0] * 36
        self.odom_pub.publish(odom)

        self.tf_broadcaster.sendTransform(
            self._make_tf(self.odom_frame, self.sentry.tf_child_frame_id,
                          pos_odom, yaw, now)
        )

        for robot in self.extra_robots:
            pos_r, _, yaw_r = robot.snapshot()
            self.tf_broadcaster.sendTransform(
                self._make_tf(self.frame_id, robot.tf_child_frame_id,
                              pos_r, yaw_r, now)
            )

    @staticmethod
    def _make_tf(parent: str, child: str,
                 pos: np.ndarray, yaw: float, stamp) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = 0.0
        half_yaw = yaw * 0.5
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        return t

def main():
    rclpy.init()
    node = None
    try:
        node = SimRobotNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        while rclpy.ok():
            now_ns = node.get_clock().now().nanoseconds
            sentry_motion_allowed = node.is_sentry_motion_allowed()
            sentry_motion_blocked = not sentry_motion_allowed
            node.gui.tick(sentry_motion_blocked, now_ns)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
