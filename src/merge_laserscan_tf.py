#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros


def finite_positive(r: float) -> bool:
    return math.isfinite(r) and r > 0.0


def yaw_of(q) -> float:
    # quaternion -> yaw (Z)
    ysqr = q.y * q.y
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (ysqr + q.z * q.z)
    return math.atan2(t3, t4)


def scan_to_points_in_target(scan: LaserScan, tf: TransformStamped) -> List[Tuple[float, float]]:
    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    yaw = yaw_of(tf.transform.rotation)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    pts = []
    angle = scan.angle_min
    for r in scan.ranges:
        if finite_positive(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            # 2D rigid transform (z 무시)
            X = tx + x * cy - y * sy
            Y = ty + x * sy + y * cy
            pts.append((X, Y))
        angle += scan.angle_increment
    return pts


class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')

        # params
        self.scan0_topic = self.declare_parameter('scan0_topic', '/scan0').get_parameter_value().string_value
        self.scan1_topic = self.declare_parameter('scan1_topic', '/scan1').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('output_topic', '/scan_merged').get_parameter_value().string_value
        self.target_frame = self.declare_parameter('target_frame', 'base_scan').get_parameter_value().string_value
        self.tf_timeout_sec = self.declare_parameter('tf_timeout_sec', 0.2).get_parameter_value().double_value
        self.pub_timer = self.create_timer(0.1, self.try_merge) 
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub0 = self.create_subscription(LaserScan, self.scan0_topic, self.cb0, qos)
        self.sub1 = self.create_subscription(LaserScan, self.scan1_topic, self.cb1, qos)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.s0 = None  # type: LaserScan | None
        self.s1 = None  # type: LaserScan | None

        self.get_logger().info(f"LaserScanMerger up: {self.scan0_topic} + {self.scan1_topic} -> {self.output_topic} (frame={self.target_frame})")

    def cb0(self, msg: LaserScan):
        self.s0 = msg
        # self.try_merge()

    def cb1(self, msg: LaserScan):
        self.s1 = msg
        # self.try_merge()

    def try_merge(self):
        if self.s0 is None or self.s1 is None:
            return

        # 최신 TF 조회 (timeout 포함)
        try:
            tf0 = self.tf_buffer.lookup_transform(
                self.target_frame, self.s0.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec))
            tf1 = self.tf_buffer.lookup_transform(
                self.target_frame, self.s1.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # 출력 스캔 메타
        out = LaserScan()
        out.header.frame_id = self.target_frame
        # 두 입력 중 더 최신 stamp 사용
        out.header.stamp = self.s0.header.stamp if self.s0.header.stamp.sec >= self.s1.header.stamp.sec else self.s1.header.stamp

        # 각도/증분 설정
        out.angle_min = -math.pi
        out.angle_max = math.pi
        inc0 = abs(self.s0.angle_increment) if self.s0.angle_increment != 0.0 else 0.0
        inc1 = abs(self.s1.angle_increment) if self.s1.angle_increment != 0.0 else 0.0
        if inc0 > 0.0 and inc1 > 0.0:
            out.angle_increment = min(inc0, inc1)
        elif inc0 > 0.0:
            out.angle_increment = inc0
        elif inc1 > 0.0:
            out.angle_increment = inc1
        else:
            out.angle_increment = math.radians(0.5)  # fallback

        out.range_min = min(self.s0.range_min, self.s1.range_min)
        out.range_max = max(self.s0.range_max, self.s1.range_max)
        out.time_increment = 0.0
        out.scan_time = 0.0

        N = int(math.floor((out.angle_max - out.angle_min) / out.angle_increment)) + 1
        ranges = [float('inf')] * N

        # 두 스캔을 target_frame 으로 변환 → polar binning(최소거리)
        pts: List[Tuple[float, float]] = []
        pts += scan_to_points_in_target(self.s0, tf0)
        pts += scan_to_points_in_target(self.s1, tf1)

        for (x, y) in pts:
            r = math.hypot(x, y)
            if r < out.range_min or r > out.range_max:
                continue
            th = math.atan2(y, x)
            idx = int(round((th - out.angle_min) / out.angle_increment))
            if 0 <= idx < N and r < ranges[idx]:
                ranges[idx] = float(r)  # 명시적 float 변환

        # ranges 타입/값 보정 (NaN/Inf 안전)
        out.ranges = [float(v) if math.isfinite(v) else float('inf') for v in ranges]
        out.intensities = []  # 사용 안 함
        
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LaserScanMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
