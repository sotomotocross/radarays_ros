#!/usr/bin/env python3
"""Radar-image-to-detection-scan converter.

radarays_embree_sensor_system/radarays_optix_sensor_system publish a real,
material-aware polar radar image (mono8, height=n_cells range bins,
width=n_angles angle bins, row-major: pixel[row * n_angles + angle_id],
row 0 = near range, row (n_cells-1) = range_max) -- but nothing turns that
into a detection-quality range measurement per angle. rmagine's own
generic /scan+/points path is a pure geometric raycast with no material
awareness at all, so it can't do this either: every ray that hits
anything (including a barely-reflective surface like water) is reported
as a real detection.

This node does what a real radar's own signal-processing chain does:
per angle column, walk outward from near range and report the first
(nearest) range bin whose intensity exceeds a threshold as the leading
edge of a real detection -- weak, low-reflectivity returns (calibrated
material tagging keeps water's own reflectivity low) don't cross that
threshold and correctly produce "no detection" (+inf), the same
convention sensor_msgs/msg/LaserScan already uses elsewhere in this
pipeline. Unlike fully excluding water from the ray-traced scene (a
blunt workaround that also throws away any chance of ever modeling real,
weak sea clutter), this keeps water in the scene -- its material
determines whether/how much it shows up, exactly like a real radar's own
clutter behavior.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


class RadarImageToScan(Node):
    def __init__(self):
        super().__init__('radar_image_to_scan')

        self.declare_parameter('image_topic', 'radar/image')
        self.declare_parameter('scan_topic', 'radar/detection_scan')
        self.declare_parameter('frame_id', 'radar')
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 1000.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        # 8-bit image intensity (0-255) a range bin must exceed to count as
        # a real detection, not clutter. No universal "correct" value --
        # calibrate against real captured images for the materials in use
        # (see the launch/README note this script ships with).
        self.declare_parameter('intensity_threshold', 40)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.intensity_threshold = int(self.get_parameter('intensity_threshold').value)

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'radar_image_to_scan up: {self.image_topic} -> {self.scan_topic} '
            f'(threshold={self.intensity_threshold})'
        )

    def _on_image(self, msg: Image) -> None:
        n_cells = msg.height  # range bins, row-major, row 0 = near range
        n_angles = msg.width  # angle bins
        if n_cells == 0 or n_angles == 0:
            return

        resolution = self.range_max / n_cells
        data = bytes(msg.data)
        ranges = [float('inf')] * n_angles

        for col in range(n_angles):
            for row in range(n_cells):
                if data[row * n_angles + col] > self.intensity_threshold:
                    ranges[col] = row * resolution
                    break

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / max(n_angles - 1, 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = RadarImageToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
