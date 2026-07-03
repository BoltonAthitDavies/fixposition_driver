#!/usr/bin/env python3
"""Log the Fixposition topics shown in the PlotJuggler layout (config.xml) to CSV.

Subscribes to the same three topics PlotJuggler plots and writes one CSV per
topic. Orientation quaternions are converted to roll/pitch/yaw (radians), the
same way PlotJuggler's "Quaternion to RPY" plugin does.

Usage:
    source /opt/ros/humble/setup.bash
    source ~/fixposition_driver/install/setup.bash
    python3 analysis/log_fixposition.py                 # logs until Ctrl-C
    python3 analysis/log_fixposition.py --duration 30    # logs for 30 s
    python3 analysis/log_fixposition.py -o /tmp/mylogs   # custom output dir
"""

import argparse
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix


def quat_to_rpy(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in radians (ZYX / aerospace convention)."""
    # roll (rotation about x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (rotation about y)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    # yaw (rotation about z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class FixpositionLogger(Node):
    def __init__(self, out_dir):
        super().__init__("fixposition_logger")
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self._files = {}
        self._writers = {}
        self._counts = {}

        self._open("poiimu",
                   ["stamp", "ang_vel_x", "ang_vel_y", "ang_vel_z",
                    "lin_acc_x", "lin_acc_y", "lin_acc_z",
                    "roll", "pitch", "yaw"])
        self._open("odometry_llh", ["stamp", "latitude", "longitude", "altitude"])
        self._open("odometry_ecef",
                   ["stamp", "pos_x", "pos_y", "pos_z", "roll", "pitch", "yaw"])
        self._open("odometry_enu",
                   ["stamp", "pos_x", "pos_y", "pos_z", "roll", "pitch", "yaw"])

        # QoS matches the driver's "sensor_short" (best-effort) profile
        self.create_subscription(Imu, "/fixposition/poiimu",
                                 self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/fixposition/odometry_llh",
                                 self.on_llh, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/fixposition/odometry_ecef",
                                 self.on_ecef, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/fixposition/odometry_enu",
                                 self.on_enu, qos_profile_sensor_data)

        self.get_logger().info(f"Logging to {out_dir} (Ctrl-C to stop)")

    def _open(self, name, header):
        path = os.path.join(self.out_dir, f"{name}.csv")
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(header)
        self._files[name] = f
        self._writers[name] = w
        self._counts[name] = 0

    def on_imu(self, msg):
        r, p, y = quat_to_rpy(msg.orientation.x, msg.orientation.y,
                              msg.orientation.z, msg.orientation.w)
        self._writers["poiimu"].writerow([
            stamp_to_sec(msg.header.stamp),
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            r, p, y])
        self._counts["poiimu"] += 1

    def on_llh(self, msg):
        self._writers["odometry_llh"].writerow([
            stamp_to_sec(msg.header.stamp),
            msg.latitude, msg.longitude, msg.altitude])
        self._counts["odometry_llh"] += 1

    def _write_odom(self, name, msg):
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._writers[name].writerow([
            stamp_to_sec(msg.header.stamp),
            pos.x, pos.y, pos.z, r, p, y])
        self._counts[name] += 1

    def on_ecef(self, msg):
        self._write_odom("odometry_ecef", msg)

    def on_enu(self, msg):
        self._write_odom("odometry_enu", msg)

    def close(self):
        for name, f in self._files.items():
            f.flush()
            f.close()
        summary = ", ".join(f"{n}={c}" for n, c in self._counts.items())
        self.get_logger().info(f"Rows written: {summary}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-o", "--output-dir", default=None,
                        help="output directory (default: ./fixposition_logs/<timestamp>)")
    parser.add_argument("-d", "--duration", type=float, default=None,
                        help="stop automatically after N seconds")
    args = parser.parse_args()

    out_dir = args.output_dir or os.path.join(
        "fixposition_logs", datetime.now().strftime("%Y%m%d_%H%M%S"))

    rclpy.init()
    node = FixpositionLogger(out_dir)
    if args.duration is not None:
        node.create_timer(args.duration, lambda: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\nLogs saved in: {out_dir}")


if __name__ == "__main__":
    main()
