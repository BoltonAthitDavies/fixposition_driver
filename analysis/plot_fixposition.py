#!/usr/bin/env python3
"""Plot Fixposition data, reproducing the PlotJuggler tabs (config.xml).

Reads from EITHER:
  - a CSV log directory produced by log_fixposition.py, or
  - a ros2 bag (a directory with metadata.yaml, or a .mcap / .db3 file).

Creates figures matching config.xml plus an X/Y trajectory:
    - imu:  angular velocity, linear acceleration, orientation (roll/pitch/yaw)
    - gnss: latitude, longitude, altitude
    - odom: ECEF position x/y/z and orientation roll/pitch/yaw
    - path_xy: top-down ENU trajectory

Time axis is seconds relative to the earliest sample across all topics.

Usage:
    python3 analysis/plot_fixposition.py                     # newest CSV dir under ./fixposition_logs
    python3 analysis/plot_fixposition.py fixposition_bag     # a ros2 bag (auto-detected)
    python3 analysis/plot_fixposition.py <csv_dir>           # a CSV log dir
    python3 analysis/plot_fixposition.py <source> --save     # write PNGs instead of showing
    python3 analysis/plot_fixposition.py <source> --bag      # force bag interpretation

Reading a bag requires ROS sourced (rosbag2_py):
    source /opt/ros/humble/setup.bash
    source ~/fixposition_driver/install/setup.bash
"""

import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd

# reuse the exact quaternion->rpy conversion used by the logger
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_fixposition import quat_to_rpy  # noqa: E402

# topic -> short name used for the per-topic DataFrames
TOPIC_MAP = {
    "/fixposition/poiimu": "poiimu",
    "/fixposition/odometry_llh": "odometry_llh",
    "/fixposition/odometry_ecef": "odometry_ecef",
    "/fixposition/odometry_enu": "odometry_enu",
}


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# --------------------------------------------------------------------------- #
# Source loading (CSV dir or ros2 bag)
# --------------------------------------------------------------------------- #
def newest_log_dir():
    dirs = sorted(glob.glob(os.path.join("fixposition_logs", "*")))
    return dirs[-1] if dirs else None


def looks_like_bag(path):
    if os.path.isfile(path) and path.endswith((".mcap", ".db3")):
        return True
    if os.path.isdir(path):
        return os.path.exists(os.path.join(path, "metadata.yaml"))
    return False


def load_csv_dir(log_dir):
    data = {}
    for name in TOPIC_MAP.values():
        path = os.path.join(log_dir, f"{name}.csv")
        if os.path.exists(path):
            df = pd.read_csv(path)
            if not df.empty:
                data[name] = df
    return data


def _storage_id_and_uri(bag_path):
    """Return (uri, storage_id) for rosbag2, auto-detecting the storage format."""
    if os.path.isfile(bag_path):
        return bag_path, ("mcap" if bag_path.endswith(".mcap") else "sqlite3")
    # directory: read metadata.yaml for the storage identifier
    import yaml
    meta_path = os.path.join(bag_path, "metadata.yaml")
    try:
        meta = yaml.safe_load(open(meta_path))
        sid = meta["rosbag2_bagfile_information"]["storage_identifier"]
    except Exception:
        sid = "sqlite3"
    return bag_path, sid


def load_bag(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    uri, storage_id = _storage_id_and_uri(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    rows = {name: [] for name in TOPIC_MAP.values()}
    msg_cache = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        name = TOPIC_MAP.get(topic)
        if name is None:
            continue
        if topic not in msg_cache:
            msg_cache[topic] = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_cache[topic])
        t = stamp_to_sec(msg.header.stamp)

        if name == "poiimu":
            r, p, y = quat_to_rpy(msg.orientation.x, msg.orientation.y,
                                  msg.orientation.z, msg.orientation.w)
            rows[name].append([t,
                               msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z,
                               msg.linear_acceleration.x, msg.linear_acceleration.y,
                               msg.linear_acceleration.z, r, p, y])
        elif name == "odometry_llh":
            rows[name].append([t, msg.latitude, msg.longitude, msg.altitude])
        else:  # odometry_ecef / odometry_enu
            q = msg.pose.pose.orientation
            pos = msg.pose.pose.position
            r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
            rows[name].append([t, pos.x, pos.y, pos.z, r, p, y])

    cols = {
        "poiimu": ["stamp", "ang_vel_x", "ang_vel_y", "ang_vel_z",
                   "lin_acc_x", "lin_acc_y", "lin_acc_z", "roll", "pitch", "yaw"],
        "odometry_llh": ["stamp", "latitude", "longitude", "altitude"],
        "odometry_ecef": ["stamp", "pos_x", "pos_y", "pos_z", "roll", "pitch", "yaw"],
        "odometry_enu": ["stamp", "pos_x", "pos_y", "pos_z", "roll", "pitch", "yaw"],
    }
    data = {}
    for name, r in rows.items():
        if r:
            data[name] = pd.DataFrame(r, columns=cols[name])
    return data


def load_source(source, force_bag=False):
    if force_bag or looks_like_bag(source):
        return load_bag(source)
    return load_csv_dir(source)


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", nargs="?", default=None,
                        help="CSV log dir or ros2 bag (default: newest under ./fixposition_logs)")
    parser.add_argument("--bag", action="store_true",
                        help="force interpreting SOURCE as a ros2 bag")
    parser.add_argument("--save", action="store_true",
                        help="save PNGs into the source dir instead of showing windows")
    args = parser.parse_args()

    source = args.source or newest_log_dir()
    if not source or not os.path.exists(source):
        sys.exit("No source found. Run log_fixposition.py, record a bag, "
                 "or pass a path explicitly.")

    data = load_source(source, force_bag=args.bag)
    imu = data.get("poiimu")
    llh = data.get("odometry_llh")
    ecef = data.get("odometry_ecef")
    enu = data.get("odometry_enu")

    stamps = [d["stamp"] for d in (imu, llh, ecef, enu) if d is not None]
    if not stamps:
        sys.exit(f"No matching Fixposition topics found in {source}")
    t0 = min(s.iloc[0] for s in stamps)
    for d in (imu, llh, ecef, enu):
        if d is not None:
            d["t"] = d["stamp"] - t0

    figs = []

    # ---- IMU tab ----
    if imu is not None:
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
        fig.suptitle("IMU  (/fixposition/poiimu)")
        for c in ("ang_vel_x", "ang_vel_y", "ang_vel_z"):
            ax[0].plot(imu["t"], imu[c], label=c)
        ax[0].set_ylabel("angular vel\n[rad/s]"); ax[0].legend(loc="upper right", ncol=3)
        for c in ("lin_acc_x", "lin_acc_y", "lin_acc_z"):
            ax[1].plot(imu["t"], imu[c], label=c)
        ax[1].set_ylabel("linear acc\n[m/s^2]"); ax[1].legend(loc="upper right", ncol=3)
        for c in ("roll", "pitch", "yaw"):
            ax[2].plot(imu["t"], imu[c], label=c)
        ax[2].set_ylabel("orientation\n[rad]"); ax[2].legend(loc="upper right", ncol=3)
        ax[2].set_xlabel("time [s]")
        for a in ax:
            a.grid(True, alpha=0.3)
        fig.tight_layout()
        figs.append(("imu", fig))

    # ---- GNSS tab ----
    if llh is not None:
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
        fig.suptitle("GNSS  (/fixposition/odometry_llh)")
        ax[0].plot(llh["t"], llh["latitude"], color="tab:red")
        ax[0].set_ylabel("latitude [deg]")
        ax[1].plot(llh["t"], llh["longitude"], color="tab:green")
        ax[1].set_ylabel("longitude [deg]")
        ax[2].plot(llh["t"], llh["altitude"], color="tab:orange")
        ax[2].set_ylabel("altitude [m]"); ax[2].set_xlabel("time [s]")
        for a in ax:
            a.grid(True, alpha=0.3)
        fig.tight_layout()
        figs.append(("gnss", fig))

    # ---- ODOM tab ----
    if ecef is not None:
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(11, 8))
        fig.suptitle("Odometry ECEF  (/fixposition/odometry_ecef)")
        for c in ("pos_x", "pos_y", "pos_z"):
            ax[0].plot(ecef["t"], ecef[c], label=c)
        ax[0].set_ylabel("position [m]"); ax[0].legend(loc="upper right", ncol=3)
        for c in ("roll", "pitch", "yaw"):
            ax[1].plot(ecef["t"], ecef[c], label=c)
        ax[1].set_ylabel("orientation [rad]"); ax[1].legend(loc="upper right", ncol=3)
        ax[1].set_xlabel("time [s]")
        for a in ax:
            a.grid(True, alpha=0.3)
        fig.tight_layout()
        figs.append(("odom", fig))

    # ---- X/Y path (top-down trajectory) ----
    # Prefer ENU (already a local East/North frame). If ENU is unavailable
    # (published only once fusion/GNSS establishes the ENU origin), fall back to
    # ECEF re-centered on its first sample -- same path shape, axes not true E/N.
    if enu is not None:
        path_df, title, xlabel, ylabel = (
            enu, "Odometry path X/Y  (/fixposition/odometry_enu)",
            "East  x [m]", "North  y [m]")
        px, py = path_df["pos_x"], path_df["pos_y"]
    elif ecef is not None:
        path_df, title = (ecef,
                          "Odometry path X/Y  (ECEF re-centered; ENU unavailable)")
        xlabel, ylabel = "x - x0 [m]", "y - y0 [m]"
        px = ecef["pos_x"] - ecef["pos_x"].iloc[0]
        py = ecef["pos_y"] - ecef["pos_y"].iloc[0]
    else:
        path_df = None

    if path_df is not None:
        fig, ax = plt.subplots(figsize=(9, 9))
        fig.suptitle(title)
        sc = ax.scatter(px, py, c=path_df["t"], cmap="viridis", s=8)
        ax.plot(px, py, color="gray", alpha=0.3, lw=0.8)
        ax.scatter(px.iloc[0], py.iloc[0], c="green", s=90, marker="o",
                   label="start", zorder=5)
        ax.scatter(px.iloc[-1], py.iloc[-1], c="red", s=90, marker="X",
                   label="end", zorder=5)
        ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3); ax.legend(loc="best")
        fig.colorbar(sc, ax=ax, label="time [s]")
        fig.tight_layout()
        figs.append(("path_xy", fig))

    if args.save:
        out_dir = source if os.path.isdir(source) else os.path.dirname(source) or "."
        for name, fig in figs:
            out = os.path.join(out_dir, f"{name}.png")
            fig.savefig(out, dpi=120)
            print(f"saved {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
