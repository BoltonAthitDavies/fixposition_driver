# Fixposition logging & plotting

Python pipeline that mirrors the PlotJuggler layout (`config.xml`): log the same
Fixposition topics to CSV, then plot them with matplotlib.

## Topics logged

| CSV file | Topic | Type | Columns |
|---|---|---|---|
| `poiimu.csv` | `/fixposition/poiimu` | `sensor_msgs/Imu` | ang_vel x/y/z, lin_acc x/y/z, roll/pitch/yaw |
| `odometry_llh.csv` | `/fixposition/odometry_llh` | `sensor_msgs/NavSatFix` | latitude, longitude, altitude |
| `odometry_ecef.csv` | `/fixposition/odometry_ecef` | `nav_msgs/Odometry` | pos x/y/z, roll/pitch/yaw |
| `odometry_enu.csv` | `/fixposition/odometry_enu` | `nav_msgs/Odometry` | pos x/y/z, roll/pitch/yaw (used for the X/Y path) |

Orientation quaternions are converted to roll/pitch/yaw in **radians** (same as
PlotJuggler's "Quaternion to RPY" plugin). `stamp` is the message header time in
seconds; the plotter shifts everything to a common relative start.

## Usage

Source ROS and the workspace first (a new terminal already does this via `.bashrc`):

```bash
source /opt/ros/humble/setup.bash
source ~/fixposition_driver/install/setup.bash
```

**1. Log** (with the driver running and publishing):

```bash
python3 analysis/log_fixposition.py                 # log until Ctrl-C
python3 analysis/log_fixposition.py --duration 30   # log for 30 s
python3 analysis/log_fixposition.py -o /tmp/run1    # custom output dir
```

CSVs go to `fixposition_logs/<timestamp>/` by default.

**2. Plot** — from a CSV log dir **or** a ros2 bag:

```bash
python3 analysis/plot_fixposition.py                    # newest CSV dir, show windows
python3 analysis/plot_fixposition.py fixposition_logs/20260701_1452
python3 analysis/plot_fixposition.py fixposition_bag    # a ros2 bag (auto-detected)
python3 analysis/plot_fixposition.py <source> --save    # write PNGs instead of showing
python3 analysis/plot_fixposition.py <source> --bag     # force bag interpretation
```

A "bag" is either a bag directory (has `metadata.yaml`) or a `.mcap`/`.db3` file.
Reading a bag needs ROS sourced (uses `rosbag2_py`); the same field extraction and
quaternion->RPY conversion as the CSV path is applied.

Figures produced: `imu`, `gnss`, `odom`, and `path_xy` (top-down trajectory,
colored by time, with start/end markers).

### Recording a bag to plot later

```bash
ros2 bag record -o fixposition_bag \
  /fixposition/poiimu /fixposition/odometry_llh \
  /fixposition/odometry_ecef /fixposition/odometry_enu
# ... move the sensor ... then Ctrl-C, and:
python3 analysis/plot_fixposition.py fixposition_bag --save
```

**X/Y path source:** the path prefers `/fixposition/odometry_enu` (true local
East/North). ENU only publishes once fusion/GNSS establishes the ENU origin — if
the bag/log has no ENU samples (e.g. stationary, no fix), the path automatically
falls back to ECEF re-centered on its first sample (same shape, axes not true E/N,
noted in the title).

## Notes

- QoS is best-effort to match the driver's `sensor_short` profile.
- The X/Y path uses the **ENU** odometry (local meters), not ECEF (which is in
  millions of meters and not useful for a top-down view).
- Need the raw messages instead? `ros2 bag record /fixposition/...` is the
  alternative; this pipeline is the lightweight CSV route for quick analysis.
