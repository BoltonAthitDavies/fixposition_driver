# Fixposition Vision-RTK 2 ROS2 Driver

ROS2 driver for the [Fixposition Vision-RTK 2](https://www.fixposition.com/product) sensor on **Linux** (ROS 2 Humble).

Forked from [fixposition/fixposition\_driver](https://github.com/fixposition/fixposition_driver) with added launch tooling for automatic sensor discovery over WiFi and Ethernet.

> [!IMPORTANT]
> Tags 8.0.0 onward break compatibility with previous versions. See the [Migration Guide](https://docs.fixposition.com/fd/migration-guide).

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Fixposition Vision-RTK 2 sensor reachable on the network (WiFi or Ethernet)
- Sensor TCP0 port (21000) configured with the required output messages via the sensor web UI
- *(Optional, for the camera stream)* GStreamer H.264 decoder plugins:
  `sudo apt install gstreamer1.0-libav gstreamer1.0-plugins-bad`

## Build

```bash
# Ignore ROS1 and example packages
touch fixposition-sdk/fpsdk_ros1/COLCON_IGNORE
touch fixposition-sdk/examples/COLCON_IGNORE

# Build
source /opt/ros/humble/setup.bash
colcon build --packages-select fpsdk_common fpsdk_ros2 --cmake-args -DBUILD_TESTING=OFF
colcon build --cmake-args -DBUILD_TESTING=OFF
```

## Launching the Driver

There are several ways to launch the driver. Pick whichever fits your setup.

### Option 1: Auto-discovery script (recommended)

```bash
./run_driver.sh
```

Best for day-to-day use. The script automatically finds the sensor on the network — it tries WiFi first (resolving the mDNS name `fp-0e47c8.local`) then falls back to Ethernet (`10.0.1.1`). It writes a runtime config to `~/.config/fixposition/config.yaml` with the resolved IP and launches the driver. No manual IP editing needed.

Environment variable overrides:

| Variable | Default | Description |
|---|---|---|
| `FP_SENSOR_HOST` | `fp-0e47c8.local` | Sensor mDNS hostname (WiFi) |
| `FP_SENSOR_ETH_IP` | `10.0.1.1` | Sensor static IP (Ethernet fallback) |
| `FP_SENSOR_PORT` | `21000` | Sensor TCP data port |

### Option 2: Direct ROS2 launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fixposition_driver_ros2 node.launch
```

Uses the config at [fixposition_driver_ros2/launch/config.yaml](fixposition_driver_ros2/launch/config.yaml). You need to set the sensor IP manually in the `stream:` line:

```yaml
stream: tcpcli://<sensor-ip>:21000
```

Use this when you know the sensor's IP won't change (e.g. static IP or DHCP reservation).

### Option 3: Verify raw data first (no ROS)

```bash
nc <sensor-ip> 21000
```

Not a launch method — just a quick sanity check. If you see `$FP,...` lines streaming, the sensor is reachable and outputting data. Useful before launching the driver for the first time or when debugging connection issues.

## Coordinate System

The driver uses **ECEF** (Earth-Centered, Earth-Fixed) as the global frame — not a base station.

### Frames

| Frame | Description |
|---|---|
| `FP_ECEF` | Global ECEF frame (absolute position on Earth). Tree root |
| `FP_ENU0` | Local East-North-Up tangent plane at a fixed origin point. Static relative to `FP_ECEF` |
| `FP_POI` | Point of Interest — the vehicle reference point at the configured output location |
| `FP_POISH` | Smoothed POI — same as `FP_POI` but without position jumps |
| `FP_VRTK` | Vision-RTK sensor body frame. Also the frame of the raw/corrected IMU and bias topics |
| `FP_CAM` | Camera frame (fixed extrinsic from `FP_VRTK`) |
| `FP_IMUH` | IMU level (horizontal) frame relative to `FP_POI`. Roll/pitch only; yaw is zeroed |
| `GNSS1` / `GNSS2` | Primary / secondary GNSS antenna positions |

### TF Tree

```
FP_ECEF                                       (tree root)
├── FP_ENU0        (static,  FP_A-TF ECEF→ENU0)
└── FP_POI         (dynamic, FP_A-ODOMETRY)
    ├── FP_IMUH    (dynamic, FP_A-TF POI→IMUH)
    ├── FP_POISH   (dynamic, FP_A-TF POI→POISH)
    └── FP_VRTK    (static,  FP_A-TF POI→VRTK)
        └── FP_CAM (static,  FP_A-TF VRTK→CAM)
```

Notes:

- `FP_POI` has a single parent (`FP_ECEF`); its pose in `FP_ENU0` is obtained by
  chaining through `FP_ECEF`. There is **no** direct `FP_ENU0 → FP_POI` transform
  in default mode — `/odometry_enu` is published as a *topic*, not a TF.
- Which TF edges appear depends on the messages enabled on the sensor's TCP0
  output: `FP_A-TF_ECEFENU0`, `POIVRTK`, `VRTKCAM`, `POIIMUH`, `POIPOISH`.
- All TF edges require an initialized fusion engine; before that the driver logs
  "Is the fusion engine initialized?" and skips them.

With `nav2_mode: true`, the driver instead publishes the standard ROS2 Navigation2
convention (the `FP_A-ODOMENU`/`ODOMSH` transforms are used here):

```
FP_ECEF → FP_ENU0 → map (static identity)
                      └── odom
                            └── vrtk_link
```

## Published Topics

All topics are under the `/fixposition` namespace by default (`output_ns` in config).

### Odometry (~10 Hz, from fusion)

| Topic | Type | Frame | Description |
|---|---|---|---|
| `/odometry_ecef` | `nav_msgs/Odometry` | `FP_ECEF` → `FP_POI` | Absolute ECEF position + velocity |
| `/odometry_enu` | `nav_msgs/Odometry` | `FP_ENU0` → `FP_POI` | Local ENU position + velocity |
| `/odometry_llh` | `sensor_msgs/NavSatFix` | `FP_POI` | Lat/Lon/Height (converted from ECEF) |
| `/odometry_smooth` | `nav_msgs/Odometry` | `FP_ECEF` → `FP_POISH` | Smooth ECEF (no jumps) |
| `/odometry_enu_smooth` | `nav_msgs/Odometry` | `FP_ENU0` → `FP_POISH` | Smooth ENU (no jumps) |
| `/ypr` | `geometry_msgs/Vector3Stamped` | `FP_ENU0` | Yaw / Pitch / Roll in ENU |

### IMU

| Topic | Type | Rate | Frame | Description |
|---|---|---|---|---|
| `/poiimu` | `sensor_msgs/Imu` | ~10 Hz | `FP_ECEF` | Accel + angular vel at POI (from fusion). No orientation — use odometry topics for that |
| `/imu_ypr` | `geometry_msgs/Vector3Stamped` | ~10 Hz | `FP_POI` | Pitch/Roll from IMU only (yaw is zeroed) |
| `/fpa/rawimu` | `fpmsgs/FpaImu` | ~200 Hz | `FP_VRTK` | Raw accelerometer + gyroscope |
| `/fpa/corrimu` | `fpmsgs/FpaImu` | ~200 Hz | `FP_VRTK` | Bias-corrected accelerometer + gyroscope |
| `/fpa/imubias` | `fpmsgs/FpaImubias` | ~10 Hz | `FP_VRTK` | Estimated accel + gyro biases |

### GNSS

| Topic | Type | Description |
|---|---|---|
| `/gnss1` | `sensor_msgs/NavSatFix` | Primary GNSS antenna position |
| `/gnss2` | `sensor_msgs/NavSatFix` | Secondary GNSS antenna position |
| `/fpa/llh` | `fpmsgs/FpaLlh` | Lat/Lon/Height with ENU covariance |
| `/fpa/gnssant` | `fpmsgs/FpaGnssant` | Antenna state / power / age |
| `/fpa/gnsscorr` | `fpmsgs/FpaGnsscorr` | Correction data status (fix type, base station info) |

### Status & Epoch

| Topic | Type | Description |
|---|---|---|
| `/fpa/odometry` | `fpmsgs/FpaOdometry` | Full fusion odometry with status flags |
| `/fpa/odomenu` | `fpmsgs/FpaOdomenu` | Full ENU odometry with status flags |
| `/fpa/odomsh` | `fpmsgs/FpaOdomsh` | Full smooth odometry with status flags |
| `/fpa/odomstatus` | `fpmsgs/FpaOdomstatus` | Fusion status (IMU, GNSS, camera, wheelspeed) |
| `/fpa/eoe` | `fpmsgs/FpaEoe` | End-of-epoch marker |
| `/fpa/text` | `fpmsgs/FpaText` | Sensor text messages (errors, warnings, info) |
| `/fpa/tp` | `fpmsgs/FpaTp` | Time pulse info (GPS week, leap seconds) |
| `/fusion` | `fpmsgs/FusionEpoch` | Bundled fusion epoch (odometry + status + bias) |

### NMEA

| Topic | Type | Description |
|---|---|---|
| `/nmea/gga` | `fpmsgs/NmeaGga` | Fix data (position, quality, HDOP) |
| `/nmea/gll` | `fpmsgs/NmeaGll` | Position (lat/lon) |
| `/nmea/gsa` | `fpmsgs/NmeaGsa` | DOP and active satellites |
| `/nmea/gst` | `fpmsgs/NmeaGst` | Pseudorange error statistics |
| `/nmea/gsv` | `fpmsgs/NmeaGsv` | Satellites in view |
| `/nmea/hdt` | `fpmsgs/NmeaHdt` | True heading |
| `/nmea/rmc` | `fpmsgs/NmeaRmc` | Recommended minimum (date, time, speed, course) |
| `/nmea/vtg` | `fpmsgs/NmeaVtg` | Course/speed over ground |
| `/nmea/zda` | `fpmsgs/NmeaZda` | Date and time |
| `/nmea` | `fpmsgs/NmeaEpoch` | Bundled NMEA epoch |

### Camera (optional, disabled by default)

The sensor's camera video is streamed as H.264/RTP over UDP — a **separate path** from the TCP data stream — and republished as ROS2 image topics.

| Topic | Type | Frame | Description |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `FP_CAM` | Decoded frames (`bgr8`, 640×400, ~12 Hz) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | `FP_CAM` | JPEG-encoded frames |

**To enable it:**

1. Install the H.264 decoder plugins (see [Prerequisites](#prerequisites)).
2. Configure the sensor to stream the camera to **this host**: web UI → *Configuration → Camera → Stream*: set *Enable*, *Encoding* `H.264`, *Method* `Unicast`, *Destination* `<this-host-ip>:5004`. (Config is locked while Fusion is running — stop Fusion to change it.)
3. Set `camera.enabled: true` in [config.yaml](fixposition_driver_ros2/launch/config.yaml) and launch the driver.

```bash
ros2 run rqt_image_view rqt_image_view      # select either camera topic
ros2 topic hz /fixposition/camera/image_raw
```

Camera settings in `config.yaml`:

| Param | Default | Description |
|---|---|---|
| `camera.enabled` | `false` | Publish the camera image topics |
| `camera.port` | `5004` | UDP port the sensor unicasts the H.264 stream to |
| `camera.frame_id` | `FP_CAM` | `frame_id` stamped on published images |
| `camera.pipeline` | `""` | Optional full GStreamer pipeline override (empty = built from port) |

> [!NOTE]
> The capture runs in a background thread and reconnects automatically (using `reconnect_delay`). The JPEG encode is skipped when no one subscribes to the compressed topic. Frames only publish while the driver is running.

### Subscribed Topics (input)

| Topic | Type | Description |
|---|---|---|
| `/rtcm` | `rtcm_msgs/Message` | RTCM correction data |
| `/fixposition/speed` | `fpmsgs/Speed` | Wheelspeed input |

## Subscribing to Topics

The driver publishes with **BEST_EFFORT** reliability and **VOLATILE** durability (`qos_type: sensor_short`). When using `ros2 topic echo`, you must match this QoS:

```bash
ros2 topic echo /fixposition/odometry_enu --qos-reliability best_effort
```

Custom message types require sourcing the workspace first:

```bash
source install/setup.bash
ros2 topic echo /fixposition/fpa/odometry --qos-reliability best_effort
```

## Visualisation

```bash
# RViz
rviz2 -d ~/fixposition_driver/fixposition.rviz

# PlotJuggler
ros2 run plotjuggler plotjuggler
```

## Recording a Bag

```bash
ros2 bag record -o fixposition_bag \
  /tf /tf_static \
  /fixposition/fpa/corrimu /fixposition/fpa/eoe /fixposition/fpa/gnssant \
  /fixposition/fpa/gnsscorr /fixposition/fpa/imubias /fixposition/fpa/llh \
  /fixposition/fpa/odomenu /fixposition/fpa/odometry /fixposition/fpa/odomsh \
  /fixposition/fpa/odomstatus /fixposition/fpa/rawimu /fixposition/fpa/text \
  /fixposition/fpa/tp /fixposition/fusion /fixposition/gnss1 /fixposition/gnss2 \
  /fixposition/imu_ypr /fixposition/nmea /fixposition/odometry_ecef \
  /fixposition/odometry_enu /fixposition/odometry_enu_smooth \
  /fixposition/odometry_llh /fixposition/odometry_smooth \
  /fixposition/poiimu /fixposition/speed /fixposition/ypr /rtcm \
  /ins/imu /ins/imu_bias /ins/lever_arm /ins/nav_sat_fix \
  /ins/nav_sat_ref /ins/ncom /ins/odometry /ins/path
```

The `/ins/*` topics are from the Xsens Vision Navigator (if running alongside). To include camera video, add `/fixposition/camera/image_raw/compressed` (JPEG — much smaller than raw).

## Datasets

Six bags have been recorded so far. They live in the workspace root but are
**git-ignored** (`fixposition_bag*/`), so they exist only locally. They fall
into two groups: three **bench recordings** made in Bangkok, where the sensor
barely moved, and three **vehicle recordings** made on the Rayong test track,
which are real drives.

| Bag | Recorded | Duration | Size | Location | Path length | Extent | Max speed | Camera |
|---|---|---|---|---|---|---|---|---|
| [fixposition_bag](fixposition_bag/) | 2026-07-02 14:22 | 88 s | 15 MB | Bangkok (13.7635 N, 100.5279 E) | 23 m | 7 × 6 m | 1.8 km/h | no |
| [fixposition_bag_01](fixposition_bag_01/) | 2026-07-02 15:22 | 153 s | 191 MB | Bangkok | 22 m | 4 × 5 m | 3.0 km/h | no |
| [fixposition_bag_02](fixposition_bag_02/) | 2026-07-02 15:34 | 174 s | 216 MB | Bangkok | 31 m | 5 × 5 m | 1.6 km/h | no |
| [fixposition_bag_wgv_001](fixposition_bag_wgv_001/) | 2026-07-08 10:04 | 159 s | 1.7 GB | Rayong track (12.9766 N, 101.4554 E) | 481 m | 71 × 207 m | 15.5 km/h | yes |
| [fixposition_bag_wgv_002](fixposition_bag_wgv_002/) | 2026-07-08 10:15 | 197 s | 2.3 GB | Rayong track | 526 m | 166 × 148 m | 14.6 km/h | yes |
| [fixposition_bag_wgv_003](fixposition_bag_wgv_003/) | 2026-07-08 10:22 | 208 s | 1.9 GB | Rayong track | 646 m | 161 × 116 m | 19.1 km/h | yes |

Path length, extent and max speed are measured from `/fixposition/odometry_llh`
and the `/fixposition/odometry_ecef` twist. The three Bangkok bags never travel
more than a few meters, so they are useful for checking IMU, fusion status and
topic plumbing — but **not** for trajectory evaluation. Use the `wgv` bags for
anything motion-related.

### Topic coverage

Not every bag has the same topics — the sensor's TCP0 output config changed
between sessions (`ODOMENU`/`ECEFENU0` were enabled after the first recording,
and the camera stream only for the `wgv` session).

| Topic group | `fixposition_bag` | `_01` / `_02` | `_wgv_001..003` |
|---|---|---|---|
| `odometry_ecef`, `odometry_llh`, `poiimu`, `fpa/rawimu`, `fpa/corrimu`, `fpa/llh`, `fpa/odometry`, `fpa/odomstatus`, `tf`, `tf_static` | yes | yes | yes |
| `odometry_enu`, `fpa/odomenu`, `ypr` | — | yes | yes |
| `imu_ypr` | — | — | yes |
| `camera/image_raw` | — | — | yes |

Topics that were recorded but never published by the sensor (all the `nmea/*`,
`gnss1`, `gnss2`, `fusion`, `fpa/eoe`, the `*_smooth` odometry, …) are present
in the bags with **zero** messages.

### Figures in each bag directory

Every bag directory ships pre-rendered plots, generated by the scripts in
[analysis/](analysis/):

| File | What it shows |
|---|---|
| `imu.png` | Angular velocity, linear acceleration, orientation |
| `gnss.png` | Latitude / longitude / altitude over time |
| `odom.png` | ECEF position and orientation over time |
| `path_xy.png` | Top-down ENU trajectory, colored by elapsed time |
| `path_map.html` / `path_map.png` | Trajectory on a satellite map, colored by speed |

## Analysis & Plotting

[analysis/plot_fixposition.py](analysis/plot_fixposition.py) reproduces the
PlotJuggler tabs with matplotlib, reading from **either** a ros2 bag (directory
with `metadata.yaml`, or a `.mcap`/`.db3` file) or a CSV log dir produced by
[analysis/log_fixposition.py](analysis/log_fixposition.py). With `--save` it
writes the figures as PNGs into the source directory:

```bash
source /opt/ros/humble/setup.bash        # rosbag2_py needed to read a bag
source install/setup.bash
python3 analysis/plot_fixposition.py fixposition_bag_01 --save
```

This produces `imu.png`, `gnss.png`, `odom.png`, and `path_xy.png` inside
`fixposition_bag_01/` (the PNGs in every bag directory were generated this
way — see [Datasets](#datasets)). See
[analysis/README.md](analysis/README.md) for the full logging + plotting
pipeline.

### How `path_xy.png` is plotted

`path_xy.png` is the top-down X/Y trajectory. The script:

1. **Reads the bag** with `rosbag2_py.SequentialReader`, deserializing each
   message and keeping the four Fixposition topics (`poiimu`, `odometry_llh`,
   `odometry_ecef`, `odometry_enu`). Timestamps are shifted to a common
   relative start; quaternions are converted to roll/pitch/yaw.
2. **Picks the path frame:** it prefers `/fixposition/odometry_enu` (a local
   East/North frame in meters, plotted directly). If the bag has no ENU samples
   — ENU only publishes once fusion/GNSS establishes the origin — it falls back
   to `/fixposition/odometry_ecef` re-centered on its first sample (same shape,
   axes not true E/N, noted in the title).
3. **Draws the figure:** points are scattered and **colored by elapsed time**
   (viridis colorbar), joined by a faint gray line, with a green **start**
   marker and a red **X end** marker, at equal aspect ratio.
4. **Saves** it as `path_xy.png` (dpi 120) in the bag directory.

### Trajectory on a satellite map (`path_map`)

[analysis/map_fixposition.py](analysis/map_fixposition.py) puts the same
trajectory on a real map instead of bare axes. It takes lat/lon from
`/fixposition/odometry_llh` and the speed from the `/fixposition/odometry_ecef`
twist, and writes into each bag directory:

- `path_map.html` — interactive Leaflet map (Google Satellite by default, with
  Hybrid / Roadmap / OpenStreetMap / Esri selectable in the layer control)
- `path_map.png` — a headless-Firefox screenshot of that HTML

The path is drawn segment by segment, colored blue → green → yellow → red by
speed, with a green **START** and a red **END** marker.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
pip install folium selenium                 # one-off; PNG also needs firefox + geckodriver
python3 analysis/map_fixposition.py fixposition_bag_wgv_001 fixposition_bag_wgv_002
```

All bags passed in one run **share a single color scale** (the fastest bag sets
the top of the legend) so the maps are directly comparable. Pass
`--per-bag-scale` to give each map its own range, or `--no-png` to skip the
screenshot step.

## Troubleshooting

### Sensor IP changed (ENETUNREACH)

The sensor's IP is assigned by DHCP and changes when the network changes. Symptoms: `Network is unreachable (101, ENETUNREACH)` in the driver log.

**Best fix:** Use `./run_driver.sh` which auto-resolves the sensor IP every launch.

**Manual fix:**

```bash
# 1. Find your subnet
ip -4 addr show wlo1 | grep inet

# 2. Scan for the sensor (replace 10.x.x with your subnet)
for h in $(seq 1 254); do
  (timeout 1 bash -c "echo > /dev/tcp/10.x.x.$h/21000" 2>/dev/null && echo "SENSOR at 10.x.x.$h") &
done; wait

# 3. Verify data
nc <sensor-ip> 21000

# 4. Update stream: in BOTH config files
#    fixposition_driver_ros2/launch/config.yaml
#    install/fixposition_driver_ros2/share/fixposition_driver_ros2/launch/config.yaml
```

### Topics disappear after changing WiFi

```bash
ros2 daemon stop && ros2 daemon start && ros2 topic list
```

If `FASTRTPS_DEFAULT_PROFILES_FILE` is set in your shell, unset it — an interface whitelist breaks discovery on network changes.

### No data on topics

1. Check the sensor web UI I/O config — the required messages must be enabled on TCP0
2. Verify with `nc <sensor-ip> 21000` that you see `$FP,...` lines
3. Make sure you use `--qos-reliability best_effort` when echoing

### No camera image

The driver log tells you which stage failed:

- `Camera: could not open GStreamer pipeline` → the H.264 decoder is missing.
  Install it: `sudo apt install gstreamer1.0-libav gstreamer1.0-plugins-bad`
  (verify with `gst-inspect-1.0 avdec_h264`).
- `Camera: failed to read frame, reconnecting` → the pipeline opened but no packets
  arrive. The sensor must **unicast** the stream to this host's IP (see
  [Camera setup](#camera-optional-disabled-by-default)). Note the host IP is DHCP-assigned
  and can change on reboot — update the sensor's *Destination* to match.
- No `/fixposition/camera/*` topics at all → `camera.enabled` is `false`, or the driver
  isn't running (topics only exist while it runs).
- Multicast (the sensor's default) often doesn't reach the host through switches doing
  IGMP snooping — prefer `Unicast`.

## Documentation

- [Fixposition Docs: ROS Driver](https://docs.fixposition.com/fd/fixposition-ros-driver)
- [Fixposition Docs: I/O Messages](https://docs.fixposition.com/fd/i-o-messages)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
