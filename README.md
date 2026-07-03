# Fixposition Vision-RTK 2 ROS2 Driver

ROS2 driver for the [Fixposition Vision-RTK 2](https://www.fixposition.com/product) sensor on **Linux** (ROS 2 Humble).

Forked from [fixposition/fixposition\_driver](https://github.com/fixposition/fixposition_driver) with added launch tooling for automatic sensor discovery over WiFi and Ethernet.

> [!IMPORTANT]
> Tags 8.0.0 onward break compatibility with previous versions. See the [Migration Guide](https://docs.fixposition.com/fd/migration-guide).

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Fixposition Vision-RTK 2 sensor reachable on the network (WiFi or Ethernet)
- Sensor TCP0 port (21000) configured with the required output messages via the sensor web UI

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
  /fixposition/poiimu /fixposition/speed /fixposition/ypr /rtcm
```

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

## Documentation

- [Fixposition Docs: ROS Driver](https://docs.fixposition.com/fd/fixposition-ros-driver)
- [Fixposition Docs: I/O Messages](https://docs.fixposition.com/fd/i-o-messages)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
