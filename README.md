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

## Topic Rates

| Topics | Rate | Source |
|---|---|---|
| `/fixposition/odometry_*`, `/fixposition/ypr`, `/fixposition/poiimu` | ~10 Hz | Fusion output (configurable in sensor web UI) |
| `/fixposition/fpa/rawimu`, `/fixposition/fpa/corrimu` | ~200 Hz | Raw IMU data |

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
