# Avatar Dashboard — MK32 telemetry viewer

Two pieces:

```
bridge/telemetry_udp_bridge.py   ROS 2 node, Pi 5  → sends UDP/JSON to the MK32
AvatarDashboard/                 Android (Kotlin/Compose) app shown on the MK32
```

The Pi forwards `/motor_diagnostics`, `/joint_states`, `/arm_active_motors`
and `/drive_active_motors` as one compact JSON datagram (~10 Hz). The app
listens on UDP **:9870** and shows three tabbed windows tuned for the small
MK32 screen:

1. **Diagnostics** – one color card per motor (green OK / amber WARN /
   red FAULT / grey STALE). Tap a card to expand voltage, current, coil &
   board temperature, statusword/flags, error register, error code and
   heartbeat.
2. **Joints** – every joint with its live value (radians + degrees, the
   telescope in metres + mm) and a zero-centred auto-scaling bar.
3. **Motors** – grid of the full arm + drive roster; lit = alive, dim =
   dropped, with an `online / total` count per bus.

A persistent top strip shows link state (**LIVE / STALE / NO LINK**),
packet age, and a red **FAULT** flag the moment any motor faults.

---

## 1. Run the bridge on the Pi

Set the MK32's fixed IP at the top of `telemetry_udp_bridge.py`
(`DEFAULT_MK32_IP`) or pass it at launch:

```bash
python3 telemetry_udp_bridge.py --ros-args \
    -p mk32_ip:=192.168.0.50 -p mk32_port:=9870 -p rate_hz:=10.0
```

Drop the file into your `ros2_canbus` package (next to
`motor_heartbeat_node.py`) or run it standalone — it only needs `rclpy`,
`sensor_msgs`, and `diagnostic_msgs`, which you already have.

If you renamed any topics in `controller_config.json`, update the
`TOPIC_*` constants near the top of the bridge to match.

## 2. Build the Android app

Requirements: Android Studio (Koala or newer) **or** command-line
Android SDK + JDK 17.

1. Open the `AvatarDashboard/` folder in Android Studio.
2. Copy `local.properties.template` → `local.properties` and set
   `sdk.dir` to your Android SDK path. (Android Studio fills this in
   automatically the first time you open the project.)
3. Let Gradle sync, then **Run** onto the MK32 (or any Android 7.0+ device).

Command line:

```bash
cd AvatarDashboard
# (first time) generate the wrapper jar with a local gradle, OR open once in
# Android Studio which creates it for you, then:
./gradlew assembleDebug
# APK lands in app/build/outputs/apk/debug/app-debug.apk
```

Install the APK on the MK32:

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## 3. Network notes

* Put the Pi and MK32 on the same subnet (you already use 192.168.0.x).
* The app listens on UDP 9870; nothing needs to be opened outbound.
* `usesCleartextTraffic` is enabled — this is a plain LAN UDP link, no TLS.
* The app declares STALE after 600 ms without a packet and NO LINK after
  2.5 s, so a dead bridge is obvious within a couple of seconds.

## Changing the listen port

The port is defined once in `DashboardViewModel.LISTEN_PORT` (app) and
must match `mk32_port` on the bridge.
