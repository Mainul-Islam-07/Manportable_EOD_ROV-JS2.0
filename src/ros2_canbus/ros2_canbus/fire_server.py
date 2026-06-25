#!/usr/bin/env python3
"""
fire_server.py  --  ROS 2 fire-control + presence server for Raspberry Pi 5.

Two TCP servers run inside one rclpy node:

  * FIRE server  (port 5005): receives "FIRE <hold_ms>\n" from the ARMSWITCH
    ("JS2 FCS") Android app and drives GPIO17 for that duration. Unchanged
    protocol from the original one-shot connect/send/close design.

  * HEARTBEAT server (port 5006): the app holds a persistent connection here
    and sends "PING\n" once per second while it is open/foregrounded. When
    pings stop for longer than HEARTBEAT_TIMEOUT, the app is considered closed.

ROS 2:
    Publishes std_msgs/Int8 on  /fire_mode  at 5 Hz:
        1  -> app is open  (recent heartbeat)
        0  -> app is closed (heartbeat timed out, or never connected)

    Safety default is 0: if the Pi cannot tell, it publishes 0.

The legacy RPi.GPIO library does NOT work on the Pi 5 (RP1 I/O controller).
This uses gpiozero with the lgpio backend.

Install (Pi 5, Bookworm):
    sudo apt update
    sudo apt install -y python3-gpiozero python3-lgpio
    # plus a ROS 2 distro (e.g. Humble/Jazzy) with rclpy + std_msgs

Run (inside a sourced ROS 2 environment):
    python3 fire_server.py
"""

import socket
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import UInt8

from gpiozero import LED  # simple on/off digital output; perfect for a relay/trigger line

# ---------------- CONFIG ----------------
HOST = "0.0.0.0"      # listen on all interfaces (Pi will be 192.168.0.100)
FIRE_PORT = 5005      # must match the Port field in the Android app
HEARTBEAT_PORT = 5006 # app's persistent presence connection

GPIO_PIN = 17         # BCM GPIO17  (physical pin 11)
ACTIVE_HIGH = True    # True: pin goes HIGH to fire. False for active-low relay boards.
MIN_HOLD_MS = 10
MAX_HOLD_MS = 5000
FIRE_RECV_TIMEOUT = 5.0   # seconds to wait for a fire command before dropping a client

# presence / heartbeat
HEARTBEAT_INTERVAL = 1.0  # app sends a PING this often (informational, app side)
HEARTBEAT_TIMEOUT  = 3.0  # no PING for this long -> app considered closed (3x interval)
PUBLISH_PERIOD     = 0.2  # 5 Hz publish on /fire_mode
# ----------------------------------------

# initial_value=False => starts de-asserted (safe) regardless of ACTIVE_HIGH
fire_pin = LED(GPIO_PIN, active_high=ACTIVE_HIGH, initial_value=False)

# Only allow one physical fire at a time across all connections.
fire_lock = threading.Lock()


class FireModeNode(Node):
    def __init__(self):
        super().__init__("fire_mode_node")

        # Keep the latest value available to late-joining subscribers.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(UInt8, "/fire_mode", qos)

        # Presence state, guarded by a lock since the heartbeat thread writes it.
        self._presence_lock = threading.Lock()
        self._last_ping = 0.0          # monotonic time of last PING seen
        self._ever_pinged = False

        # 5 Hz publisher.
        self.create_timer(PUBLISH_PERIOD, self._publish_fire_mode)

        self.get_logger().info(
            f"fire_mode_node up. FIRE:{FIRE_PORT} HEARTBEAT:{HEARTBEAT_PORT} "
            f"GPIO{GPIO_PIN} active_{'high' if ACTIVE_HIGH else 'low'} | "
            f"publishing /fire_mode at {1.0/PUBLISH_PERIOD:.0f} Hz, "
            f"timeout {HEARTBEAT_TIMEOUT:.1f}s"
        )

    # ---- presence ----
    def note_ping(self):
        with self._presence_lock:
            self._last_ping = time.monotonic()
            self._ever_pinged = True

    def is_app_open(self):
        with self._presence_lock:
            if not self._ever_pinged:
                return False
            return (time.monotonic() - self._last_ping) <= HEARTBEAT_TIMEOUT

    def _publish_fire_mode(self):
        self.pub.publish(UInt8(data=1 if self.is_app_open() else 0))


def log_info(node, msg):
    node.get_logger().info(msg)


# ---------------- FIRE handling ----------------

def do_fire(node, hold_ms):
    """Drive GPIO asserted for hold_ms, then de-assert. Blocking."""
    hold_s = hold_ms / 1000.0
    with fire_lock:
        log_info(node, f"FIRE -> GPIO{GPIO_PIN} HIGH for {hold_ms} ms")
        fire_pin.on()
        try:
            time.sleep(hold_s)
        finally:
            fire_pin.off()
        log_info(node, f"GPIO{GPIO_PIN} LOW (fire complete)")


def parse_command(line):
    """
    Returns (hold_ms, None) on success or (None, error_string) on failure.
    Accepts:  'FIRE'        -> default 500 ms
              'FIRE <ms>'   -> clamped to [MIN_HOLD_MS, MAX_HOLD_MS]
    """
    parts = line.strip().split()
    if not parts:
        return None, "empty"
    if parts[0].upper() != "FIRE":
        return None, f"unknown command '{parts[0]}'"
    if len(parts) == 1:
        return 500, None
    try:
        hold_ms = int(parts[1])
    except ValueError:
        return None, "hold time not an integer"
    if hold_ms < MIN_HOLD_MS:
        hold_ms = MIN_HOLD_MS
    elif hold_ms > MAX_HOLD_MS:
        hold_ms = MAX_HOLD_MS
    return hold_ms, None


def handle_fire_client(node, conn, addr):
    conn.settimeout(FIRE_RECV_TIMEOUT)
    peer = f"{addr[0]}:{addr[1]}"
    log_info(node, f"[FIRE] connected: {peer}")
    try:
        buf = b""
        while b"\n" not in buf:
            chunk = conn.recv(64)
            if not chunk:
                log_info(node, f"[FIRE] {peer} closed before sending a command")
                return
            buf += chunk
            if len(buf) > 256:  # sanity cap against junk
                conn.sendall(b"ERR too long\n")
                return

        line = buf.split(b"\n", 1)[0].decode("utf-8", errors="replace")
        log_info(node, f"[FIRE] {peer} sent: {line!r}")

        hold_ms, err = parse_command(line)
        if err is not None:
            conn.sendall(f"ERR {err}\n".encode("utf-8"))
            return

        # Acknowledge first; the app waits for 'OK' before its own hold loop.
        conn.sendall(b"OK\n")
        do_fire(node, hold_ms)

    except socket.timeout:
        log_info(node, f"[FIRE] {peer} timed out")
        try:
            conn.sendall(b"ERR timeout\n")
        except OSError:
            pass
    except Exception as e:
        log_info(node, f"[FIRE] {peer} error: {e}")
        try:
            conn.sendall(f"ERR {e}\n".encode("utf-8"))
        except OSError:
            pass
    finally:
        try:
            conn.close()
        except OSError:
            pass
        log_info(node, f"[FIRE] disconnected: {peer}")


# ---------------- HEARTBEAT handling ----------------

def handle_heartbeat_client(node, conn, addr):
    """
    Persistent presence connection. The app sends 'PING\n' ~every second.
    Each PING refreshes the presence timestamp. We reply 'PONG\n' so the app
    can detect a dead link too. When the socket drops or pings stop, presence
    naturally times out in the node.
    """
    peer = f"{addr[0]}:{addr[1]}"
    log_info(node, f"[HB] connected: {peer}")
    # Slightly longer than the timeout so a single late ping doesn't kill the socket.
    conn.settimeout(HEARTBEAT_TIMEOUT + 2.0)
    try:
        buf = b""
        while True:
            chunk = conn.recv(64)
            if not chunk:
                log_info(node, f"[HB] {peer} closed connection")
                return
            buf += chunk
            if len(buf) > 1024:
                buf = buf[-1024:]  # never let junk grow unbounded
            # Process any complete lines.
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                token = line.strip().upper()
                if token == b"PING":
                    node.note_ping()
                    try:
                        conn.sendall(b"PONG\n")
                    except OSError:
                        return
                # ignore anything else silently
    except socket.timeout:
        log_info(node, f"[HB] {peer} timed out (no pings)")
    except Exception as e:
        log_info(node, f"[HB] {peer} error: {e}")
    finally:
        try:
            conn.close()
        except OSError:
            pass
        log_info(node, f"[HB] disconnected: {peer}")


# ---------------- server accept loops ----------------

def accept_loop(node, port, handler, stop_evt):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, port))
    srv.listen(5)
    srv.settimeout(1.0)  # so we can check stop_evt periodically
    while not stop_evt.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        t = threading.Thread(target=handler, args=(node, conn, addr), daemon=True)
        t.start()
    srv.close()


def main():
    rclpy.init()
    node = FireModeNode()
    stop_evt = threading.Event()

    fire_thread = threading.Thread(
        target=accept_loop, args=(node, FIRE_PORT, handle_fire_client, stop_evt),
        daemon=True,
    )
    hb_thread = threading.Thread(
        target=accept_loop, args=(node, HEARTBEAT_PORT, handle_heartbeat_client, stop_evt),
        daemon=True,
    )
    fire_thread.start()
    hb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("shutting down, forcing GPIO LOW")
        stop_evt.set()
        fire_pin.off()
        try:
            fire_pin.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
