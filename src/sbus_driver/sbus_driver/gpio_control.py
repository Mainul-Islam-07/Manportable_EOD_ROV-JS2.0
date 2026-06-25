#!/usr/bin/env python3
"""
Combined GPIO controller node.

  SBUS light_state (uint8 0/1) ──► GPIO 17 + GPIO 27   (lights)
  TCP "FIRE" command           ──► GPIO 22             (fire signal)

A single process owns every pin so nothing fights for the GPIO line.
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from gpiozero import LED

from sbus_interfaces.msg import SbusControl


# --- Pin assignments (BCM) ---
LIGHT_PIN_A = 22
LIGHT_PIN_B = 27
FIRE_PIN    = 17

# --- TCP server config ---
LISTEN_HOST  = "0.0.0.0"
LISTEN_PORT  = 5005
RECV_TIMEOUT = 5      # seconds to wait for "FIRE" after connect
HOLD_TIMEOUT = 10     # safety cap on how long we hold the fire line


class GpioControlNode(Node):
    def __init__(self):
        super().__init__("gpio_control_node")

        # ---- GPIO setup ----
        self.light_a = LED(LIGHT_PIN_A)
        self.light_b = LED(LIGHT_PIN_B)
        self.fire    = LED(FIRE_PIN)
        self.light_a.off()
        self.light_b.off()
        self.fire.off()
        self._last_light = 0

        # ---- ROS2 subscription (lights) ----
        self.sub = self.create_subscription(
            SbusControl,
            "/sbus/control",
            self._sbus_cb,
            10,
        )

        # ---- TCP server thread (fire) ----
        self._tcp_stop = threading.Event()
        self._tcp_thread = threading.Thread(target=self._tcp_serve, daemon=True)
        self._tcp_thread.start()

        self.get_logger().info(
            f"GPIO controller ready | lights: GPIO {LIGHT_PIN_A}+{LIGHT_PIN_B} | "
            f"fire: GPIO {FIRE_PIN} | TCP {LISTEN_HOST}:{LISTEN_PORT}"
        )

    # ---------------- SBUS / lights ----------------
    def _sbus_cb(self, msg: SbusControl):
        state = int(msg.light_state)
        if state == self._last_light:
            return
        if state == 1:
            self.light_a.on()
            self.light_b.on()
            self.get_logger().info("Lights ON")
        else:
            self.light_a.off()
            self.light_b.off()
            self.get_logger().info("Lights OFF")
        self._last_light = state

    # ---------------- TCP / fire ----------------
    def _tcp_serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((LISTEN_HOST, LISTEN_PORT))
        except OSError as e:
            self.get_logger().error(f"TCP bind failed: {e}")
            return
        server.listen(1)
        server.settimeout(1.0)  # wake up periodically to check stop flag

        while not self._tcp_stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"accept error: {e}")
                continue
            self.get_logger().info(f"TCP connection from {addr[0]}:{addr[1]}")
            self._handle_client(conn, addr)

        try:
            server.close()
        except Exception:
            pass

    def _handle_client(self, conn, addr):
        armed = False
        try:
            conn.settimeout(RECV_TIMEOUT)
            f = conn.makefile("rwb", buffering=0)
            line = f.readline().decode("utf-8", errors="replace").strip()

            if line != "FIRE":
                self.get_logger().warn(f"  rejected from {addr}: bad command {line!r}")
                f.write(b"FAIL\n")
                return

            self.fire.on()
            armed = True
            self.get_logger().info(f"FIRING from {addr[0]}")
            f.write(b"OK\n")

            # Hold connection until client closes (or safety cap fires)
            conn.settimeout(HOLD_TIMEOUT)
            try:
                while True:
                    data = conn.recv(64)
                    if not data:
                        break  # client closed → stop firing
            except socket.timeout:
                pass
        except Exception as e:
            self.get_logger().warn(f"  error from {addr}: {e}")
        finally:
            if armed:
                self.fire.off()
                self.get_logger().info("STOPPED")
            try:
                conn.close()
            except Exception:
                pass

    # ---------------- shutdown ----------------
    def destroy_node(self):
        self._tcp_stop.set()
        try:
            self._tcp_thread.join(timeout=2.0)
        except Exception:
            pass
        for led in (self.light_a, self.light_b, self.fire):
            try:
                led.off()
                led.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GpioControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()