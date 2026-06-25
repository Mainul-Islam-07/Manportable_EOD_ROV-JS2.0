#!/usr/bin/env python3
"""
light_control_node.py  (ROS 2 Jazzy)

Subscribes to /sbus/control (SbusControl) and drives the two light GPIOs
on a Raspberry Pi 5 according to msg.light_state (0=off, 1=on).

Light GPIOs: BCM 27 and BCM 22.

Pi 5 note: the RP1 I/O controller is NOT supported by legacy RPi.GPIO.
Use gpiozero with the lgpio backend, standard on Pi 5 / Ubuntu 24.04.

    sudo apt install python3-gpiozero python3-lgpio
"""

import rclpy
from rclpy.node import Node

from sbus_interfaces.msg import SbusControl   # <-- change to your actual package name

from gpiozero import LED, Device
from gpiozero.pins.lgpio import LGPIOFactory

# Force the lgpio backend (required for Pi 5)
Device.pin_factory = LGPIOFactory()

LIGHT_GPIOS = (27, 22)


class LightControlNode(Node):
    def __init__(self):
        super().__init__("light_control_node")

        self.lights = [LED(pin) for pin in LIGHT_GPIOS]
        self._last_state = None

        self.sub = self.create_subscription(
            SbusControl, "/sbus/control", self.cb, 10
        )

        self._apply(0)  # start with lights off
        self.get_logger().info(f"light_control_node ready on GPIO {LIGHT_GPIOS}")

    def cb(self, msg):
        self._apply(msg.light_state)

    def _apply(self, state):
        state = 1 if state else 0
        if state == self._last_state:
            return
        for led in self.lights:
            led.on() if state else led.off()
        self._last_state = state
        self.get_logger().info(f"Lights {'ON' if state else 'OFF'}")

    def destroy_node(self):
        for led in self.lights:
            led.off()
            led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LightControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()