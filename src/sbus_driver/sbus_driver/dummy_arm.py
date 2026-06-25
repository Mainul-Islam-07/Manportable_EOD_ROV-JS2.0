#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sbus_interfaces.msg import SbusControl

HOME_Y   = -0.10
TARGET_Y =  0.30
STEP     =  0.002

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self._pub = self.create_publisher(SbusControl, '/sbus/control', 10)
        self.y = HOME_Y
        self.going_forward = True
        self.create_timer(0.1, self._tick)

    def _tick(self):
        if self.going_forward:
            self.y += STEP
            if self.y >= TARGET_Y:
                self.y = TARGET_Y
                self.going_forward = False
        else:
            self.y -= STEP
            if self.y <= HOME_Y:
                self.y = HOME_Y
                self.going_forward = True

        msg = SbusControl()
        msg.control_mode = 1
        msg.operation_mode = 0
        msg.arm_x = 0.0
        msg.arm_y = round(self.y, 3)
        msg.arm_z = 0.05
        self._pub.publish(msg)
        self.get_logger().info(f'arm_y={msg.arm_y:.3f} {"→" if self.going_forward else "←"}')

def main():
    rclpy.init()
    rclpy.spin(ArmMover())

if __name__ == '__main__':
    main()