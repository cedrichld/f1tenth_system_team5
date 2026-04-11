#!/usr/bin/env python3
"""
Simple WASD teleop for f1tenth_gym_ros.

Controls:
  w         bump speed up by 0.5 m/s and re-center steering
  s         stop (speed = 0)
  a / d     steer left / right (0.08 rad per tap, persists)
  c         re-center steering
  space     stop and re-center
  q / Ctrl-C  quit

Publishes AckermannDriveStamped to /drive at 20 Hz.
"""
import sys, tty, termios, select, threading
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


SPEED_STEP = 0.5     # m/s per w press
STEER_STEP = 0.08    # rad per a/d press
STEER_LIMIT = 0.4189
SPEED_LIMIT = 8.0


class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.speed = 0.0
        self.steer = 0.0
        self.timer = self.create_timer(0.05, self.publish_cmd)

    def publish_cmd(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steer)
        self.pub.publish(msg)

    def on_key(self, k):
        if k == 'w':
            self.speed = min(SPEED_LIMIT, self.speed + SPEED_STEP)
            self.steer = 0.0  # re-center on accelerate
        elif k == 's':
            self.speed = 0.0
        elif k == 'a':
            self.steer = min(STEER_LIMIT, self.steer + STEER_STEP)
        elif k == 'd':
            self.steer = max(-STEER_LIMIT, self.steer - STEER_STEP)
        elif k == 'c':
            self.steer = 0.0
        elif k == ' ':
            self.speed = 0.0
            self.steer = 0.0
        elif k in ('q', '\x03'):
            return False
        return True


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = TeleopWASD()
    print(__doc__)
    print(f"speed: {node.speed:.1f}  steer: {node.steer:.3f}")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            k = read_key()
            if k is None:
                continue
            if not node.on_key(k):
                break
            print(f"\rspeed: {node.speed:+.2f}  steer: {node.steer:+.3f}  ", end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.speed = 0.0
        node.steer = 0.0
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
