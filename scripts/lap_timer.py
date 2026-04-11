#!/usr/bin/env python3
"""
Simple lap timer for f1tenth_gym_ros sim.

Uses the first received odom pose as the start/finish line reference.
A lap is counted when:
  1. The car leaves a minimum distance (LEAVE_RADIUS) from start, then
  2. The car comes back within RETURN_RADIUS of start

Prints lap number, lap time, and running best.

Usage:
    python3 lap_timer.py
    (Ctrl-C to stop)
"""
import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


LEAVE_RADIUS = 3.0    # m — must go at least this far before a lap can complete
RETURN_RADIUS = 0.8   # m — must come within this of start to trigger lap end
MIN_LAP_TIME = 1.0    # s — filter out bogus short "laps"


class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')
        self.sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.on_odom, 10)
        self.start_x = None
        self.start_y = None
        self.state = 'waiting_to_leave'  # → 'waiting_to_return'
        self.lap_start_time = None
        self.lap_num = 0
        self.best_lap = float('inf')
        self.laps = []
        self.lap_max_v = 0.0
        self.lap_min_v = float('inf')

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.hypot(vx, vy)
        if v > self.lap_max_v:
            self.lap_max_v = v
        if v < self.lap_min_v:
            self.lap_min_v = v
        now = time.time()

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.lap_start_time = now
            print(f"\n[lap_timer] Start point set: ({x:.2f}, {y:.2f})")
            print(f"[lap_timer] Drive at least {LEAVE_RADIUS}m away to begin lap timing\n")
            return

        d = math.hypot(x - self.start_x, y - self.start_y)

        if self.state == 'waiting_to_leave':
            if d > LEAVE_RADIUS:
                self.state = 'waiting_to_return'

        elif self.state == 'waiting_to_return':
            if d < RETURN_RADIUS:
                lap_time = now - self.lap_start_time
                if lap_time < MIN_LAP_TIME:
                    return  # ignore spurious
                self.lap_num += 1
                v_max = self.lap_max_v
                v_min = self.lap_min_v if self.lap_min_v != float('inf') else 0.0
                if self.lap_num == 1:
                    # First lap is warmup (car starts from rest, misaligned start point)
                    print(f"[Warmup lap]  time: {lap_time:6.3f}s   v_max: {v_max:4.2f}   v_min: {v_min:4.2f} m/s   (not counted)")
                else:
                    counted_num = self.lap_num - 1
                    self.laps.append(lap_time)
                    if lap_time < self.best_lap:
                        self.best_lap = lap_time
                        tag = '  ← NEW BEST'
                    else:
                        tag = ''
                    avg = sum(self.laps) / len(self.laps)
                    print(f"[lap {counted_num}]  time: {lap_time:6.3f}s   v_max: {v_max:4.2f}   v_min: {v_min:4.2f} m/s   best: {self.best_lap:6.3f}s   avg: {avg:6.3f}s{tag}")
                self.lap_start_time = now
                self.lap_max_v = 0.0
                self.lap_min_v = float('inf')
                self.state = 'waiting_to_leave'


def main():
    rclpy.init()
    node = LapTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.laps:
            print(f"\n[lap_timer] Finished: {len(node.laps)} laps, best {node.best_lap:.3f}s")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
