#!/usr/bin/env python3
""" Follows the right hand wall in the maze.

The turn rate comes from two rays that hit the same wall.
So the lengths give the angle of the wall. From this we can get
the distance the robot will have after driving forward a short way.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class MazeRunner(Node):
    def __init__(self):
        super().__init__("mazerunner")

        # This is everything the behaviour depends on.
        self.declare_parameter("wall_distance", 0.45)
        self.declare_parameter("lookahead", 0.35)
        self.declare_parameter("speed", 0.25)
        self.declare_parameter("gain", 1.6)
        self.declare_parameter("max_turn", 1.2)
        self.declare_parameter("front_stop", 0.55)
        self.declare_parameter("sector", 8.0)

        self.running = False
        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        self.create_service(Trigger, "toggle_pause", self.on_toggle)
        self.get_logger().info('Paused. Call /toggle_pause to start.')

    def p(self, name):
        return self.get_parameter(name).value

    def on_toggle(self, request, response):
        self.running = not self.running
        response.success = True
        response.message = 'running' if self.running else 'paused'
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def index(scan, angle):
        """ Angle to index. A 120 degree scan has no ray at -90"""
        i = int(round((angle - scan.angle_min) / scan.angle_increment))
        return max(0, min(len(scan.ranges) - 1, i))

    def look(self, scan, angle):
        """Shortest good return in a small sector so that no bad ray can steer."""
        half = math.radians(self.p("sector"))
        lo = self.index(scan, angle - half)
        hi = self.index(scan, angle + half)
        good = [r for r in scan.ranges[lo:hi + 1]
                if scan.range_min < r < scan.range_max]
        return min(good) if good else scan.range_max

    def on_scan(self, scan):
        twist = Twist()
        if self.running:
            theta = math.radians(45.0)
            b = self.look(scan, -math.pi / 2)          # out to the right
            a = self.look(scan, -math.pi / 2 + theta)  # forward right
            front = self.look(scan, 0.0)

            # The two ray lengths form a triangle with the wall.
            num = a * math.cos(theta) - b
            den = a * math.sin(theta)
            hyp = math.hypot(num, den)
            sin_a, cos_a = num / hyp, den / hyp

            # Where the wall will be after driving forward a bit.
            ahead = b * cos_a + self.p("lookahead") * sin_a
            turn = self.p("gain") * (self.p("wall_distance") - ahead)

            limit = self.p("max_turn")
            stop = self.p("front_stop")
            if front < stop:
                # A corner. Commit to the turn, because the side reading
                # still looks fine right up until the robot hits the wall.
                turn = limit
            twist.angular.z = max(-limit, min(limit, turn))

            # Ease off as the front comes nearer.The robot drives along the curve of the wall. 
            twist.linear.x = self.p("speed") * min(1.0, front / stop)

        # Paused means a zero Twist, not silence. The diff drive plugin keeps
        # the last command it was given.
        self.cmd.publish(twist)


def main():
    rclpy.init()
    node = MazeRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
