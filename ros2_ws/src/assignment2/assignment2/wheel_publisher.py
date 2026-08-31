"""
Task 4 Here we add wheel motion
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class WheelPublisher(Node):
    """Publishes an increasing angle for both drive wheels"""
    JOINT_NAMES = ['left_wheel_joint', 'right_wheel_joint']
    PUBLISH_PERIOD = 0.05
    WHEEL_SPEED = 2.0

    def __init__(self):
        """Set up the publisher and timer"""
        super().__init__("wheel_publisher")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(self.PUBLISH_PERIOD, self.publish_joint_state)
        self.angle = 0.0
        self.get_logger().info("Publishing wheel joint states on /joint_states")

    def publish_joint_state(self):
        """Advance the wheel angle and publish it to /joint_sates
        Both wheels have same angle, so they turn in the same direction"""
        self.angle = (self.angle + self.WHEEL_SPEED * self.PUBLISH_PERIOD) % (2 * math.pi)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = [self.angle, self.angle]
        self.publisher.publish(msg)


def main(args=None):
    """Run the node"""
    rclpy.init(args=args)
    node = WheelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
