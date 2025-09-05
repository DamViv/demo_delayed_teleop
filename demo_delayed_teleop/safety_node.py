#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import subprocess

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.override = False

        # Publishers
        self.pub_user = self.create_publisher(Twist, '/cmd_vel_user', 10) 
        self.pub_direct = self.create_publisher(Twist, '/fleet_0/cmd_vel', 10)      # real robot command topic

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_out', self.delayed_callback, 10)  # delayed command topic (from delay_node)
        self.create_subscription(Joy, '/joy_user', self.user_callback, 10)          # user joystick command topic
        self.create_subscription(Joy, '/joy_safety', self.safety_callback, 10)      # safety joystick command topic

        self.get_logger().info("SafetyNode ready")

    def user_callback(self, msg):
        if not self.override:
            twist = Twist()
            twist.header.stamp = self.get_clock().now().to_msg()
            if len(msg.axes) >= 2:
                twist.twist.linear.x = msg.axes[1]
                twist.twist.angular.z = msg.axes[0]
            self.pub_user.publish(twist)

    def delayed_callback(self, msg):
        if not self.override:
            self.pub_direct.publish(msg)

    def safety_callback(self, msg):
        if msg.buttons[0] == 1:
            self.override = True
            self.get_logger().warn("Safety override activated")
        elif msg.buttons[1] == 1:
            self.override = False
            self.get_logger().info("Control returned to user")
        elif msg.buttons[2] == 1:
            self.set_delay(1.25)
            self.get_logger().info("Moon mode: 1.25s mean delay")
        elif msg.buttons[3] == 1:
            self.set_delay(0.0)
            self.get_logger().info("Earth mode: no delay")
        elif msg.buttons[4] == 1:
            self.set_delay(2.5)
            self.get_logger().info("Artificial: 2.5s delay")

        if self.override and len(msg.axes) >= 2:
            twist = Twist()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x = msg.axes[1]
            twist.twist.angular.z = msg.axes[0]
            self.pub_direct.publish(twist)

    def set_delay(self, value):
        try:
            subprocess.run([
                'ros2', 'param', 'set', '/delay_node', 'delay_sec', str(value)
            ], check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to set delay: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
