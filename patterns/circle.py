#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircularMotion(Node):
    def __init__(self, namespace, radius, linear_speed):
        super().__init__('circular_motion_' + namespace)
        self.robot1_publisher = self.create_publisher(Twist, f'/bot_1/cmd_vel', 10)
        self.robot2_publisher = self.create_publisher(Twist, f'/bot_2/cmd_vel', 10)
        self.robot3_publisher = self.create_publisher(Twist, f'/bot_3/cmd_vel', 10)
        self.radius = radius
        self.linear_speed = linear_speed
        self.angular_speed = -linear_speed / radius
        self.timer = self.create_timer(0.1, self.move_in_circle)

    def move_in_circle(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
       
        self.robot1_publisher.publish(twist)
        self.robot2_publisher.publish(twist)
        self.robot3_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    # Define robots with different namespaces
    motion = CircularMotion('bot_1', radius=3.0, linear_speed=0.5)
    try:
        rclpy.spin(motion)
    except KeyboardInterrupt:
        pass

    motion.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
