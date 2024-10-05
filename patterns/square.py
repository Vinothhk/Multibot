#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SquareMotion(Node):
    def __init__(self, namespace, side_length, linear_speed, angular_speed):
        super().__init__('square_motion_' + namespace)

        # Publisher for each robot's cmd_vel topic
        self.publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)

        # Parameters for square motion
        self.side_length = side_length
        self.linear_speed = linear_speed
        self.angular_speed = -angular_speed
        self.move_straight_time = side_length / linear_speed  # Time to move forward one side
        self.turn_time = math.pi / 2 / angular_speed  # Time to turn 90 degrees

        self.state = 'move_straight'  # Current motion state ('move_straight' or 'turn')
        self.start_time = self.get_clock().now()  # To keep track of the time spent in each state
        self.sides_completed = 0  # Track the number of sides completed in the square

        # Timer to continuously update movement
        self.timer = self.create_timer(0.1, self.move_in_square)

    def move_in_square(self):
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds

        twist = Twist()

        # Move forward in a straight line
        if self.state == 'move_straight':
            twist.linear.x = self.linear_speed
            self.publisher.publish(twist)

            # Switch to turning after covering one side
            if elapsed_time > self.move_straight_time:
                self.state = 'turn'
                self.start_time = self.get_clock().now()  # Reset time for turning

        # Turn 90 degrees
        elif self.state == 'turn':
            twist.angular.z = self.angular_speed
            self.publisher.publish(twist)

            # Switch back to moving straight after completing the turn
            if elapsed_time > self.turn_time:
                self.state = 'move_straight'
                self.sides_completed += 1
                self.start_time = self.get_clock().now()  # Reset time for moving straight

        # Reset the pattern after completing the square (4 sides)
        if self.sides_completed == 4:
            self.sides_completed = 0

def main(args=None):
    rclpy.init(args=args)

    # Define side length and speed
    side_length = 6.0
    linear_speed = 0.8
    angular_speed = 0.5

    # Create nodes for each robot's square motion
    motion_bot1 = SquareMotion('bot_1', side_length, linear_speed, angular_speed)
    motion_bot2 = SquareMotion('bot_2', side_length, linear_speed, angular_speed)
    motion_bot3 = SquareMotion('bot_3', side_length, linear_speed, angular_speed)

    # Spin each node separately
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(motion_bot1)
    executor.add_node(motion_bot2)
    executor.add_node(motion_bot3)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Cleanup
    motion_bot1.destroy_node()
    motion_bot2.destroy_node()
    motion_bot3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
