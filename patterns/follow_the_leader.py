#! /usr/bin/env python3

# File name: swarm_control.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

class SwarmControl(Node):
    def __init__(self):
        super().__init__('swarm_control')
        # Create publisher for robot_2
        self.publisher_robot2 = self.create_publisher(Twist, '/bot_2/cmd_vel', 10)
        # Create publisher for robot_3
        self.publisher_robot3 = self.create_publisher(Twist, '/bot_3/cmd_vel', 10)
        self.get_logger().info('Publishers are created...')
        # Create subscriptions for the poses of all robots
        self.subscription_leader = self.create_subscription(Odometry, 'bot_1/odom', self.leader_pose_callback, 10)
        self.subscription_robot2 = self.create_subscription(Odometry, 'bot_2/odom', self.robot2_pose_callback, 10)
        self.subscription_robot3 = self.create_subscription(Odometry, 'bot_3/odom', self.robot3_pose_callback, 10)
        self.get_logger().info('Subscribers are created...')
        
        # Initialize variables
        self.leader_pose = Odometry()
        self.robot2_pose = Odometry()
        self.robot3_pose = Odometry()
        self.desired_distance = 0.5  # Desired distance between robots
        
        self.k_align = 0.3
        self.k_cohesion = 0.2
        self.k_separation = 0.5
        
        self.get_logger().info('NODE IS ACTIVE ! ...')
    def leader_pose_callback(self, msg):
        # Update leader robot's pose
        self.get_logger().info('Leader Callback...')
        self.leader_pose = msg

    def robot2_pose_callback(self, msg):
        # Update robot_2's pose
        self.get_logger().info('Robot 1 Callback...')
        self.robot2_pose = msg
        self.move_robot(self.publisher_robot2, self.robot2_pose,self.leader_pose)

    def robot3_pose_callback(self, msg):
        self.get_logger().info('Robot 2 Callback...')
        # Update robot_3's pose
        self.robot3_pose = msg
        self.move_robot(self.publisher_robot3, self.robot3_pose,self.robot2_pose)
        
    def get_yaw_from_quaternion(self, orientation):
        # Convert orientation from Quaternion to Euler
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def move_robot(self, publisher, robot_pose, other_poses):
        vel_msg = Twist()
        self.get_logger().info('Robot is moving...')
        """ # Alignment
        align_vel = Twist()
        align_vel.linear.x += other_poses.pose.pose.position.x
        align_vel.linear.y += other_poses.pose.pose.position.y
        
        # Cohesion
        cohesion_vel = Twist()
        cohesion_vel.linear.x += other_poses.pose.pose.position.x
        cohesion_vel.linear.y += other_poses.pose.pose.position.y


        # Separation
        separation_vel = Twist() """
        linear_distance = math.sqrt((other_poses.pose.pose.position.x - robot_pose.pose.pose.position.x) ** 2 + (other_poses.pose.pose.position.y - robot_pose.pose.pose.position.y) ** 2)
        if linear_distance > self.desired_distance or linear_distance < self.desired_distance:
            angle_to_leader = math.atan2(other_poses.pose.pose.position.y - robot_pose.pose.pose.position.y, other_poses.pose.pose.position.x - robot_pose.pose.pose.position.x)
            follower_yaw = self.get_yaw_from_quaternion(other_poses.pose.pose.orientation)
            angular_difference = angle_to_leader - follower_yaw
            vel_msg.linear.x = linear_distance
            vel_msg.angular.z = 2 * angular_difference
            self.get_logger().info(f" Linear Distance: {linear_distance}, Angular Difference: {angular_difference}")
        
            """ # Total velocity
            msg.linear.x = self.k_align * align_vel.linear.x + self.k_cohesion * cohesion_vel.linear.x + self.k_separation * separation_vel.linear.x
            msg.angular.z = self.k_align * align_vel.linear.y + self.k_cohesion * cohesion_vel.linear.y + self.k_separation * separation_vel.linear.y """
        
            self.get_logger().info('Moving Robot...')
            publisher.publish(vel_msg)
        
        """    self.get_logger().info('Robot pose: %s' % str(robot_pose.pose))
        self.get_logger().info('Alignment velocity: %s' % str(align_vel))
        self.get_logger().info('Cohesion velocity: %s' % str(cohesion_vel))
        self.get_logger().info('Separation velocity: %s' % str(separation_vel))
        self.get_logger().info('Total velocity: %s' % str(msg)) """

def main(args=None):
    rclpy.init(args=args)
    swarm_control = SwarmControl()
    rclpy.spin(swarm_control)
if __name__ == '__main__':
    main()
