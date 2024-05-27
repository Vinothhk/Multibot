#! /usr/bin/env python3
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import math
from nav_msgs.msg import Odometry
class NavToPoseActionClient(Node):
    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self._bot_2_action_client = ActionClient(self, NavigateToPose, '/bot_2/navigate_to_pose')
        self._bot_3_action_client = ActionClient(self, NavigateToPose, '/bot_3/navigate_to_pose')
        self.subscription_leader = self.create_subscription(Odometry, 'bot_1/odom', self.leader_pose_callback, 10)
        self.subscription_robot2 = self.create_subscription(Odometry, 'bot_2/odom', self.robot2_pose_callback, 10)
        self.leader_pose = Odometry()
        self.robot2_pose = Odometry()
        self.robot3_pose = Odometry()
        
        
    def get_yaw_from_quaternion(self, orientation):
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw 
    def leader_pose_callback(self, msg):
        self.leader_pose=msg
        yaw_goal = self.get_yaw_from_quaternion(self.leader_pose.pose.pose.orientation)
        x_goal = self.leader_pose.pose.pose.position.x - (0.75*math.cos(yaw_goal))
        y_goal = self.leader_pose.pose.pose.position.y - (0.75*math.sin(yaw_goal))
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x_goal
        goal_pose.pose.pose.position.y = y_goal
        goal_pose.pose.pose.orientation.z = yaw_goal
        self.get_logger().info('waiting for action server')
        self._bot_2_action_client.wait_for_server()
        self.get_logger().info('action server detected')
        self._send_goal_future = self._bot_2_action_client.send_goal_async(goal_pose, feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def robot2_pose_callback(self,msg):
        self.robot2_pose=msg
        yaw_goal = self.get_yaw_from_quaternion(self.robot2_pose.pose.pose.orientation)
        x_goal = self.robot2_pose.pose.pose.position.x - (0.75*math.cos(yaw_goal))
        y_goal = self.robot2_pose.pose.pose.position.y - (0.75*math.sin(yaw_goal))
        
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x_goal
        goal_pose.pose.pose.position.y = y_goal
        goal_pose.pose.pose.orientation.z = yaw_goal
        self.get_logger().info('waiting for action server')
        self._bot_3_action_client.wait_for_server()
        self.get_logger().info('action server detected')
        self._send_goal_future = self._bot_3_action_client.send_goal_async(goal_pose, feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result:' + str(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

        
def main(args=None):
    rclpy.init(args=args)
    swarm_control = NavToPoseActionClient()
    rclpy.spin(swarm_control)
    
if __name__ == '__main__':
    main()