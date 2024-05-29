#! /usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import math
from nav_msgs.msg import Odometry

class Followtheleader(Node):
    def __init__(self):
        super().__init__('LeaderFollower')
        self.x1 = 0.0
        self.y1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0
        self.x3 = 0.0
        self.y3 = 0.0 
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.yaw3 = 0.0   
        self._bot_1_action_client = ActionClient(self,NavigateToPose, '/bot_1/navigate_to_pose')
        self._bot_2_action_client = ActionClient(self, NavigateToPose, '/bot_2/navigate_to_pose')
        self._bot_3_action_client = ActionClient(self, NavigateToPose, '/bot_3/navigate_to_pose')
        self.subscription_robot1 = self.create_subscription(Odometry, 'bot_1/odom', self.robot1_pose_callback, 10)
        self.subscription_robot2 = self.create_subscription(Odometry, 'bot_2/odom', self.robot2_pose_callback, 10)        
        self.subscription_robot3 = self.create_subscription(Odometry, 'bot_3/odom', self.robot3_pose_callback, 10)
        self.robot1_pose = Odometry()
        self.robot2_pose = Odometry()
        self.robot3_pose = Odometry()
        self.sub_pose = self.create_subscription(PoseStamped, 'Pose', self.goal_callback,10)
        self.get_logger().info('Initialized..')

    def calc_dist(self,x,y,a,b):
        dist = math.sqrt(math.pow(x-a,2)+math.pow(y-b,2))
        return dist
    
    def get_yaw_from_quaternion(self, orientation):
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw
     
    def goal_callback(self, msg):
        p =  msg.pose
        goal_x = p.position.x
        goal_y = p.position.y
        goal_yaw = self.get_yaw_from_quaternion(p.orientation)
        delta1 = self.calc_dist(goal_x, goal_y, self.x1, self.y1)
        delta2 = self.calc_dist(goal_x, goal_y, self.x2, self.y2)
        delta3 = self.calc_dist(goal_x, goal_y, self.x3, self.y3)
        delta = [delta1,delta2,delta3]
        index = delta.index(min(delta)) + 1
        leaderNS = f'bot_{index}'
        if index==1:
            self.sendgoal(goal_x,goal_y,goal_yaw,1,0.0)
            self.sendgoal(goal_x,goal_y,goal_yaw,2,1.0)
            self.sendgoal(goal_x,goal_y,goal_yaw,3,2.0)
        elif index==2:
            self.sendgoal(goal_x,goal_y,goal_yaw,2,0.0)
            self.sendgoal(goal_x,goal_y,goal_yaw,1,1.0)
            self.sendgoal(goal_x,goal_y,goal_yaw,3,2.0)
        elif index==3:
            self.sendgoal(goal_x,goal_y,goal_yaw,3,0.0)
            self.sendgoal(goal_x,goal_y,goal_yaw,1,1.0)
            self.sendgoal(goal_x,goal_y,goal_yaw,2,2.0)
            
    def sendgoal(self,a,b,y,i,alpha):
        if i ==1:
            x_goal = a - (alpha*math.cos(y))
            y_goal = b - (alpha*math.sin(y))
            self.get_logger().info('sending goal to action server 1')
            goal_pose = NavigateToPose.Goal()
            goal_pose.pose.header.frame_id = 'map'
            goal_pose.pose.pose.position.x = x_goal
            goal_pose.pose.pose.position.y = y_goal
            goal_pose.pose.pose.orientation.z = y
            self.get_logger().info('waiting for action server 1')
            self._bot_1_action_client.wait_for_server()
            self.get_logger().info('action server 1 detected')
            self._send_goal_future = self._bot_1_action_client.send_goal_async(goal_pose, feedback_callback=self.feedback_callback)
            self.get_logger().info('goal sent')
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        elif i==2:
            x_goal = a - (alpha*math.cos(y))
            y_goal = b - (alpha*math.sin(y))
            self.get_logger().info('sending goal to action server 2')
            goal_pose = NavigateToPose.Goal()
            goal_pose.pose.header.frame_id = 'map'
            goal_pose.pose.pose.position.x = x_goal
            goal_pose.pose.pose.position.y = y_goal
            goal_pose.pose.pose.orientation.z = y
            self.get_logger().info('waiting for action server 2')
            self._bot_2_action_client.wait_for_server()
            self.get_logger().info('action server 2 detected')
            self._send_goal_future = self._bot_2_action_client.send_goal_async(goal_pose, feedback_callback=self.feedback_callback)
            self.get_logger().info('goal sent')
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        elif i==3:
            x_goal = a - (alpha*math.cos(y))
            y_goal = b - (alpha*math.sin(y))
            self.get_logger().info('sending goal to action server 3')
            goal_pose = NavigateToPose.Goal()
            goal_pose.pose.header.frame_id = 'map'
            goal_pose.pose.pose.position.x = x_goal
            goal_pose.pose.pose.position.y = y_goal
            goal_pose.pose.pose.orientation.z = y
            self.get_logger().info('waiting for action server 3')
            self._bot_3_action_client.wait_for_server()
            self.get_logger().info('action server 3 detected')
            self._send_goal_future = self._bot_3_action_client.send_goal_async(goal_pose, feedback_callback=self.feedback_callback)
            self.get_logger().info('goal sent')
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            
    def robot1_pose_callback(self,msg):
        self.x1 = msg.pose.pose.position.x
        self.y1 = msg.pose.pose.position.y
        self.yaw1 = self.get_yaw_from_quaternion(msg.pose.pose.orientation)        
    
    def robot2_pose_callback(self,msg):
        self.x2 =  msg.pose.pose.position.x
        self.y2 = msg.pose.pose.position.y
        self.yaw2 = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
    def robot3_pose_callback(self,msg):
        self.x3= msg.pose.pose.position.x
        self.y3 =msg.pose.pose.position.y    
        self.yaw3 = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(\n')
            return
        self.get_logger().info('Goal accepted :)\n')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result:' + str(result)+'\n')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback)+'\n' )

        
def main(args=None):
    rclpy.init(args=args)
    swarm_control = Followtheleader()
    rclpy.spin(swarm_control)
    
if __name__ == '__main__':
    main()