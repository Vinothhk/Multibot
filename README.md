# Multi Robot Simulation
### Description
Hello! This Repository contains ROS2 simulation files for Multi Robot Simulation in Gazebo World. Currently it works in 3 Robots.

*Leader - Follower Implementation*: The Coordinator_Node coordinates the movement of two follower robots (bot_2 and bot_3) based on the position of a leader robot (bot_1). It achieves this by subscribing to the Odometry messages of the leader and follower robots, computing new goal positions while maintaining a fixed distance gap, and sending navigation goals using the NavigateToPose action.

Goal pose for bot 2 is sent by getting leader bot's position (through odometry callback).
Goal pose for bot 3 is sent by getting bot 2's position, which means it follows the leader robot with double of the distance which is maintained by bot 2 with leader robot.

### Initial Setup
```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Vinothhk/Multibot.git

cd ..
colcon build && source install/setup.bash
```

### Launch Files
To launch the robots in gazebo:
```
ros2 launch multibot spawn_robots.launch.xml
```
Load the Navigation Stack:
```
ros2 launch multibot navigation.xml
```
### Script
Leader - Follower Script:
```
ros2 run multibot followtheleader.py
```

### Send Goal to robot

**Send Goal via RViz**

- Load the *follow_leader.rviz* from rviz folder of the package
- Ensure the goal pose topic is set to the */bot_1/goal_pose* in Tool Properties (or) Change it by clicking Panels -> Tool Properties -> 2D Goal Pose

**Using Joystick**
```
sudo apt install ros-$DISTRO-teleop-twist-joy
```
Then,
```
ros2 launch teleop_twist_joy teleop-launch.py joy_vel:='/bot_1/cmd_vel'
```

**Teleop**
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### DEMO
![demo gif](img/leader_follower.gif)
