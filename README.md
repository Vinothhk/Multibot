# Multi Robot Simulation

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
https://github.com/Vinothhk/Multibot/blob/main/img/leader_follower.gif