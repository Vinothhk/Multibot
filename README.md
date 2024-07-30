# Multibot

To launch the robots in gazebo:
```
ros2 launch multibot spawn_robots.launch.xml
```
Load the Navigation Stack:
```
ros2 launch multibot navigation.xml
```

**Send Goal via RViz**

- Load the *nav.rviz* from rviz folder of the workspace
- Ensure the goal pose topic is set to the */bot_1/goal_pose* in Tool Properties

**Using Joystick**
```
sudo apt install ros-$DISTRO-teleop-twist-joy
```
Then,
```
ros2 launch teleop_twist_joy teleop-launch.py joy_vel:='/bot_1/cmd_vel'
```

**Run the Script**
```
ros2 run multibot nav.py
```

https://github.com/user-attachments/assets/b32ab187-4374-4ac9-8bb0-ee5217228e8f



