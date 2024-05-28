# Robotics presentation: roslaunch commands

## Presentation (SlideNumber)

### Bringup of the robot (0)

```bash
roslaunch rubot_mecanum_description rubot_bringup_hw_rock_custom.launch
```

### Autonomous Driving of pattern (5)

```bash
roslaunch rubot_control rubot_nav_path_custom.launch
```

### Self Navigation holonomic (6)

```bash
roslaunch rubot_control rubot_self_nav_holonomic.launch 
```

### Wall Follower holonomic (9)

```bash
roslaunch rubot_control ruboot_wall_follower_holonomic.launch
```

### SLAM Bringup

```bash
roslaunch rubot_slam rubot_slam_bringup_hw_rock.launch
```

### Generating the map (10)
    
```bash
roslaunch rubot_slam rubot_slam.launch
```

You might want to move the rubot around to see the whole map, 
- use wall follower:
    
```bash
roslaunch rubot_control ruboot_wall_follower_holonomic.launch
```
- or use keyboard teleop:
    
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


### Follow waypoints (11)

```bash
roslaunch rubot_slam rubot_navigation.launch
```

First define the waypoints in the waypoints.yaml file.
Then execute this script:
```bash
roslaunch rubot_slam waypoints_goal.launch
```

### Image Processing (14)

```bash
roslaunch rubot_slam rubot_navigation.launch
```

```bash
roslaunch rubot_projects rubot_project3_signals.launch
```

### Computer Vision only simulation (16)

```bash
roslaunch rubot_projects rubot_projects_bringup_sw.launch 
```
    
```bash
roslaunch rubot_slam rubot_navigation_project3.launch
```

```bash
roslaunch rubot_projects rubot_project_computer_vision.launch
```



## Optional

### Control Robot with keyboard

```bash
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
or
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Wall Follower non-holonomic

```bash
roslaunch rubot_control .launch
```

