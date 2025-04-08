# Vikings Bot bringup package

This package conatains files to bringup nodes related to Vikings Bot project.

## Setup


### Install dependencies:
```
rosdep install -y --from-paths src --ignore-src
```


## Simulation
### Start simulation

__Launch gazebo world:__

```
ros2 launch vikings_bot_bringup start_simulation.launch.xml
```

__To spawn robot execute:__
```
ros2 launch vikings_bot_bringup bringup_vikings_bot.launch.py
```

__To spawn another robot execute:__
```
ros2 launch vikings_bot_bringup bringup_vikings_bot.launch.py robot_nr:=2
```
In Rviz enable second robot visualization.

__To move robots with keypad execute:__

Vikings Bot 1:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=vikings_bot_1/cmd_vel
```

For different numbered robots:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=vikings_bot_{robot_nr}/cmd_vel
```

__To test Nav2, in Rviz us `2D Goal Pose` button to send goal to robot.__
