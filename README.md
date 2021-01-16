rpi robotics team project, fall 2020
Ubuntu 18.04 + ROS melodic + Gazebo 9
run the following commands to start:

```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300

roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch

rosrun test main_final.py
```
