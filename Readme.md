## Summary

This Assignment consists of one ROS 2 package, "colourChaser", and 3 Python scripts **collision.py**,  **colourChaser.py**, and **roaming.py**. Where **collision** is a simple script to ensure the robot doesn't collide with any objects in the environment. **colourChaser** is the script that detects the colours in the background, creating a list to be ticked off when a colour is found. Lastly **roaming** is simply a script that sets the robot's velocity and moves it forward in the environment.

## Installation guide

twist mux is used to help with ros topics and requires downloading to root on every launch

```bash
sudo apt update
sudo apt install ros-humble-twist-mux
```

## StartUp

- open a termernal  
  
- `cd into ros2_ws/`
- open 4 other tabs

On tab 1

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
(after all the other tabs are set up)
ros2 launch uol_turtlebot_simulator object-search-1.launch.py
```

On tab 2

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch colourChaser twist_mux_launch.py
```

On tab 3

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run colourChaser roamer
```

On tab 4

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run colourChaser chaser
```

On tab 5

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run colourChaser collision
```
