## Project Presentation

For project installation and presentation, please refer to [NewbieOogway](https://newbie-oogway.readthedocs.io/en/latest/)

## BUILD and ONLY BUILD ros2_aruco FIRST. THEN SOURCE. 
```
colcon build --packages-select ros2_aruco
source install/setup.bash
```
## BUILD t4_wyman. THEN SOURCE
```
colcon build --packages-select t4_wyman
source install/setup.bash
```
## LAUNCH tb4_wyman_patrol.launch.py
```
ros2 launch t4_wyman tb4_wyman_patrol.launch.py
```
## Aruco detection
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```
## BUILD move_turtlebot
```
colcon build --packages-select move_turtlebot
source install/setup.bash
```
## RUN move_turtlebot4 node
```
ros2 run move_turtlebot4 move_node
```
## Move turtlebot4 robot
```
You could control the turtlebot4 move in gazebo and when the camera detects the AR marker,
the turtlebot4 velocity would be set to 0. And the turtlebot4 should automatically moves toward the marker
```


