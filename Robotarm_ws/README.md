## Project Presentation

For project installation and presentation, please refer to [NewbieOogway](https://newbie-oogway.readthedocs.io/en/latest/)

## Start ManipulatorX and visualize in Rviz
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py 
ros2 launch open_manipulator_x_description open_manipulator_x_rviz.launch.py
```
## CLI controller
```
ros2 run open_manipulator_x_teleop teleop_keyboard
```
## RealSense PointCloud2
```
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
## RealSense Camera and Aruco detection
```
ros2 launch realsense2_camera rs_launch.py
ros2 launch aruco_ros single.launch.py 
```
