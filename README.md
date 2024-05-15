## Project Presentation

For project installation and presentation, please refer to Our Project Website

[NewbieOogway](https://newbie-oogway.readthedocs.io/en/latest/)


_________________________________________________

## A Project name - "NewbieOogway"

## B Team members
- Xingyu Chen (xchen281@jh.edu)
- Zhiyuan Zhu (zzhu77@jhu.edu)
- Yinan Liu (yliu569@jh.edu)


## C Short description of the project/application
Our application leverages the advanced and flexible platforms of TurtleBot4 and OpenManipulatorX. Our aim is to unveil a groundbreaking prototype that encapsulates the core functionalities of navigation and exploration within indoor environments, akin to the esteemed capabilities of a Roomba robot. While it may not feature the conventional vacuum or mop head, it excels in other significant aspects, offering an enhanced user experience.

This prototype stands out by integrating an innovative feature—a robotic arm designed to efficiently clear its path of obstacles. This addition addresses a common frustration among users of traditional cleaning robots: the struggle with clothing and other items that often impede their operation. By seamlessly blending the reliable movement and search capabilities of a Roomba with the precision and adaptability of the OpenManipulatorX.

## D Required hardware
- [OpenManupulatorX](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)
- [TurtleBot4](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)

## E Required purchases
- TBD (see in next week)


## F Deliverables
- I Minimal: list the ROS packages that will be delivered regardless of problems encountered
    - Turtlebot
        - ObjectDetection_Msgs: Custom messages for object detection data.
        - description packages: on top of t4 and arm (assignment #4)
        - ObjectDetection_Action: Utilizing OpenCV for simple object detection, obtaining relative distance to the object.
        - Use AR tags for objects first (then CV if necessary)
        - PathTracking_Msgs: Messages for path tracking commands and status. (check navigation packages)
        - PathTracking_Action: Control the Turtlebot4 along a predefined path, and stop in front of object (check navigation package)
        - RViz: Utilize RViz for real-time visualization of sensor data, robot state, and planned movements. Essential for debugging and demonstration purposes.
        - Gazebo: Deploy Gazebo for physics-based simulation to test robot behaviors in a controlled virtual environment. Enables testing of motion planning, collision avoidance, and interaction with virtual objects.
    - OpenManupulatorX (Workable in Simulation)
        - RbtArmTf_Msg: communicate the arm 6DOF information and status
        - RbtArm_Action: communicate the arm movement and status (idle/active etc.) (check available actions MoveIt and use its services)
        - GripperTf_Msg: communicate the gripper 1DOF information and status
        - Gripper_Action: communicate the gripper movement and status (idle/active etc.)
        - RbtArm_Kinematic: calculate the kinematic (forward and inverse) of robotic arm
        - Visualization (URDF): visualize the robotic arm in simulation envrionment
        - GUI (testing): joint space or cartesian space control GUI for independent testing


- II Expected: list the ROS packages that will be delivered if everything goes according to plan
    - Turtlebot
        - ObstacleDetection_Msgs: Messages for Obstacle detection data, including obstacles and large objects.
        - ObstacleDetection_Action: Use Yolo deployed on remote workstation for advanced object and obstacle detection, identify object and obstacles. (keep this for maximum deliverables)
        - Once object detected move close enough (navigation action) to get object within arm reach
        - PathPlanning_Msgs: Messages for path planning instructions and feedback. (navigation stack)
        - PathPlanning_Action: Dynamicly plan a line path towards object based on object and obstacle detection.
        - WifiComm_Msgs: Messages for advanced data transmission, including WiFi. (this should be ok with robot)
        - WifiComm_Node: Managing advanced communication between the remote workstation and the robot, sending images from camera and receiving obstacle detection result from workstation.
        - RViz: Enhanced integration with RViz for detailed visualization of obstacle detection, path planning, and communication effectiveness.
        - Gazebo: Advanced simulation scenarios in Gazebo, including obstacles and complex environments for robustness testing of obstacle avoidance and path planning.
    - OpenManupulatorX (Workable in Simulation and realworld but have some bugs e.g. difference from simulation, inaccuracy position to the object, collision due to inappropriate path plan)
        - RbtArmTf_Msg: communicate the arm 6DOF information and status
        - Put a sequence of detect/move base(navigation)/move arm (moveit)/close gripper (mobile manipulation package)
        - RbtArm_Action: communicate the arm movement and status (idle/active etc.)
        - GripperTf_Msg: communicate the gripper 1DOF information and status
        - Gripper_Action: communicate the gripper movement and status (idle/active etc.)
        - RbtArm_Kinematic: calculate the kinematic (forward and inverse) of robotic arm (learn how to use MoveIt)
        - Visualization (URDF): visualize the robotic arm in simulation envrionment
        - **RbtArm_MotionPlanner:** calculate the plan path of robotic arm (minimal deliverable with Moveit)
        
    

- III Maximum: list the ROS packages that will be delivered if projects turns out better than expected
    - Turtlebot
        - PointCloudProcessing_Msgs: Messages for point cloud data and processing results.
        - Upgrade services to CV
        - PointCloudProcessing_Node: Node for acquiring and processing point clouds using Lidar.
        - Navigation stack for slam/navigation/exploration
        - Interrupt exploration and switch to pick/place
        - DecisionMaking_Msgs: Messages for decision-making processes: look for AR tag/CV and decide to pick up an object
        - DecisionMaking_Node: Node that integrates obstacle avoidance, path planning, and object detection for complex task execution strategies. (done by navigation)
        - RViz: Utilize RViz for comprehensive visualization including point cloud processing, coordinate transformation, and decision-making strategies.
        - Gazebo: Deploy complex simulation environments in Gazebo that challenge the robot with varied terrain, obstacles, and intricate tasks requiring advanced decision-making.
    - OpenManupulatorX (Workable in Simulation and realworld with no bug)
        - RbtArmTf_Msg: communicate the arm 4DOF information and status
        - Confirm that arm picked up object
        - RbtArm_Action: communicate the arm movement and status (idle/active etc.)
        - GripperTf_Msg: communicate the gripper 1DOF information and status
        - Gripper_Action: communicate the gripper movement and status (idle/active etc.)
        - RbtArm_Kinematic: calculate the kinematic (forward and inverse) of robotic arm
        - Visualization (URDF): visualize the robotic arm in simulation envrionment
        - GUI (testing): joint space or cartesian space control GUI for independent testing
        - **RbtArm_MotionPlanner:** calculate the plan path of robotic arm
        


## G Timetable
- I Timeline for progress

| | phase                  | timeline | deliverable |
|-| ---------------------- | -------- | ----------- |
|1| Scope and Requirement  | week1    | **1**. Investigate the hardware functions of platform. **2**. Investigate the software package of the platform 3. Determine the need to purchase hardware and update the timeline| 
|2| Architecture Design    | week1, 2 | **1**. Determine the 1st version system architecture and design language for the following development. **2**. Finalize the functions (*algorithms*)  and packages (*communication/message pipeline*), and allocate workload per person **3**. determine the software integration test milestone dates. **4**. Detail the deliveries (minimal, expected, and maximum) after familiar with the system.
|3| Development            |          | **1**. Integration and merge repo per week. **2**. Track the progress of per person and per functions


- II Timeline for hardware
- Ready to use [TurtleBot4](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html#hardware-specifications)
- Ready to use [OpenManupulatorX](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/) (Suppose only rely on turtle bot4 power supply and do not need extra PSU)

