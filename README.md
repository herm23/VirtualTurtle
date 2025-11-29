# Assignment 1
This repository contains the **ROS 2** solution developed for **Assignment 1** of the **Intelligent Robotics** course held at UniPD.  
The system integrates AprilTag-based goal computation, autonomous navigation with Nav2, LiDAR-based table detection through circle extraction, and a manual corridor-navigation fallback. It includes modular nodes for tag detection, circle detection, and navigation orchestration, together with launch files for both full-system execution and isolated component testing.

## Architecture overview
The robot starts from a predefined location, determines a goal positioned between two AprilTags, navigates autonomously to that point using Nav2, and finally detects cylindrical tables using a LiDAR-based circle-extraction pipeline. A manual corridor-navigation module is included as a fallback strategy to enhance robustness in constrained environments.

Key subsystems include:
- TagDetector Node: Queries AprilTag transforms, computes poses in arbitrary frames, and returns reliable tag information through a dedicated service.
- CircleDetector Node: Performs LiDAR clustering, refinement, and multi-stage circle fitting to locate cylindrical tables.
- NavigationNode: Coordinates localization, goal computation, autonomous navigation, corridor handling, and table detection.

![class_diagram_page-0001 (1)](https://github.com/user-attachments/assets/f2788e87-a4b4-4036-96c0-f89ce5d6d5d6)


## Demo videos
🎥 Project Videos https://drive.google.com/drive/folders/1x9G0x06GG4FukoS3zG4bcN6fa-_zNoG7?usp=sharing
## Launch Instructions
1. **Clone repositories into your ROS2 workspace**
```bash
cd YOUR_ROS2_WS/src
git clone https://github.com/PieroSimonet/ir_2526.git
git clone https://github.com/danieleCapuzzo/intelligent-robotics-assignment1.git
```

2. **Build the project**
```bash
cd YOUR_ROS2_WS
colcon build
```

3. **Source the workspace**
```bash
source install/setup.bash
```

5. **Launch the full assignment**
```bash
ros2 launch group28_assignament_1 launch.py
```
You may need to wait up to ~10 seconds for everything to start.
The delays are rather large but in the case of issues, try with bigger ones
and build/launch (modify the launch files).

## Optional: run components separately in two different terminals
```bash 
ros2 launch group28_assignament_1 sim.launch.py
ros2 launch group28_assignament_1 nodes.launch.py
```
This is particularly useful to visualize outputs well, avoiding them
to get mixed-up in with the cluttered simulation messages.
