# Assignment 1
This repository contains the **ROS 2** solution developed for **Assignment 1** of the **Intelligent Robotics** course held at UniPD.  
The system integrates AprilTag-based goal computation, autonomous navigation with Nav2, LiDAR-based table detection through circle extraction, and a manual corridor-navigation fallback. It includes modular nodes for tag detection, circle detection, and navigation orchestration, together with launch files for both full-system execution and isolated component testing.

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
