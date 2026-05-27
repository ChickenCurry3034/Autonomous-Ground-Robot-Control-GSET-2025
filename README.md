# **PID Control Optimization and Trajectory Tracking for Autonomous Ground Robots**

## Overview

This project presents a novel approach to PID (Proportional-Integral-Derivative) control for differential drive robots, specifically the TurtleBot3. Traditional PID controllers assume independent control over all three degrees of freedom (x, y, and yaw), which is impossible for differential drive systems. This research demonstrates a novel control methodology that enables differential drive robots to reach arbitrary x, y, and yaw targets in two-dimensional space while maintaining trajectory fidelity.

The key innovation is that this approach enables cheaper, durable, and more energy-efficient differential drive drivetrains to achieve performance comparable to complex omnidirectional systems, making advanced autonomous control universally applicable across real-world robotic applications. The methodology has been validated using two path planning algorithms—A* and Dijkstra—to evaluate trajectory tracking consistency and algorithmic efficiency across diverse path complexities.

### Key Features
- **Differential Drive Control**: Novel PID methodology for 3-DOF control in 2-DOF systems
- **Multi-Algorithm Testing**: Validated with A* and Dijkstra path planning algorithms
- **Dual Implementation**: MATLAB simulations and Python real-robot implementation
- **Comparative Analysis**: Performance metrics and efficiency comparisons between algorithms
- **Scalable Architecture**: Applicable to any differential drive robotic platform

---

## Installation

### Prerequisites

Before beginning, ensure you have the following installed on your system:

- **For MATLAB Simulations**: 
  - MATLAB R2020a or later
  - Robotics System Toolbox (optional but recommended)
  
- **For Python Implementation (Real Robot)**:
  - Python 3.8 or later
  - ROS (Robot Operating System) Noetic or Foxy (on Ubuntu 20.04 or compatible Linux)
  - TurtleBot3 packages and dependencies

### MATLAB Installation

1. **Clone or download this repository** to your local machine:
   ```bash
   git clone <repository-url>
   cd "Autonomous Ground Robot Control"
   ```

2. **Navigate to the Simulation folder**:
   ```bash
   cd Simulation
   ```

3. **Open MATLAB** and add the simulation directories to your path:
   - In MATLAB, go to `Home` → `Set Path`
   - Add all subdirectories under `Simulation/` (A-star Alg, Basic PID, Dijkstra Alg, Enhanced PID)
   - Click "Save"

4. **Verify installation** by opening one of the `mainStd2025.m` files—it should display path and trajectory data without errors.

### Python/ROS Installation (Real Robot)

1. **Set up your Ubuntu machine** (20.04 LTS recommended) with ROS Noetic:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full
   ```

2. **Install TurtleBot3 dependencies**:
   ```bash
   sudo apt-get install ros-noetic-turtlebot3*
   sudo apt-get install ros-noetic-turtlebot3-msgs
   ```

3. **Create a ROS workspace** (if you don't have one):
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   ```

4. **Copy the TurtleBot3 source files**:
   - Copy the contents of the `Gazebo Workspace/src/` directory into `~/catkin_ws/src/`
   - Copy the Python control scripts from `TurtleBot Code/` to your workspace

5. **Build your workspace**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

6. **Install Python dependencies**:
   ```bash
   pip install numpy scipy matplotlib
   ```

---

## Setup

### MATLAB Simulation Setup

Each simulation variant is contained in its own directory and operates independently. The directory structure is as follows:

- **Basic PID**: Traditional PID control without enhancements
- **Enhanced PID**: Improved PID with gain optimization
- **A-star Alg**: Pathfinding using A* algorithm with Basic PID trajectory tracking
- **Dijkstra Alg**: Pathfinding using Dijkstra's algorithm with Basic PID trajectory tracking

**To configure a simulation:**

1. Open the corresponding `mainStd2025.m` file in MATLAB
2. Review the parameter settings at the top of the script:
   - `Kp`, `Ki`, `Kd`: PID tuning parameters
   - `waypoints`: Target coordinates for the robot trajectory
   - `dt`: Time step for simulation
   - `max_velocity`: Maximum linear velocity of the robot
3. Modify waypoints in the associated `.txt` files (e.g., `way_points_ext.txt`, `way_points_int.txt`) as needed
4. Run the script to generate simulation results

### Python/ROS Setup (Real Robot)

1. **Configure the TurtleBot3**:
   ```bash
   export TURTLEBOT3_MODEL=burger  # or waffle, depending on your model
   source ~/catkin_ws/devel/setup.bash
   ```

2. **Connect to the robot's ROS network**:
   - Ensure your laptop and robot are on the same network
   - On your laptop, set up the ROS master URI:
     ```bash
     export ROS_MASTER_URI=http://<robot-ip>:11311
     export ROS_HOSTNAME=<your-laptop-ip>
     ```

3. **Verify communication**:
   ```bash
   rostopic list
   ```
   If this returns a list of topics, your connection is successful.

4. **Select the control algorithm**:
   - Navigate to the `TurtleBot Code/` directory
   - Choose one of the implemented control strategies:
     - `enhancedPID.py`: Enhanced PID controller
     - `enhancedPD.py`: Enhanced PD (derivative only) controller
     - `enhancedP.py`: Enhanced P (proportional only) controller

---

## Experimentation

### Running MATLAB Simulations

1. **Launch the simulation**:
   - Open the desired `mainStd2025.m` file
   - Press `Run` (or Ctrl+Enter)

2. **Analyze simulation output**:
   - The simulation generates trajectory plots showing:
     - Actual robot path vs. desired path
     - Error metrics over time
     - Velocity and angular velocity profiles
   - `.txt` files are generated with detailed trajectory data for post-analysis

3. **Experiment workflow**:
   - **Baseline Testing**: Run simulations with default parameters
   - **Parameter Sweep**: Modify PID gains and observe convergence behavior
   - **Algorithm Comparison**: Compare A* and Dijkstra results using the same waypoints
   - **Scalability Testing**: Increase waypoint count and complexity to evaluate algorithm efficiency

### Running Real Robot Experiments

1. **Power on the TurtleBot3** and wait for system initialization (~30 seconds)

2. **Launch the control node**:
   ```bash
   cd ~/catkin_ws
   python src/enhancedPID.py  # Replace with your chosen controller
   ```

3. **Monitor real-time performance**:
   ```bash
   # In a separate terminal:
   rostopic echo /odom              # Monitor odometry
   rostopic echo /cmd_vel           # Monitor velocity commands
   rqt_plot /odom/pose/pose/position/x &  # Plot X position
   ```

4. **Data collection and analysis**:
   - Log experimental data using rosbag:
     ```bash
     rosbag record /odom /cmd_vel -o experiment_data
     ```
   - Analyze trajectories and compare with simulation predictions
   - Document performance metrics (settling time, steady-state error, overshoot)

### Comparison Experiments

To compare algorithm performance:

1. **Simulation Comparison**:
   - Run identical waypoint sequences through A-star and Dijkstra variants
   - Measure computation time and path optimality
   - Document results in the trajectory files for comparison

2. **Real vs. Simulated**:
   - Execute the same control algorithm on both simulation and real robot
   - Compare trajectory fidelity and identify sources of deviation
   - Adjust parameters iteratively to match real-world behavior

3. **Controller Tuning**:
   - Systematically vary PID parameters using the enhanced variants
   - Evaluate convergence speed, overshoot, and steady-state accuracy
   - Document optimal parameters for different path complexities

---

## Project Structure

```
Autonomous Ground Robot Control/
├── Simulation/                 # MATLAB-based simulations
│   ├── A-star Alg/            # A* pathfinding with PID control
│   ├── Basic PID/             # Standard PID implementation
│   ├── Dijkstra Alg/          # Dijkstra pathfinding with PID control
│   └── Enhanced PID/          # Optimized PID parameters
├── TurtleBot Code/             # Python implementations for real robot
│   ├── enhancedPID.py         # Enhanced PID controller
│   ├── enhancedPD.py          # Enhanced PD controller
│   └── enhancedP.py           # Proportional-only controller
├── Gazebo Workspace/           # ROS environment for simulation
│   └── src/
│       ├── turtlebot3/
│       ├── turtlebot3_msgs/
│       └── turtlebot3_simulations/
└── README.md                   # This file
```

---

## Authors

This project was developed by **Aarush Mane**, **Hannah Truitt**, **Rylan Chintada**, **Varun Akella**, and **Maahishee Patel** for the **New Jersey Governor's School of Engineering & Technology** at **Rutgers University**. The project was advised by **Shreya Srikanth** (Project Mentor) and **Daniel Baker** (Residential Teaching Assistant).

### Additional Resources
- GSET Rutgers Project Showcase (2025): [View publications and presentation videos](https://gset.rutgers.edu/publications/2025)

---

## Publications

### Peer-Reviewed Conference Paper
This research was presented and published at the **2025 MIT IEEE Undergraduate Research Technology Conference (URTC)**:

- **IEEE Xplore Publication**: [PID Control Optimization and Trajectory Tracking for Autonomous Ground Robots](https://ieeexplore.ieee.org/document/11533106)

### Extended Research Paper
The detailed technical paper and supplementary materials can be accessed through:
- **Google Drive**: [Full Research Paper](https://drive.google.com/file/d/1pk8C5_hpkfgGiQvAD37CN6ly7VYBQcyL/view?usp=sharing)

---

## Getting Help

- **MATLAB Issues**: Ensure all required toolboxes are installed and paths are correctly configured
- **ROS Setup Issues**: Verify network connectivity and environment variables are properly sourced
- **Real Robot Issues**: Check that the TurtleBot3 is powered and on the same network; review ROS logs with `roscore` running in a separate terminal
- **Contact**: For additional assistance, refer to the GSET Rutgers program resources or the original authors' institutional contacts
