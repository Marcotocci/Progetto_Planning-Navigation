# Lunar Ants
**Autonomous Lunar Exploration Architecture using ROS 2 Nav2 and Behavior Trees**

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-34a853?style=flat&logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-ff69b4?style=flat)
![C++](https://img.shields.io/badge/C++-17-00599C?style=flat&logo=c%2B%2B)

This repository contains the final project for the Planning and Navigation course (Prof. Riccardo Caccavale).  
The project implements a complete autonomous exploration and navigation stack for a lunar rover, featuring custom costmap layer, reactive decision-making, and visual servoing.

---

## Architecture & Overview
This project is built upon the Sim_Rover framework by PRISMA Lab. It extends the base simulation with advanced planning and navigation capabilities.

### Key Features
* **Custom Illumination Costmap Layer:** A C++ Nav2 plugin that dynamically injects high spatial costs into shadowed areas, forcing the global planner to compute energy-aware trajectories.
* **Reactive Behavior Tree Architecture:** Uses BehaviorTree.CPP via a dedicated executor to handle mission priorities and interrupts.
* **Grid-Based Frontier Exploration:** Autonomous mapping and state validation for CPU-efficient, long-term exploration.
* **Visual Servoing (IBVS):** Integration for active target tracking, managed through the mission's Behavior Tree.

---

## Installation & Prerequisites

To ensure all dependencies are correctly configured, this project requires a Docker-based setup. Please follow the procedure explained in the original repository to prepare your environment.

1. **Docker Setup (Image & Container)**: Go to the Prerequisites section of the [Sim_Rover repository](https://github.com/prisma-lab/Sim_Rover) and follow the instructions to download Docker, build the Docker image, and create the container.
2. **Clone this repository**
   ```bash
   git clone https://github.com/Marcotocci/Progetto_Planning-Navigation.git
   ```
3. **Build the project**:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## How to Run

The project requires launching the simulation environment with the navigation stack, the mission manager based on the Behavior Tree, and optionally the RViz visualization.

### 1. Launch Simulation & Navigation Stack
This command starts the Gazebo environment, loads the robot, and initializes SLAM, navigation nodes, and the illumination data publisher:
```bash
ros2 launch rover_bringup rover_sim.launch
```

### 2. Start the Lunar Mission
In a new terminal, execute the launch file that starts the decision-making architecture. This initializes the battery simulator, light zone manager, target detector, and the Behavior Tree executor:
```bash
ros2 launch progetto_planning lunar_mission.launch.py
```

### 3. Visualize with RViz
A pre-configured RViz setup is provided to visualize the mission data. A text file containing the exact launch command is located at `ros2_ws/src/pkg/progetto_planning/rviz`. Alternatively, you can open a new terminal and run the following command directly:
```bash
rviz2 -d src/pkg/progetto_planning/rviz/lunar_mission.rviz --ros-args -p use_sim_time:=true
```

---

## System Requirements
* **OS:** Ubuntu 22.04 LTS (via Docker)
* **ROS 2:** Humble Hawksbill
* **Simulator:** Gazebo Ignition / Harmonic

---

## Acknowledgments
This project is a derivative work based on the simulation environment provided by **PRISMA Lab (University of Naples Federico II)**.
* Base Repository: [PRISMA Lab - Sim_Rover](https://github.com/prisma-lab/Sim_Rover)
* Special thanks to Prof. Riccardo Caccavale for the guidance during the Planning and Navigation course.
