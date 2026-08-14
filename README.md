# GPS-Denied Navigation & Dynamic Replanning Simulation for MUM-T Scenarios

### A* Path Planning • Kalman-Based Localization • Dynamic Obstacle Replanning

A simulation-based autonomous navigation system developed for **Manned-Unmanned Teaming (MUM-T)-inspired operations**, integrating global A* path planning, dead-reckoning localization, Kalman-based state estimation, dynamic obstacle detection, and sequential path replanning in a synthetic GPS-denied environment.

> **Current scope:** This project is a single-agent simulation and algorithmic validation of navigation concepts relevant to MUM-T scenarios. Multi-agent MUM-T coordination is planned as future work.

---

## 🎬 Project Demonstration

### End-to-End Autonomous Navigation

![End-to-End Navigation](results/06_mumt_navigation.gif)

The final animation integrates the complete navigation pipeline:

- Synthetic occupancy-grid environment
- A* global path planning
- True robot trajectory
- Dead-reckoning localization
- Kalman-filter-based state estimation
- Dynamic obstacle introduction
- Sequential path replanning
- Collision checking
- Goal completion
- Localization-error comparison

---

# 🚀 Project Overview

Autonomous navigation becomes challenging when a robot must operate without GPS and when obstacles can appear after a route has already been planned.

This project explores that problem through a controlled 2D simulation.

The system begins with a synthetic occupancy grid and generates an initial collision-free path using A*. During navigation, noisy motion measurements are used to simulate localization drift through dead reckoning. A Kalman filter is then used to improve the position estimate using periodic simulated visual/map-based position observations.

The environment is subsequently modified by introducing dynamic obstacles. When the active path becomes blocked, the planner is invoked again from a point before the detected collision, allowing the robot to continue toward the goal.

---

# 🧭 Navigation Pipeline

```text
                    Synthetic Environment
                            │
                            ▼
                     Occupancy Grid
                            │
                            ▼
                       A* Planner
                            │
                            ▼
                      Planned Path
                       /         \
                      /           \
                     ▼             ▼
            Dead Reckoning     Dynamic Obstacles
                     │             │
                     ▼             ▼
              Kalman Filter    Collision Detection
                     │             │
                     ▼             ▼
              State Estimate    A* Replanning
                     │             │
                     └──────┬──────┘
                            ▼
                   End-to-End Simulation
                            │
                            ▼
                       Goal Reached
```

---

# 🎯 Project Objectives

- Generate a synthetic 2D occupancy-grid environment.
- Generate an initial collision-free route using A*.
- Simulate robot motion along the planned trajectory.
- Demonstrate localization drift using dead reckoning.
- Reduce localization error using Kalman filtering.
- Introduce dynamic obstacles during navigation.
- Detect when the current route becomes blocked.
- Determine a suitable replanning point before the obstacle.
- Replan the remaining route using A*.
- Support multiple sequential replanning events.
- Maintain a collision-free route after replanning.
- Visualize the complete navigation pipeline through an end-to-end animation.

---

# 🧩 System Components

## 1. Synthetic Occupancy Grid

A synthetic **200 × 200 cell occupancy grid** represents the navigation environment.

Static obstacles are generated using:

- Boundary walls
- Random rectangular obstacles
- Circular obstacles

The resulting occupancy grid is used as the environment for global path planning.

**Output:**

```text
data/occupancy_grid.npy
```

---

## 2. A* Global Path Planning

The A* algorithm generates an initial collision-free path from the start position to the goal.

The planner operates on the occupancy grid and searches for a feasible route around static obstacles.

**Output:**

```text
data/planned_path.npy
```

The generated path contains **347 points** with an initial path distance of **346 cells**.

---

## 3. Dead-Reckoning Localization

Dead reckoning estimates the robot's position from its previous state and simulated motion measurements.

Noise is introduced into the motion measurements to represent imperfect sensing.

Because dead reckoning integrates these errors over time, the estimated position gradually drifts away from the true trajectory.

**Output:**

```text
data/dead_reckoning_path.npy
```

---

## 4. Kalman-Filter State Estimation

A Kalman filter is used to improve the position estimate obtained from noisy dead-reckoning motion.

The prediction step uses simulated motion measurements, while periodic position observations are incorporated as measurement updates.

### Important simulation detail

The position observations used by the filter are **simulated visual/map-based measurements**. They are generated from the known simulation trajectory with measurement noise.

This is **not a physical camera system**.

The purpose is to model the type of external position observation that could be provided by a visual localization or map-matching system in a real deployment.

**Output:**

```text
data/kalman_filtered_path.npy
```

### Localization Results

| Metric | Dead Reckoning | Kalman Filter |
|---|---:|---:|
| Average Error | 5.49 cells | **2.30 cells** |
| Final Error | 23.25 cells | **2.66 cells** |
| Maximum Error | 23.25 cells | **5.19 cells** |

The Kalman filter achieved:

- **58.10% reduction in average localization error**
- **88.57% reduction in final localization error**

---

## 5. Dynamic Obstacle Detection

The initial A* path is not assumed to remain valid for the entire mission.

During navigation, dynamic obstacles are introduced sequentially into the environment.

When an obstacle intersects the active path, the system:

1. Detects the path blockage.
2. Identifies the first collision location.
3. Selects a point before the collision as the replanning location.
4. Initiates a new A* search from that point toward the goal.

This allows the navigation system to respond to changes in the environment rather than following the original route blindly.

---

## 6. Sequential A* Replanning

The replanner supports **multiple sequential dynamic-obstacle events**.

In the final simulation:

- 3 dynamic obstacles were introduced.
- 3 replanning events were triggered.
- All 3 replanning attempts succeeded.
- The final route remained collision-free.
- The robot reached the goal.

The final replanned route was slightly longer than the original route, demonstrating the expected trade-off between path efficiency and obstacle avoidance.

**Output:**

```text
data/dynamic_replanned_path.npy
```

---

## 7. End-to-End Simulation and Animation

The final animation combines the major components of the system into a single visualization.

It shows:

- Occupancy grid
- Original A* path
- True robot trajectory
- Dead-reckoning estimate
- Kalman-filter estimate
- Dynamic obstacles
- Replanning events
- Final goal state
- Localization-error comparison

---

# 📊 Results

## Navigation and Replanning Performance

| Metric | Result |
|---|---:|
| Occupancy Grid | 200 × 200 cells |
| Initial A* Path Points | 347 |
| Original Path Distance | 346 cells |
| Final Replanned Path Distance | 362 cells |
| Additional Distance | 16 cells |
| Additional Distance | **4.62%** |
| Dynamic Obstacles | 3 |
| Replans Triggered | 3 |
| Successful Replans | 3 |
| Final Path Collision | **No** |
| Goal Reached | **Yes** |

The dynamic replanning system successfully avoided all introduced obstacles while increasing the total path distance by only **16 cells (4.62%)** compared with the original route.

## Localization Performance

| Metric | Dead Reckoning | Kalman Filter |
|---|---:|---:|
| Average Error | 5.49 cells | **2.30 cells** |
| Final Error | 23.25 cells | **2.66 cells** |
| Maximum Error | 23.25 cells | **5.19 cells** |

### Improvement

```text
Average localization error reduction : 58.10%
Final localization error reduction   : 88.57%
```

---

# 📁 Project Structure

```text
MUMT-Autonomous-Navigation/
│
├── data/
│   ├── occupancy_grid.npy
│   ├── planned_path.npy
│   ├── dead_reckoning_path.npy
│   ├── kalman_filtered_path.npy
│   └── dynamic_replanned_path.npy
│
├── experiments/
│   ├── img_to_grid.py
│   └── replanned_path.npy
│
├── results/
│   ├── 01_occupancy_grid.png
│   ├── 02_a_star_path.png
│   ├── 03_dead_reckoning.png
│   ├── 04_kalman_filter.png
│   ├── 05_dynamic_replanning.png
│   └── 06_mumt_navigation.gif
│
├── src/
│   ├── animation.py
│   ├── kalman_filter.py
│   ├── localization.py
│   ├── planner.py
│   ├── replanner.py
│   └── synthetic_map.py
│
├── README.md
└── requirements.txt
```

The `experiments/` directory contains optional exploratory work that is not required for the main simulation pipeline.

---

# ⚡ Quick Start

## 1. Clone the repository

```bash
git clone https://github.com/ashka-22/MUMT-Autonomous-Navigation.git
cd MUMT-Autonomous-Navigation
```

## 2. Install dependencies

```bash
pip install -r requirements.txt
```

## 3. Run the pipeline

Run the modules in this order because each stage generates data required by subsequent stages.

### Generate the environment

```bash
python src/synthetic_map.py
```

### Generate the global A* path

```bash
python src/planner.py
```

### Generate the dead-reckoning trajectory

```bash
python src/localization.py
```

### Apply Kalman filtering

```bash
python src/kalman_filter.py
```

### Perform dynamic obstacle detection and replanning

```bash
python src/replanner.py
```

### Run the complete navigation animation

```bash
python src/animation.py
```

The generated results and animation are stored in the `results/` directory.

---

# 🛠️ Technologies Used

- **Python**
- **NumPy**
- **Matplotlib**
- **OpenCV**
- **A* Pathfinding**
- **Kalman Filtering**
- **2D Occupancy-Grid Mapping**
- **Autonomous Navigation Simulation**

---

# 🔬 Current Scope

This project currently focuses on **simulation and algorithmic validation** of an autonomous navigation stack.

The implemented system demonstrates:

- Global path planning
- Noisy localization
- Dead-reckoning drift
- Kalman-filter-based state estimation
- Dynamic obstacle handling
- Sequential path replanning
- Collision checking
- End-to-end visualization

The current implementation represents a **single autonomous agent** operating in a 2D simulated environment.

The MUM-T aspect is therefore currently **scenario-oriented rather than a full multi-agent MUM-T implementation**.

---

# ⚠️ Limitations

The current system has several simulation-level limitations:

- The environment is synthetically generated rather than obtained from a real-world map.
- Motion and sensor measurements are simulated.
- The Kalman filter uses simulated visual/map-based position observations rather than a physical camera.
- Dynamic obstacles are introduced according to predefined simulation events.
- The robot is represented as a point moving through a 2D grid.
- Realistic vehicle dynamics and kinematic constraints are not modeled.
- Obstacle detection is simulated rather than performed using a physical perception system.
- ROS/ROS 2 and Gazebo are not currently integrated.
- Multi-agent MUM-T coordination is not implemented in the current version.

---

# 🚀 Future Scope

### Perception

- Real-time camera-based obstacle detection
- Visual localization
- Map matching
- Sensor fusion with real sensor measurements

### Navigation

- Real-time replanning
- Dynamic-obstacle prediction
- More realistic robot kinematics
- Closed-loop control using state feedback

### Simulation

- ROS / ROS 2 integration
- Gazebo or other robotics simulators
- More realistic sensor and motion models

### MUM-T

- Multi-agent coordination
- Communication between aerial and ground platforms
- Cooperative planning
- Shared environmental mapping
- Distributed task allocation

### Hardware

- Deployment on a mobile robot
- Integration with real IMU/encoder measurements
- Real-world obstacle avoidance

---

# 👤 Author

**Ashka**

GitHub: [@ashka-22](https://github.com/ashka-22)

---

# 📌 Project Status

**Completed simulation-based navigation prototype**

The current version provides a complete end-to-end demonstration of autonomous navigation, localization, dynamic obstacle handling, and sequential replanning in a synthetic 2D environment.
