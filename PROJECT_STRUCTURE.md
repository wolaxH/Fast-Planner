# Fast-Planner Project Structure

This document describes the organization of the Fast-Planner codebase with Hector Quadrotor + Gazebo integration.

## 📁 Root Directory

```
Fast-Planner/
├── README.md              # Main project documentation
├── CLAUDE.md              # AI assistant development guide
├── PROJECT_STRUCTURE.md   # This file
├── .gitignore             # Git ignore rules
├── fast_planner/          # Core Fast-Planner modules
├── uav_simulator/         # UAV simulation components (SO(3) system)
├── hector_ws/             # Hector Quadrotor workspace
└── src/                   # ROS workspace source link
```

## 🎯 Core Modules

### fast_planner/
Main Fast-Planner algorithms and planning system.

```
fast_planner/
├── plan_env/              # Environment perception (ESDF, occupancy mapping)
├── path_searching/        # Front-end path planning (Kinodynamic A*, Topo PRM)
├── bspline/              # B-spline trajectory representation
├── bspline_opt/          # Back-end trajectory optimization
├── plan_manage/          # High-level planning FSM and coordination
│   ├── launch/           # Launch files
│   │   ├── hector_fast_planner.launch    # Main Hector+Gazebo launch
│   │   ├── kino_algorithm_hector.xml     # Planning parameters for Hector
│   │   ├── kino_replan.launch            # SO(3) lightweight simulation
│   │   └── rviz.launch                   # Visualization
│   ├── src/              # Source files
│   │   ├── hector_cmd_bridge.cpp         # Fast-Planner → Hector velocity bridge
│   │   └── traj_server.cpp               # Trajectory execution server
│   ├── scripts/          # Helper scripts
│   │   └── odom_to_tf.py                 # TF broadcaster for RViz
│   └── worlds/           # Gazebo world files
│       └── fast_planner_obstacles.world  # Obstacle environment
├── traj_utils/           # Trajectory utilities
└── poly_traj/            # Polynomial trajectory representation
```

### hector_ws/
Hector Quadrotor simulation workspace for Gazebo integration.

```
hector_ws/
└── src/
    ├── hector_quadrotor/          # Quadrotor simulation and control
    │   ├── hector_quadrotor_gazebo/       # Gazebo spawn launch files
    │   ├── hector_quadrotor_description/  # URDF models
    │   └── hector_quadrotor_controllers/  # Flight controllers
    └── hector_models/
        └── hector_sensors_description/    # Kinect camera URDF
```

### uav_simulator/
Lightweight quadrotor simulation (SO(3) system, no Gazebo).

```
uav_simulator/
├── so3_quadrotor_simulator/   # Physics-based quadrotor dynamics
├── so3_control/               # SO(3) geometric controller
├── local_sensing_node/        # Depth camera/LiDAR simulation
├── map_generator/             # Random 3D obstacle map generation
└── Utils/
    └── waypoint_generator/    # Converts RViz goals to waypoints
```

## 🚀 Launch Systems

### Option 1: Hector Quadrotor + Gazebo (Recommended)

**Launch Command:**
```bash
source /opt/ros/noetic/setup.bash
source ~/Fast-Planner/hector_ws/devel/setup.bash --extend
source ~/Fast-Planner/devel/setup.bash --extend
roslaunch plan_manage hector_fast_planner.launch
```

**Features:**
- Full 3D visualization in Gazebo
- Realistic Kinect depth camera
- Hector Quadrotor physics
- Obstacle world environment

### Option 2: SO(3) Lightweight Simulation

**Launch Command:**
```bash
source ~/Fast-Planner/devel/setup.bash
roslaunch plan_manage kino_replan.launch
```

**Features:**
- Faster startup (no Gazebo)
- Random obstacle generation
- SO(3) geometric controller
- Point cloud visualization

## ⚙️ Configuration Files

| File | Purpose |
|------|---------|
| `kino_algorithm_hector.xml` | Planning parameters for Hector (ESDF, optimization) |
| `hector_fast_planner.launch` | Main Hector system launch |
| `kino.rviz` | RViz visualization config |
| `fast_planner_obstacles.world` | Gazebo obstacle world |

### Key Parameters (kino_algorithm_hector.xml)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `sdf_map/pose_type` | 3 | DEPTH_ODOM_INDEP mode (no timestamp sync) |
| `sdf_map/obstacles_inflation` | 0.3 | Obstacle safety margin |
| `optimization/dist0` | 0.8 | Safe distance threshold |
| `optimization/lambda2` | 30.0 | Distance cost weight |
| `manager/max_vel` | 1.0 | Maximum velocity (m/s) |

## 📊 Data Flow (Hector System)

```
Gazebo World (Obstacles)
  → Kinect Camera (Depth Image)
  → SDFMap (ESDF Building, pose_type=3)
  → Fast-Planner (Kinodynamic A* + B-spline)
  → Trajectory Server
  → Hector Command Bridge (/cmd_vel)
  → Hector Quadrotor Controller
  → Gazebo Physics
  → Odometry feedback → (loop)
```

### ROS Topics

**Input:**
- `/move_base_simple/goal` - RViz 2D Nav Goal
- `/ground_truth/state` - Hector odometry
- `/camera/depth/image_raw` - Kinect depth image
- `/camera/depth/points` - Kinect point cloud

**Output:**
- `/cmd_vel` - Velocity commands to Hector
- `/planning/bspline` - Planned trajectory
- `/sdf_map/occupancy` - ESDF occupancy map

## 🛠️ Build System

```bash
# Build Hector workspace first
cd ~/Fast-Planner/hector_ws
catkin_make

# Build Fast-Planner
cd ~/Fast-Planner
catkin_make

# Build specific package
catkin_make --pkg plan_manage
```

## 🔧 Key Components for Hector Integration

1. **`hector_cmd_bridge.cpp`** - Converts Fast-Planner position commands to Hector velocity commands
2. **`pose_type=3` (DEPTH_ODOM_INDEP)** - Independent depth/odom subscribers to avoid timestamp sync issues
3. **Camera frame transform** - Converts Kinect optical frame to world frame
4. **Map boundary check** - Prevents crashes when UAV flies outside map bounds

## 🎮 Usage Workflow

1. Launch system: `roslaunch plan_manage hector_fast_planner.launch`
2. Wait for Gazebo and RViz to initialize
3. In RViz, select "2D Nav Goal" tool
4. Click target position (keep z ≈ 1.0m)
5. Watch autonomous obstacle avoidance!

## 📝 Development Notes

- **Planning Algorithm**: Kinodynamic A* (front-end) + B-spline optimization (back-end)
- **Controller**: Hector velocity control via `/cmd_vel`
- **Perception**: ESDF-based obstacle representation from Kinect depth
- **Simulation**: Gazebo with Hector Quadrotor physics
- **Frame IDs**: `world` (global), `base_link` (quadrotor)

## 🔗 External Dependencies

- ROS Noetic
- Gazebo 11
- Eigen3
- PCL (Point Cloud Library)
- NLopt (v2.7.1)
- Armadillo
- Hector Quadrotor packages

See `CLAUDE.md` for installation instructions.
