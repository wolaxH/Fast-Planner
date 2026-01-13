# Fast-Planner Project Structure

This document describes the organization of the Fast-Planner codebase with SO(3) controller integration.

## 📁 Root Directory

```
Fast-Planner/
├── README.md              # Main project documentation
├── CLAUDE.md              # AI assistant development guide
├── SO3_SETUP.md           # SO(3) system setup and usage
├── PROJECT_STRUCTURE.md   # This file
├── .gitignore             # Git ignore rules
├── launch_so3.sh          # Main launch script for SO(3) system
├── setup_workspace.sh     # Workspace setup script
├── fast_planner/          # Core Fast-Planner modules
├── uav_simulator/         # UAV simulation components
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
│   │   ├── fast_planner_so3_pure.launch  # Main SO(3) system launch
│   │   ├── kino_algorithm_hector.xml     # Planning parameters
│   │   └── rviz.launch                    # Visualization
│   ├── scripts/          # Helper scripts
│   │   └── odom_to_tf.py                  # TF broadcaster for RViz
│   └── urdf/             # Robot models
│       └── hector_with_depth.urdf.xacro   # Quadrotor with depth camera
├── traj_utils/           # Trajectory utilities
└── poly_traj/            # Polynomial trajectory representation
```

### uav_simulator/
Quadrotor simulation and control components.

```
uav_simulator/
├── so3_quadrotor_simulator/   # Physics-based quadrotor dynamics
├── so3_control/               # SO(3) geometric controller
├── local_sensing_node/        # Depth camera/LiDAR simulation
├── map_generator/             # Random 3D obstacle map generation
└── Utils/
    └── waypoint_generator/    # Converts RViz goals to waypoints
```

## 🚀 Launch System

### Main Entry Point
**`launch_so3.sh`** - Single-command launch script

Starts:
1. Random obstacle map generator
2. SO(3) quadrotor simulator
3. SO(3) geometric controller
4. Depth camera simulation
5. Fast-Planner with kinodynamic replanning
6. RViz visualization

### Configuration Files

| File | Purpose |
|------|---------|
| `kino_algorithm_hector.xml` | Planning parameters (speed, safety, ESDF) |
| `gains.yaml` (so3_control) | Controller gains |
| `camera.yaml` (local_sensing) | Camera parameters |
| `kino.rviz` | RViz visualization config |

## 📊 Data Flow

```
Random Map Generator
  → Local Sensing (Depth Camera Simulation)
  → ESDF Map Builder
  → Fast-Planner (Kinodynamic A* + B-spline Optimization)
  → Trajectory Server
  → SO(3) Controller
  → SO(3) Quadrotor Simulator
  → Odometry feedback →  (loop)
```

### ROS Topics

**Input:**
- `/move_base_simple/goal` - RViz 2D Nav Goal
- `/visual_slam/odom` - Quadrotor odometry
- `/camera/depth/points` - Depth point cloud

**Output:**
- `/planning/pos_cmd` - Position commands to SO(3) controller
- `/so3_cmd` - SO(3) control commands
- `/planning/bspline` - Planned trajectory
- `/sdf_map/occupancy` - ESDF occupancy map

## 🛠️ Build System

```bash
# Build entire workspace
catkin_make

# Build specific package
catkin_make --pkg plan_manage

# Clean build
catkin_make clean && catkin_make
```

### Build Artifacts (Ignored by Git)
- `build/` - CMake build files
- `devel/` - Built executables and libraries
- `.catkin_workspace` - Catkin workspace marker

## 📚 Documentation

| Document | Purpose | Audience |
|----------|---------|----------|
| `README.md` | Project overview and original Fast-Planner docs | General users |
| `SO3_SETUP.md` | SO(3) system setup, usage, and troubleshooting | New users |
| `CLAUDE.md` | Development guide, architecture, parameters | Developers, AI assistants |
| `PROJECT_STRUCTURE.md` | Code organization and structure | Developers |

## 🔧 Key Components Added for SO(3) Integration

1. **`odom_to_tf.py`** - Broadcasts TF transforms for RViz visualization
2. **`fast_planner_so3_pure.launch`** - Complete SO(3) system launch file
3. **Modified waypoint_generator** - Set to `manual-lonely-waypoint` mode
4. **Trajectory server** - Bridges Fast-Planner and SO(3) controller

## 🎮 Usage Workflow

1. Launch system: `./launch_so3.sh`
2. Wait for initialization (5-10 seconds)
3. In RViz, select "2D Nav Goal" tool
4. Click target position
5. Watch autonomous obstacle avoidance!

## 📝 Development Notes

- **Planning Algorithm**: Kinodynamic A* (front-end) + B-spline optimization (back-end)
- **Controller**: SO(3) geometric tracking controller
- **Perception**: ESDF-based obstacle representation
- **Simulation**: Lightweight (no Gazebo), physics-based quadrotor dynamics
- **Frame IDs**: `world` (global), `body` (quadrotor)

## 🚫 Ignored Files (.gitignore)

- Build artifacts (`build/`, `devel/`)
- IDE files (`.vscode/`, `.idea/`)
- Logs (`*.log`, `*.txt` except docs)
- Temporary files (`*.tmp`, `*.bak`)
- Test scripts (`test_*.sh`, `check_*.sh`)
- Images (`*.png`, `*.jpg` except in docs/)

## 🔗 External Dependencies

- ROS Noetic (or Melodic)
- Eigen3
- PCL (Point Cloud Library)
- NLopt (v2.7.1)
- Armadillo

See `CLAUDE.md` for installation instructions.
