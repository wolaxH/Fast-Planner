# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Fast-Planner is a quadrotor motion planning system for fast flight in complex unknown environments. It implements a hierarchical planning architecture combining perception (volumetric mapping + ESDF), front-end path searching (kinodynamic A* or topological PRM), and back-end trajectory optimization (gradient-based B-spline optimization).

**Tech Stack:** ROS (Melodic/Noetic), C++11/14, CMake, Eigen3, PCL, NLopt, Armadillo

## Build Commands

### Initial Setup
```bash
# Install dependencies (must be done first)
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt && mkdir build && cd build
cmake .. && make && sudo make install

sudo apt-get install libarmadillo-dev
```

### Build System
```bash
# Full build from workspace root
catkin_make

# Clean rebuild
catkin_make clean
catkin_make

# Build specific package
catkin_make --pkg plan_manage

# Build with verbose output for debugging
catkin_make -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Source Workspace
```bash
source devel/setup.bash
```

## Running Simulations

### Launch Visualization (Terminal 1)
```bash
source devel/setup.bash
roslaunch plan_manage rviz.launch
```

### Kinodynamic Replanning (Terminal 2)
```bash
source devel/setup.bash
roslaunch plan_manage kino_replan.launch
```

### Topological Replanning (Alternative to Kinodynamic)
```bash
source devel/setup.bash
roslaunch plan_manage topo_replan.launch
```

**Interaction:** Use the "2D Nav Goal" tool in RViz to set goal positions for the drone.

## Architecture Overview

### Module Hierarchy

```
fast_planner/
├── plan_env/          # Environment perception and mapping
├── path_searching/    # Front-end path planning algorithms
├── bspline/          # B-spline trajectory representation
├── bspline_opt/      # Back-end trajectory optimization
├── plan_manage/      # High-level planning coordination & FSMs
├── traj_utils/       # Trajectory utilities
└── poly_traj/        # Polynomial trajectory representation

uav_simulator/        # Lightweight quadrotor simulator
├── map_generator/           # Random 3D obstacle map generation
├── local_sensing/          # Depth camera/LiDAR simulation
├── so3_quadrotor_simulator/ # Physics-based quadrotor dynamics
└── so3_control/            # Low-level geometric controller
```

### Data Flow Pipeline

**Kinodynamic Replanning:**
```
Depth Image + Odometry
  → SDFMap (raycasting + ESDF building)
  → KinodynamicAstar (6D state space search with dynamics)
  → NonUniformBspline (fit B-spline to path)
  → BsplineOptimizer (gradient-based refinement)
  → Trajectory execution
```

**Topological Replanning:**
```
Depth Image + Odometry
  → SDFMap (raycasting + ESDF building)
  → TopologyPRM (sample multiple distinct paths)
  → BsplineOptimizer (optimize each path with guide)
  → FastPlannerManager::selectBestTraj()
  → Trajectory execution
```

### Key Components

#### plan_env - Environment Representation
- **SDFMap**: Probabilistic volumetric mapping with ESDF computation
  - Input: Depth images + camera pose (or point clouds)
  - Process: Raycasting to update occupancy, then compute Euclidean distance field
  - Output: 3D voxel grid with signed distances for collision checking
- **EDTEnvironment**: Planning interface wrapper providing trilinear interpolation for distance/gradient queries

#### path_searching - Front-end Planning
- **KinodynamicAstar**: Searches 6D state space (position + velocity) with forward shooting to respect quadrotor dynamics
- **TopologyPRM**: Generates multiple topologically distinct paths using PRM with homotopy pruning
- **Astar**: Standard geometric A* for baseline comparison

#### bspline - Trajectory Representation
- **NonUniformBspline**: Non-uniform B-spline implementation (order-3, C² continuity)
  - De Boor's algorithm for efficient evaluation
  - Time parameterization and feasibility checking
  - Methods: `evaluateDeBoor()`, `getDerivative()`, `parameterizeToBspline()`, `checkFeasibility()`

#### bspline_opt - Back-end Optimization
- **BsplineOptimizer**: Gradient-based optimization using NLopt
  - Multi-objective cost: smoothness (jerk) + distance (clearance) + feasibility + endpoint + guide
  - Uses ESDF gradients for efficient collision avoidance
  - Main interface: `BsplineOptimizeTraj()`

#### plan_manage - High-level Coordination
- **FastPlannerManager**: Coordinates all planning modules
  - Main methods: `kinodynamicReplan()`, `topoReplan()`, `planGlobalTraj()`, `checkTrajCollision()`
- **KinoReplanFSM** / **TopoReplanFSM**: Finite state machines managing planning lifecycle
  - States: INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ
  - Trigger replanning on collision detection or new goals

## Configuration Files

Launch files are in `fast_planner/plan_manage/launch/`:
- `kino_replan.launch` - Kinodynamic planning simulation
- `topo_replan.launch` - Topological planning simulation
- `kino_algorithm.xml` / `topo_algorithm.xml` - Core algorithm parameters
- `simulator.xml` - Simulator configuration (map size, obstacle density)
- `rviz.launch` - RViz visualization settings

### Important Parameters

**Mapping** (`kino_algorithm.xml` lines 59-98):
```xml
sdf_map/resolution: 0.1              # Voxel size in meters
sdf_map/map_size_x/y/z: 40/20/5     # Map dimensions
sdf_map/obstacles_inflation: 0.099   # Safety margin
sdf_map/local_update_range: 5.5      # Local mapping window
sdf_map/skip_pixel: 2                # Depth image downsampling (use 1 for low-res cameras)
sdf_map/depth_filter_maxdist: 5.0    # Max sensing range
sdf_map/k_depth_scaling_factor: 1000.0  # Device-dependent depth scaling
```

**Planner** (`kino_algorithm.xml` lines 101-112):
```xml
manager/max_vel: 3.0                    # Max velocity (m/s)
manager/max_acc: 2.0                    # Max acceleration (m/s²)
manager/max_jerk: 4.0                   # Max jerk (m/s³)
manager/local_segment_length: 6.0       # Replanning horizon
manager/clearance_threshold: 0.2        # Min obstacle distance
manager/control_points_distance: 0.5    # B-spline control point spacing
```

**Kinodynamic Search** (`kino_algorithm.xml` lines 114-126):
```xml
search/max_tau: 0.6                # Max time step for state transition
search/horizon: 7.0                # Search horizon distance
search/lambda_heu: 5.0             # A* heuristic weight
search/resolution_astar: 0.1       # Spatial discretization
search/allocate_num: 100000        # Node pool size
```

**Optimization** (`kino_algorithm.xml` lines 128-160):
```xml
optimization/lambda1: 10.0         # Smoothness weight (jerk minimization)
optimization/lambda2: 5.0          # Distance weight (clearance maximization)
optimization/lambda4: 0.01         # Endpoint weight
optimization/dist0: 0.4            # Safe distance threshold
optimization/max_iteration_num2: 300  # NLopt iteration limit
optimization/algorithm1/2: 15/11   # NLopt algorithm IDs
```

**Flight Mode** (`kino_replan.launch` line 40):
```xml
flight_type: 1  # Use 2D Nav Goal to select targets
flight_type: 2  # Follow predefined waypoints (point0_x/y/z, point1_x/y/z, ...)
```

## GPU Depth Rendering (Optional)

For more realistic depth simulation, enable CUDA in `uav_simulator/local_sensing/CMakeLists.txt`:
```cmake
set(ENABLE_CUDA true)
set(CUDA_NVCC_FLAGS -gencode arch=compute_XX,code=sm_XX;)  # Set based on GPU
```
Check GPU compute capability at https://developer.nvidia.com/cuda-gpus

## Development Patterns

### Adding New Planning Algorithms
1. Implement path finder in `fast_planner/path_searching/`
2. Register in `FastPlannerManager::initPlanModules()` in `fast_planner/plan_manage/src/planner_manager.cpp`
3. Add planner selection logic in FSM (`kino_replan_fsm.cpp` or `topo_replan_fsm.cpp`)
4. Create corresponding launch file and algorithm XML in `plan_manage/launch/`

### Modifying Optimization Costs
Edit cost functions in `fast_planner/bspline_opt/src/bspline_optimizer.cpp`:
- `calcSmoothnessCost()` - Jerk minimization
- `calcDistanceCost()` - Obstacle clearance
- `calcFeasibilityCost()` - Dynamic constraints
- Combine in `combineCost()` with lambda weights

### Adjusting Map Resolution
Change `sdf_map/resolution` in algorithm XML files. Smaller values (e.g., 0.05) provide finer detail but increase memory/compute. Must also adjust `search/resolution_astar` proportionally.

### Custom Sensor Integration
Replace simulator topics in launch files:
- `/pcl_render_node/depth` → your depth topic
- `/pcl_render_node/camera_pose` → your camera pose topic
- `/state_ukf/odom` → your odometry topic
Update camera intrinsics (`cx`, `cy`, `fx`, `fy`) and `k_depth_scaling_factor` accordingly.

## Common Issues

**Build Errors with NLopt:** Ensure NLopt v2.7.1 is installed. Check `/usr/local/lib` and `/usr/local/include/nlopt.h`.

**ESDF Visualization Not Showing:** Set `sdf_map/show_esdf_time: true` in algorithm XML to profile ESDF update times.

**Planner Fails to Find Path:** Increase `search/horizon` or decrease `search/resolution_astar`. Check map inflation settings.

**Trajectory Violates Dynamic Limits:** Decrease `manager/max_vel` and `manager/max_acc`, or increase `bspline/limit_ratio`.

**Compilation Errors:** ROS Melodic (Ubuntu 18.04) and Noetic (Ubuntu 20.04) are supported. Check Eigen3 and PCL versions match ROS distribution.

## ROS Topics

**Subscribed:**
- `/odom_world` - Odometry (nav_msgs/Odometry)
- `/sdf_map/depth` - Depth image (sensor_msgs/Image)
- `/sdf_map/pose` - Camera pose (geometry_msgs/PoseStamped)
- `/sdf_map/cloud` - Point cloud (sensor_msgs/PointCloud2)
- `/waypoint_generator/waypoints` - Goal waypoints (nav_msgs/Path)

**Published:**
- `/planning/pos_cmd` - Position commands (quadrotor_msgs/PositionCommand)
- `/planning/bspline` - B-spline trajectory (plan_manage/Bspline)
- `/planning/data_display` - Visualization markers (visualization_msgs/MarkerArray)

## References

**Papers:**
- RA-L 2019: Kinodynamic path searching + B-spline optimization
- ICRA 2020: Topological path searching + path-guided optimization
- T-RO: RAPTOR perception-aware planning

**Related Projects:**
- [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner) - Extended for multi-agent planning
- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) - Fast autonomous exploration
- [RACER](https://github.com/SYSU-STAR/RACER) - Autonomous racing
