#!/bin/bash
# Setup Fast-Planner as a proper catkin workspace

echo "=========================================="
echo "Setting up Fast-Planner catkin workspace"
echo "=========================================="

cd /home/etho/Fast-Planner

# Create src directory if it doesn't exist
if [ ! -d "src" ]; then
    echo "[1/3] Creating src directory and symlinking packages..."
    mkdir -p src

    # Symlink fast_planner packages
    ln -sf ../fast_planner src/fast_planner

    # Symlink uav_simulator packages
    ln -sf ../uav_simulator src/uav_simulator

    echo "  ✓ Symlinks created"
else
    echo "[1/3] src directory already exists"
fi

# Initialize workspace if needed
if [ ! -f "src/CMakeLists.txt" ]; then
    echo "[2/3] Initializing catkin workspace..."
    cd src
    catkin_init_workspace
    cd ..
else
    echo "[2/3] Workspace already initialized"
fi

# Build
echo "[3/3] Building workspace..."
source /opt/ros/noetic/setup.bash
catkin_make

echo ""
echo "=========================================="
echo "Workspace setup complete!"
echo "=========================================="
echo ""
echo "To use the workspace, run:"
echo "  source /home/etho/Fast-Planner/devel/setup.bash"
echo ""
