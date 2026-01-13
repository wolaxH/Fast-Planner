#!/bin/bash

# Fast-Planner with SO(3) Controller Launch Script

echo "=== Fast-Planner + SO(3) Controller 啟動腳本 ==="
echo ""
echo "這個版本使用："
echo "  - SO(3) 四旋翼模擬器（專為 Fast-Planner 設計）"
echo "  - SO(3) 幾何控制器（高性能軌跡追蹤）"
echo "  - Random map 生成器（代替 Gazebo 世界）"
echo "  - 本地感知模擬（深度相機）"
echo ""

# Change to Fast-Planner directory
cd /home/etho/Fast-Planner

# Source ROS Noetic first
echo "1. Sourcing ROS Noetic..."
source /opt/ros/noetic/setup.bash

# Source Fast-Planner workspace
echo "2. Sourcing Fast-Planner workspace..."
source devel/setup.bash

# Verify packages
echo ""
echo "3. 驗證套件..."
if ! rospack find plan_manage > /dev/null 2>&1; then
    echo "❌ 錯誤：找不到 plan_manage 套件"
    exit 1
fi
echo "✓ plan_manage 找到"

if ! rospack find so3_quadrotor_simulator > /dev/null 2>&1; then
    echo "❌ 錯誤：找不到 so3_quadrotor_simulator 套件"
    exit 1
fi
echo "✓ so3_quadrotor_simulator 找到"

if ! rospack find so3_control > /dev/null 2>&1; then
    echo "❌ 錯誤：找不到 so3_control 套件"
    exit 1
fi
echo "✓ so3_control 找到"

# Launch the system
echo ""
echo "4. 啟動系統..."
echo "   - Random 障礙物地圖"
echo "   - SO(3) 四旋翼模擬器"
echo "   - SO(3) 控制器"
echo "   - Fast-Planner"
echo "   - RViz"
echo ""

roslaunch plan_manage fast_planner_so3_pure.launch 2>&1 | tee launch_so3_log.txt
