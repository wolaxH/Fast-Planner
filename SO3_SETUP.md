# Fast-Planner 模擬系統指南

本專案提供兩種模擬環境：**Hector Quadrotor + Gazebo** 和 **SO(3) 輕量模擬**。

---

## 🎯 系統比較

| 特性 | Hector + Gazebo | SO(3) 輕量模擬 |
|------|-----------------|---------------|
| 視覺效果 | ✅ 完整 3D 渲染 | ⚠️ 點雲視覺化 |
| 啟動速度 | ⚠️ 較慢 (Gazebo) | ✅ 快速 |
| 物理模擬 | ✅ Gazebo 物理引擎 | ✅ 簡化動力學 |
| 深度相機 | ✅ Kinect 模擬 | ✅ 點雲生成 |
| 避障效果 | ✅ 優秀 | ✅ 優秀 |
| 適用場景 | 展示、測試 | 快速開發、除錯 |

---

## 🚁 Option 1: Hector Quadrotor + Gazebo（推薦）

### 啟動方式

```bash
# 設置環境
source /opt/ros/noetic/setup.bash
source ~/Fast-Planner/hector_ws/devel/setup.bash --extend
source ~/Fast-Planner/devel/setup.bash --extend

# 啟動系統
roslaunch plan_manage hector_fast_planner.launch
```

### 系統架構

```
Gazebo World (障礙物)
    ↓ Kinect 深度圖像
SDFMap (ESDF 建圖, pose_type=3)
    ↓ 佔據地圖 + 距離場
Fast-Planner (Kinodynamic A* + B-spline)
    ↓ 軌跡
Trajectory Server
    ↓ 位置/速度命令
Hector Command Bridge
    ↓ /cmd_vel
Hector Quadrotor Controller
    ↓
Gazebo 物理模擬
    ↓ /ground_truth/state
Fast-Planner (狀態反饋)
```

### 關鍵配置參數

文件: `fast_planner/plan_manage/launch/kino_algorithm_hector.xml`

| 參數 | 值 | 說明 |
|------|-----|------|
| `sdf_map/pose_type` | 3 | DEPTH_ODOM_INDEP 模式 |
| `sdf_map/obstacles_inflation` | 0.3 | 障礙物膨脹距離 |
| `optimization/dist0` | 0.8 | 安全距離閾值 |
| `optimization/lambda2` | 30.0 | 距離代價權重 |
| `manager/max_vel` | 1.0 | 最大速度 (m/s) |

### 技術細節

1. **pose_type=3 (DEPTH_ODOM_INDEP)**
   - 深度圖和里程計獨立訂閱
   - 避免 message_filters 時間戳同步問題
   - Hector 的深度圖和里程計時間戳不同步

2. **相機座標系轉換**
   - Kinect 光學座標系: Z 前, X 右, Y 下
   - 無人機座標系: X 前, Y 左, Z 上
   - 需要旋轉矩陣轉換

3. **Hector Command Bridge**
   - 將 Fast-Planner 位置命令轉換為速度命令
   - 使用 `/cmd_vel` 控制 Hector

---

## 🎮 Option 2: SO(3) 輕量模擬

### 啟動方式

```bash
cd ~/Fast-Planner
source devel/setup.bash
roslaunch plan_manage kino_replan.launch
```

### 系統架構

```
Random Map Generator (隨機障礙物)
    ↓ 全局點雲
Local Sensing (深度相機模擬)
    ↓ 局部點雲/深度圖
SDFMap (ESDF 建圖)
    ↓
Fast-Planner
    ↓ PositionCommand
SO(3) Controller (幾何控制器)
    ↓ SO3Command (力 + 姿態)
SO(3) Quadrotor Simulator
    ↓ Odometry
Fast-Planner (狀態反饋)
```

### 優勢

- **專為 Fast-Planner 設計**
- **高性能軌跡追蹤**
- **快速啟動**（無 Gazebo 開銷）
- **穩定控制**

---

## 🔧 調整建議

### 如果避障效果不佳

增加安全距離和代價權重：
```xml
<!-- kino_algorithm_hector.xml -->
<param name="sdf_map/obstacles_inflation" value="0.4"/>
<param name="optimization/dist0" value="1.0"/>
<param name="optimization/lambda2" value="50.0"/>
```

### 如果飛行太慢

增加速度限制：
```xml
<!-- hector_fast_planner.launch -->
<arg name="max_vel" default="1.5"/>
<arg name="max_acc" default="1.5"/>
```

### 如果控制不穩定

降低速度增益：
```xml
<!-- hector_fast_planner.launch 中的 hector_cmd_bridge -->
<param name="position_gain" value="0.8"/>
<param name="velocity_gain" value="0.8"/>
```

---

## 🐛 常見問題

### 問：無人機不動
**答**：檢查軌跡是否發布
```bash
rostopic echo /planning/bspline
```

### 問：路徑穿過障礙物
**答**：檢查 ESDF 是否正確建立
```bash
rostopic echo /sdf_map/occupancy -n 1
```

### 問：深度圖無數據
**答**：檢查 Kinect topic
```bash
rostopic hz /camera/depth/image_raw
```

### 問：無人機飛出邊界後崩潰
**答**：系統已加入邊界檢查，應自動跳過處理

### 問：RViz 中看不到軌跡
**答**：確認已添加 MarkerArray 顯示 `/planning/data_display`

---

## 📊 ROS Topics

### Hector 系統

| Topic | 類型 | 說明 |
|-------|------|------|
| `/ground_truth/state` | nav_msgs/Odometry | 無人機里程計 |
| `/camera/depth/image_raw` | sensor_msgs/Image | Kinect 深度圖 |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | Kinect 點雲 |
| `/cmd_vel` | geometry_msgs/Twist | 速度控制命令 |
| `/move_base_simple/goal` | geometry_msgs/PoseStamped | RViz 目標點 |
| `/planning/bspline` | plan_manage/Bspline | 規劃軌跡 |

---

## 📚 參考資料

- [Fast-Planner 論文](https://arxiv.org/abs/1904.05293)
- [SO(3) 控制論文](https://ieeexplore.ieee.org/document/5717652)
- [Hector Quadrotor Wiki](http://wiki.ros.org/hector_quadrotor)
- [原始 Fast-Planner GitHub](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
