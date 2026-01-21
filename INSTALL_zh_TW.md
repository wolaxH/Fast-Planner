# Fast-Planner 安裝指南

本指南詳細說明如何在 Ubuntu 系統上安裝並設置 Fast-Planner 與 Hector Quadrotor 整合環境。

---

## 📋 系統需求

### 作業系統

| 系統 | ROS 版本 | 狀態 |
|------|----------|------|
| Ubuntu 20.04 | ROS Noetic | ✅ 推薦 |
| Ubuntu 18.04 | ROS Melodic | ✅ 支援 |

### 硬體需求

| 項目 | 最低需求 | 建議配置 |
|------|----------|----------|
| CPU | 4 核心 | 8 核心以上 |
| RAM | 8 GB | 16 GB |
| 硬碟 | 20 GB 可用空間 | SSD 40 GB |
| GPU | 無（CPU 渲染） | NVIDIA GPU（CUDA 支援） |

---

## 🔧 相依套件

### 核心相依

| 套件 | 版本 | 用途 |
|------|------|------|
| ROS | Noetic/Melodic | 機器人作業系統 |
| Gazebo | 11 (Noetic) / 9 (Melodic) | 3D 物理模擬 |
| Eigen3 | 3.3+ | 線性代數運算 |
| PCL | 1.10+ | 點雲處理 |
| NLopt | 2.7.1 | 非線性優化 |
| Armadillo | 9.0+ | C++ 線性代數庫 |

### ROS 套件

```
roscpp, rospy, std_msgs, geometry_msgs, nav_msgs, sensor_msgs,
visualization_msgs, tf, cv_bridge, message_generation, dynamic_reconfigure
```

---

## 📥 安裝步驟

### 步驟 1：安裝 ROS

#### Ubuntu 20.04 (ROS Noetic)

```bash
# 設置 sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 設置金鑰
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 安裝 ROS
sudo apt update
sudo apt install ros-noetic-desktop-full

# 環境設置
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安裝建置工具
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# 初始化 rosdep
sudo rosdep init
rosdep update
```

#### Ubuntu 18.04 (ROS Melodic)

```bash
# 設置 sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 設置金鑰
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 安裝 ROS
sudo apt update
sudo apt install ros-melodic-desktop-full

# 環境設置
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安裝建置工具
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# 初始化 rosdep
sudo rosdep init
rosdep update
```

---

### 步驟 2：安裝相依套件

#### NLopt v2.7.1（必要）

```bash
cd ~
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

驗證安裝：
```bash
ls /usr/local/lib/libnlopt*
ls /usr/local/include/nlopt.h
```

#### Armadillo（必要）

```bash
sudo apt-get install libarmadillo-dev
```

#### 其他相依套件

```bash
# Eigen3
sudo apt-get install libeigen3-dev

# PCL
sudo apt-get install libpcl-dev

# OpenCV
sudo apt-get install libopencv-dev

# 額外 ROS 套件
sudo apt-get install ros-${ROS_DISTRO}-cv-bridge \
                     ros-${ROS_DISTRO}-tf \
                     ros-${ROS_DISTRO}-message-filters \
                     ros-${ROS_DISTRO}-image-transport
```

---

### 步驟 3：安裝 Hector Quadrotor（Gazebo 模擬用）

```bash
# 建立 Hector 工作空間
cd ~/Fast-Planner
mkdir -p hector_ws/src
cd hector_ws/src

# 下載 Hector Quadrotor 套件
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git

# 安裝相依
cd ~/Fast-Planner/hector_ws
rosdep install --from-paths src --ignore-src -r -y

# 編譯
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make
```

---

### 步驟 4：編譯 Fast-Planner

```bash
cd ~/Fast-Planner

# 初始化工作空間（如果尚未初始化）
./setup_workspace.sh

# 或手動編譯
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make
```

#### 編譯特定套件

```bash
catkin_make --pkg plan_manage
catkin_make --pkg plan_env
```

#### 清除並重新編譯

```bash
catkin_make clean
catkin_make
```

---

## ✅ 驗證安裝

### 測試 1：SO(3) 輕量模擬

```bash
cd ~/Fast-Planner
source devel/setup.bash
roslaunch plan_manage kino_replan.launch
```

預期結果：
- RViz 開啟並顯示隨機障礙物地圖
- 無人機模型出現在原點
- 可使用 "2D Nav Goal" 設置目標

### 測試 2：Hector + Gazebo 模擬

```bash
# 終端機 1
source /opt/ros/noetic/setup.bash
source ~/Fast-Planner/hector_ws/devel/setup.bash --extend
source ~/Fast-Planner/devel/setup.bash --extend
roslaunch plan_manage hector_fast_planner.launch
```

預期結果：
- Gazebo 開啟並顯示障礙物世界
- Hector 四旋翼出現在場景中
- RViz 顯示深度相機視角
- 可使用 "2D Nav Goal" 測試避障

---

## 🔧 選配：GPU 深度渲染

如需更真實的深度相機模擬，可啟用 CUDA 加速。

### 安裝 CUDA Toolkit

前往 [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) 下載並安裝。

### 修改 CMakeLists.txt

編輯 `uav_simulator/local_sensing/CMakeLists.txt`：

```cmake
set(ENABLE_CUDA true)

# 根據顯示卡設置 compute capability
# 查詢：https://developer.nvidia.com/cuda-gpus
set(CUDA_NVCC_FLAGS
  -gencode arch=compute_75,code=sm_75;  # RTX 20 系列
  # -gencode arch=compute_86,code=sm_86;  # RTX 30 系列
  # -gencode arch=compute_89,code=sm_89;  # RTX 40 系列
)
```

重新編譯：
```bash
cd ~/Fast-Planner
catkin_make clean
catkin_make
```

---

## ❗ 常見問題

### 問題 1：NLopt 找不到

**錯誤訊息：**
```
Could not find NLopt
```

**解決方案：**
```bash
# 確認 NLopt 已安裝
ls /usr/local/lib/libnlopt*

# 如未找到，重新安裝 NLopt v2.7.1
cd ~/nlopt/build
sudo make install

# 更新動態連結庫
sudo ldconfig
```

### 問題 2：Eigen 版本不相容

**錯誤訊息：**
```
static assertion failed: YOU_MIXED_DIFFERENT_NUMERIC_TYPES
```

**解決方案：**
```bash
# 檢查 Eigen 版本
pkg-config --modversion eigen3

# 確保使用系統 Eigen
sudo apt-get install --reinstall libeigen3-dev
```

### 問題 3：Gazebo 模型下載失敗

**錯誤訊息：**
```
[Err] [ModelDatabase.cc] Unable to download model
```

**解決方案：**
```bash
# 手動下載模型
cd ~/.gazebo
git clone https://github.com/osrf/gazebo_models.git models
```

### 問題 4：catkin_make 失敗

**解決方案：**
```bash
# 清除編譯快取
cd ~/Fast-Planner
rm -rf build devel
catkin_make
```

### 問題 5：RViz 顯示空白

**解決方案：**
```bash
# 檢查 Fixed Frame 設置
# 在 RViz 中將 Fixed Frame 改為 "world"

# 或重新設定 RViz 配置
roslaunch plan_manage rviz.launch
```

---

## 📁 目錄結構

安裝完成後的目錄結構：

```
~/Fast-Planner/
├── src/                    # ROS 套件來源連結
├── devel/                  # 編譯輸出（setup.bash 在此）
├── build/                  # 編譯中間檔案
├── fast_planner/           # Fast-Planner 核心模組
├── uav_simulator/          # SO(3) 模擬器
├── hector_ws/              # Hector Quadrotor 工作空間
│   ├── src/
│   ├── devel/
│   └── build/
└── *.md                    # 文件檔案
```

---

## 🚀 下一步

安裝完成後，請參閱：

- [README.md](README.md) - 快速啟動指南
- [SO3_SETUP.md](SO3_SETUP.md) - 模擬系統詳細說明
- [PROJECT_STRUCTURE_zh_TW.md](PROJECT_STRUCTURE_zh_TW.md) - 專案結構說明

---

## 📞 支援

如遇問題：
1. 先查閱本文件的「常見問題」章節
2. 搜尋 GitHub Issues
3. 提交新的 Issue（附上錯誤訊息和系統資訊）
