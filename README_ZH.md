# Go2-W 自主导航与视觉抓取系统 (go2w-nav-voice-grasping)

本项目基于 **宇树科技 (Unitree) Go2-W 轮履/四足机器人** 与 **D1 机械臂**，集成了语音交互指令解析、3D 激光 SLAM 自主导航以及基于深度视觉的端到端抓取与取物功能。

---

## 📌 系统架构与特性

- **🎙️ 语音控制与指令解析**：支持离线/在线语音识别与 LLM 自然语言理解，可将自然语言指令转化为导航与操作任务队列。
- **🗺️ 3D SLAM 与自主导航**：基于多传感器融合（3D 激光雷达 + IMU），实现高精度室内/半户外环境建图、全局路径规划与动态避障。
- **👁️ 3D 视觉目标识别**：集成 YOLO 目标检测与 RGB-D 点云分割，实现目标物体的 6D 姿态估计与精确定位。
- **🦾 机械臂轨迹规划与抓取**：结合 MoveIt 与逆运动学求解器，实现 D1 机械臂的高精度自主平滑抓取与放置。

---

## 🛠️ 硬件与系统要求

### 硬件环境
| 组件 | 规格/型号 | 作用 |
| :--- | :--- | :--- |
| 机器人底盘 | Unitree Go2-W | 轮足一体移动平台 |
| 机械臂 | Unitree D1 Arm | 6自由度/7自由度机械臂 |
| 机载计算单元 | NVIDIA Jetson Orin / 工业主板 | 运行 SLAM、感知与控制算法 |
| 深度相机 | RealSense D435i / Orbbec | 抓取视觉感知 |
| 麦克风阵列 | 多麦克风阵列 / 离线语音模块 | 语音采集 |

### 软件环境
- **操作系统**: Ubuntu 20.04 LTS / Ubuntu 22.04 LTS
- **ROS 版本**: ROS 1 (Noetic) / ROS 2 (Humble)
- **环境依赖**:
  - Python 3.8+
  - CUDA 11.x / 12.x & TensorRT
  - OpenCV, PCL (Point Cloud Library)
  - MoveIt / MoveIt2

---

## 📁 目录结构

```text
go2w-nav-voice-grasping/
├── docs/                   # 项目说明文档与架构图
├── config/                 # 导航、感知与机械臂配置文件
├── scripts/                # 启动脚本与环境配置工具
├── src/
│   ├── go2w_voice/         # 语音识别、TTS 与语义解析节点
│   ├── go2w_navigation/    # 3D SLAM、建图与 Nav2 导航节点
│   ├── go2w_perception/   # YOLO 目标检测与点云位姿估计节点
│   ├── go2w_manipulation/  # D1 机械臂 MoveIt 控制与抓取规划
│   └── go2w_bringup/       # 系统主控与系统 Launch 启动文件
├── README.md               # 项目主说明文档
└── .gitignore              # Git 忽略文件规则
```

---

## 🚀 快速开始

### 1. 克隆代码库

```bash
mkdir -p ~/go2w_ws/src
cd ~/go2w_ws/src
git clone https://github.com/YOUR_GITHUB_USERNAME/go2w-nav-voice-grasping.git
```

### 2. 安装依赖项

安装 ROS 核心依赖与第三方 Python 依赖：

```bash
cd ~/go2w_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 安装 Python 深度学习与感知相关依赖
pip install -r src/go2w-nav-voice-grasping/requirements.txt
```

### 3. 编译工作空间

```bash
cd ~/go2w_ws

# 若使用 ROS 1
catkin_make -DCMAKE_BUILD_TYPE=Release

# 若使用 ROS 2
colcon build --symlink-install
```

---

## 💻 运行与使用

### 1. 启动底盘与硬件接口

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_bringup robot_hardware.launch
```

### 2. 启动自主导航与建图

```bash
# 启动 SLAM 建图模式
roslaunch go2w_navigation slam.launch

# 或启动已知地图导航模式
roslaunch go2w_navigation navigation.launch map:=/path/to/your/map.yaml
```

### 3. 启动视觉抓取与语音交互主节点

```bash
roslaunch go2w_bringup system_main.launch
```

---

## ⚠️ 注意事项与常见问题

1. **大文件与权重文件**：
   - 训练好的 YOLO 模型权重（如 `.pt`、`.engine`、`.onnx`）默认已被 `.gitignore` 过滤，请确保将权重放置于 `src/go2w_perception/models/` 目录中后再运行。
2. **硬件安全**：
   - 在首次测试机械臂抓取与机器人自主移动时，请务必保证在**空旷环境**下进行，并随时准备按下**急停开关**。

---

## 📄 许可证

本项目采用 [MIT License](LICENSE) 开源许可证。