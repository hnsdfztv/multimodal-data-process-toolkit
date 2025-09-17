# 多模态数据工具链 / Multimodal Data Toolkit

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg)](https://docs.ros.org/en/humble/)

一个跨平台的模块化Python工具链，用于从不同数据源（ROS2 Bag文件和CARLA仿真器）收集同步的多模态传感器数据，并转换为标准的KITTI数据集格式。

A cross-platform, modular Python toolchain for collecting synchronized multimodal sensor data from different sources (ROS2 Bag files and CARLA simulator) and converting them to standard KITTI dataset format.

## ✨ 主要特性 / Key Features

- 🔄 **跨平台适配器架构**：支持ROS2 Bag文件和CARLA仿真器
- 📊 **统一数据格式**：平台无关的内部数据表示
- 🎯 **KITTI标准格式**：自动生成KITTI格式的数据集
- ⏱️ **时间同步**：高精度的多传感器数据同步
- 🏷️ **真值标注**：自动生成3D物体检测标注
- ⚙️ **高度可配置**：YAML配置文件驱动
- 🔧 **模块化设计**：核心逻辑与平台适配器分离

## 📋 系统要求 / Requirements

### 基础环境 / Base Environment
- **Python**: 3.8+
- **操作系统**: Ubuntu 20.04+ / Windows 10+ / macOS 10.15+

### ROS2支持 / ROS2 Support
- **ROS2**: Humble (推荐) 或更新版本
- **依赖包**: `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `cv_bridge`, `rosbag2_py`

### CARLA支持 / CARLA Support  
- **CARLA**: 0.9.13+ (推荐最新稳定版)
- **Python API**: 包含在CARLA安装中

### Python依赖 / Python Dependencies
```bash
pip install -r requirements.txt
```

## 🚀 快速开始 / Quick Start

### 1. 安装 / Installation

```bash
# 克隆仓库 / Clone repository
git clone https://github.com/hnsdfztv/multimodal-data-toolkit.git
cd multimodal-data-toolkit

# 安装Python依赖 / Install Python dependencies
pip install -r requirements.txt

# 构建ROS2包 (如果在ROS2工作区中)
# Build ROS2 package (if in ROS2 workspace)
colcon build --packages-select multimodal_data_toolkit
source install/setup.bash
```

### 2. 配置 / Configuration

编辑 `config/default_config.yaml` 文件，设置数据源和输出路径：

```yaml
# 选择数据源：'rosbag' 或 'carla'
source_type: 'carla'

# 设置输出路径
output_path: "/path/to/your/kitti_dataset_output"

# 配置CARLA设置 (如果使用CARLA)
carla_settings:
  host: "localhost"
  port: 2000
  world: "Town03"
  # ... 更多配置选项

# 配置ROS Bag设置 (如果使用ROS Bag)  
rosbag_settings:
  path: "/path/to/your/rosbag"
  image_topic: "/camera/image_raw"
  lidar_topic: "/scan"
  # ... 更多配置选项
```

### 3. 运行 / Usage

#### 方法1：使用配置文件 / Method 1: Using Configuration File
```bash
python run_processing.py --config config/default_config.yaml
```

#### 方法2：命令行参数 / Method 2: Command Line Arguments

**处理ROS2 Bag文件:**
```bash
python run_processing.py --source rosbag \
    --bag-path /path/to/your/bag \
    --output ./kitti_dataset
```

**连接CARLA仿真器:**
```bash  
python run_processing.py --source carla \
    --carla-host localhost \
    --carla-port 2000 \
    --output ./kitti_dataset
```

## 📁 项目结构 / Project Structure

```
multimodal_data_toolkit/
├── config/                          # 配置文件 / Configuration files
│   └── default_config.yaml         # 默认配置 / Default configuration
├── toolkit/                         # 核心工具包 / Core toolkit
│   ├── __init__.py
│   ├── adapters/                    # 平台适配器 / Platform adapters
│   │   ├── __init__.py
│   │   ├── ros2_bag_adapter.py     # ROS2 Bag适配器
│   │   └── carla_adapter.py        # CARLA适配器
│   ├── core_logic/                  # 核心处理逻辑 / Core processing logic
│   │   ├── __init__.py
│   │   ├── structures.py           # 统一数据结构 / Unified data structures
│   │   └── processing.py           # 处理函数 / Processing functions
│   └── utils/                       # 工具函数 / Utility functions
│       ├── __init__.py
│       └── transformations.py      # 坐标变换 / Coordinate transformations
├── run_processing.py               # 主执行脚本 / Main execution script
├── setup.py                       # Python包设置 / Python package setup
├── package.xml                    # ROS2包描述 / ROS2 package description
├── requirements.txt               # Python依赖 / Python dependencies
└── README.md                     # 本文件 / This file
```

## 🔧 配置说明 / Configuration Guide

### 基础配置 / Basic Configuration

```yaml
# 数据源类型 / Data source type
source_type: 'carla'  # 'rosbag' 或 'carla'

# 输出路径 / Output path
output_path: "/path/to/output"

# 通用设置 / General settings  
general_settings:
  sync_tolerance_s: 0.02        # 时间同步容忍度
  generate_annotations: true    # 是否生成标注
  max_frames: 1000             # 最大处理帧数
```

### ROS2 Bag配置 / ROS2 Bag Configuration

```yaml
rosbag_settings:
  path: "/path/to/rosbag"
  image_topic: "/camera/image_raw"
  lidar_topic: "/scan"
  camera_frame_id: "camera_link"
  lidar_frame_id: "base_scan"
```

### CARLA配置 / CARLA Configuration

```yaml
carla_settings:
  host: "localhost"
  port: 2000
  world: "Town03"
  
  sensor_definitions:
    camera:
      type: "sensor.camera.rgb"
      image_size_x: 1242
      image_size_y: 375
      fov: 90
      transform:
        x: 1.5
        y: 0.0  
        z: 2.4
    
    lidar:
      type: "sensor.lidar.ray_cast"
      range: 85.0
      channels: 64
      # ... 更多传感器参数
```

### 相机内参配置 / Camera Intrinsics Configuration

```yaml
camera_intrinsics:
  fx: 621.0
  fy: 621.0
  cx: 621.0
  cy: 187.5
  width: 1242
  height: 375
```

## 📊 输出格式 / Output Format

工具链生成标准的KITTI数据集格式：

```
output_directory/
├── images/                    # RGB图像 / RGB images
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── velodyne/                  # 点云数据 / Point cloud data
│   ├── 000000.bin  
│   ├── 000001.bin
│   └── ...
├── labels/                    # 3D标注 / 3D annotations
│   ├── 000000.txt
│   ├── 000001.txt
│   └── ...
├── calib/                     # 标定信息 / Calibration info
│   ├── 000000.txt
│   ├── 000001.txt
│   └── ...
└── metadata/                  # 元数据 / Metadata
    └── dataset_info.json
```

## 🔍 使用示例 / Usage Examples

### 示例1：处理TurtleBot3仿真数据 / Example 1: Processing TurtleBot3 Simulation Data

```bash
# 1. 启动TurtleBot3 Gazebo仿真 / Start TurtleBot3 Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 2. 录制数据包 / Record data bag
ros2 bag record /camera/image_raw /scan /tf /tf_static

# 3. 处理数据包 / Process data bag
python run_processing.py --source rosbag \
    --bag-path ./rosbag2_xxx \
    --image-topic /camera/image_raw \
    --lidar-topic /scan \
    --output ./turtlebot3_kitti_dataset
```

### 示例2：CARLA自动驾驶数据收集 / Example 2: CARLA Autonomous Driving Data Collection

```bash
# 1. 启动CARLA服务器 / Start CARLA server
./CarlaUE4.sh -quality-level=Low

# 2. 使用配置文件收集数据 / Collect data using config file
python run_processing.py --config config/carla_town03_config.yaml

# 3. 或使用命令行参数 / Or use command line arguments
python run_processing.py --source carla \
    --carla-host localhost \
    --carla-port 2000 \
    --carla-world Town03 \
    --max-frames 5000 \
    --output ./carla_kitti_dataset
```

## 🛠️ 开发指南 / Development Guide

### 添加新的数据源适配器 / Adding New Data Source Adapter

1. 在 `toolkit/adapters/` 中创建新的适配器文件
2. 实现适配器类，继承基础接口
3. 转换数据为统一的 `SensorData` 和 `GroundTruthObject` 格式
4. 在主脚本中注册新适配器

### 扩展核心处理逻辑 / Extending Core Processing Logic

1. 在 `toolkit/core_logic/processing.py` 中添加新函数
2. 确保函数只使用统一数据结构，不依赖特定平台
3. 添加适当的错误处理和日志记录
4. 编写单元测试

### 自定义坐标变换 / Custom Coordinate Transformations

在 `toolkit/utils/transformations.py` 中添加新的变换函数：

```python
def custom_transform(points: np.ndarray, params: Dict) -> np.ndarray:
    """自定义坐标变换函数"""
    # 实现变换逻辑
    return transformed_points
```

## 🐛 故障排除 / Troubleshooting

### 常见问题 / Common Issues

1. **ROS2导入错误** / ROS2 Import Errors
   ```bash
   # 确保ROS2环境已正确source
   source /opt/ros/humble/setup.bash
   ```

2. **CARLA连接失败** / CARLA Connection Failed
   ```bash
   # 检查CARLA服务器是否运行
   ./CarlaUE4.sh
   # 检查防火墙设置
   ```

3. **权限错误** / Permission Errors
   ```bash
   # 确保输出目录有写权限
   chmod 755 /path/to/output/directory
   ```

4. **内存不足** / Out of Memory
   ```yaml
   # 在配置文件中减少max_frames
   general_settings:
     max_frames: 500
   ```

### 调试模式 / Debug Mode

启用详细日志和调试信息：

```bash
python run_processing.py --config config.yaml --verbose
```

或在配置文件中设置：

```yaml
debug:
  enable_debug: true
  save_intermediate_results: true
```

## 📈 性能优化 / Performance Optimization

### 建议的系统配置 / Recommended System Configuration

- **CPU**: 4核心以上 / 4+ cores
- **内存**: 8GB以上 / 8GB+ RAM  
- **存储**: SSD硬盘 / SSD storage
- **GPU**: 可选，CARLA仿真推荐 / Optional, recommended for CARLA

### 性能调优提示 / Performance Tuning Tips

1. **减少处理帧数**：设置合理的 `max_frames` 值
2. **并行处理**：使用多进程处理大型数据集
3. **内存管理**：定期清理中间数据
4. **存储优化**：使用高速SSD存储

## 🤝 贡献指南 / Contributing

我们欢迎各种形式的贡献！请查看以下指南：

1. Fork项目仓库
2. 创建特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建Pull Request

### 代码风格 / Code Style

我们使用以下工具确保代码质量：

```bash
# 代码格式化
black .

# 代码检查
flake8 .

# 类型检查  
mypy toolkit/
```

## 📄 许可证 / License

本项目基于MIT许可证开源。详情请查看 [LICENSE](LICENSE) 文件。

## 📞 联系我们 / Contact

- **邮箱** / Email: your.email@example.com
- **问题反馈** / Issues: [GitHub Issues](https://github.com/your-username/multimodal-data-toolkit/issues)
- **讨论区** / Discussions: [GitHub Discussions](https://github.com/your-username/multimodal-data-toolkit/discussions)

## 🙏 致谢 / Acknowledgments

- [KITTI数据集](http://www.cvlibs.net/datasets/kitti/) - 标准数据格式参考
- [ROS2](https://docs.ros.org/en/humble/) - 机器人操作系统支持
- [CARLA仿真器](https://carla.org/) - 自动驾驶仿真平台
- [OpenCV](https://opencv.org/) - 计算机视觉库
- [NumPy](https://numpy.org/) - 数值计算支持

---

**注意**: 这是一个活跃开发中的项目。如果您遇到任何问题或有改进建议，请随时联系我们或提交Issue。

**Note**: This is an actively developed project. If you encounter any issues or have suggestions for improvement, please feel free to contact us or submit an Issue.
