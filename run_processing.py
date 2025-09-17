#!/usr/bin/env python3
"""
多模态数据工具链主执行脚本

这个脚本是整个工具链的主入口点，负责：
1. 解析命令行参数和配置文件
2. 根据配置选择合适的适配器
3. 执行数据处理流程
4. 报告处理结果

使用方法:
    python run_processing.py --config config/default_config.yaml
    python run_processing.py --config my_config.yaml --output /path/to/output
    python run_processing.py --source rosbag --bag-path /path/to/bag --output ./output
"""

import os
import sys
import argparse
import time
import logging
from pathlib import Path
from typing import Dict, Optional

# 添加当前目录到Python路径，以便导入toolkit模块
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False
    print("Warning: PyYAML not available. Configuration file support is limited.")

# 导入工具链组件
from toolkit.adapters import process_bag, process_carla
from toolkit.core_logic.processing import validate_output_directory


class MultimodalDataProcessor:
    """
    多模态数据处理器主类
    
    负责协调不同适配器的执行和结果管理
    """
    
    def __init__(self):
        self.logger = self._setup_logger()
        
    def _setup_logger(self) -> logging.Logger:
        """设置日志系统"""
        logger = logging.getLogger('MultimodalDataProcessor')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            # 控制台处理器
            console_handler = logging.StreamHandler()
            console_formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            console_handler.setFormatter(console_formatter)
            logger.addHandler(console_handler)
            
        return logger
        
    def load_config(self, config_path: str) -> Dict:
        """
        加载YAML配置文件
        
        Args:
            config_path (str): 配置文件路径
            
        Returns:
            Dict: 配置字典
            
        Raises:
            FileNotFoundError: 配置文件不存在
            ValueError: 配置文件格式错误
        """
        if not YAML_AVAILABLE:
            raise RuntimeError("PyYAML is required to load configuration files")
            
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
            
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                
            if not isinstance(config, dict):
                raise ValueError("Configuration file must contain a YAML dictionary")
                
            self.logger.info(f"Loaded configuration from: {config_path}")
            return config
            
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML configuration file: {e}")
            
    def validate_config(self, config: Dict) -> bool:
        """
        验证配置文件的完整性
        
        Args:
            config (Dict): 配置字典
            
        Returns:
            bool: 配置是否有效
        """
        required_keys = ['source_type', 'output_path']
        
        for key in required_keys:
            if key not in config:
                self.logger.error(f"Missing required configuration key: {key}")
                return False
                
        # 验证数据源类型
        source_type = config['source_type'].lower()
        if source_type not in ['rosbag', 'carla']:
            self.logger.error(f"Invalid source_type: {source_type}. Must be 'rosbag' or 'carla'")
            return False
            
        # 验证输出路径
        output_path = config['output_path']
        if not validate_output_directory(output_path):
            self.logger.error(f"Cannot write to output directory: {output_path}")
            return False
            
        # 根据数据源类型验证特定配置
        if source_type == 'rosbag':
            return self._validate_rosbag_config(config)
        elif source_type == 'carla':
            return self._validate_carla_config(config)
            
        return True
        
    def _validate_rosbag_config(self, config: Dict) -> bool:
        """验证ROS bag配置"""
        rosbag_settings = config.get('rosbag_settings', {})
        
        required_keys = ['path', 'image_topic', 'lidar_topic', 'camera_frame_id', 'lidar_frame_id']
        for key in required_keys:
            if key not in rosbag_settings:
                self.logger.error(f"Missing required rosbag setting: {key}")
                return False
                
        # 检查bag文件是否存在
        bag_path = rosbag_settings['path']
        if not os.path.exists(bag_path):
            self.logger.error(f"ROS bag file not found: {bag_path}")
            return False
            
        return True
        
    def _validate_carla_config(self, config: Dict) -> bool:
        """验证CARLA配置"""
        carla_settings = config.get('carla_settings', {})
        
        required_keys = ['host', 'port', 'world', 'sensor_definitions']
        for key in required_keys:
            if key not in carla_settings:
                self.logger.error(f"Missing required carla setting: {key}")
                return False
                
        # 验证传感器定义
        sensor_definitions = carla_settings['sensor_definitions']
        if 'camera' not in sensor_definitions or 'lidar' not in sensor_definitions:
            self.logger.error("Missing required sensor definitions (camera and lidar)")
            return False
            
        # 验证相机内参
        if 'camera_intrinsics' not in config:
            self.logger.error("Missing camera_intrinsics configuration")
            return False
            
        return True
        
    def process_data(self, config: Dict) -> Dict:
        """
        执行数据处理
        
        Args:
            config (Dict): 配置字典
            
        Returns:
            Dict: 处理结果
        """
        source_type = config['source_type'].lower()
        
        self.logger.info(f"Starting data processing with source type: {source_type}")
        start_time = time.time()
        
        try:
            if source_type == 'rosbag':
                result = self._process_rosbag(config)
            elif source_type == 'carla':
                result = self._process_carla(config)
            else:
                raise ValueError(f"Unsupported source type: {source_type}")
                
            # 添加总体统计信息
            end_time = time.time()
            result['total_processing_time_s'] = end_time - start_time
            result['config_used'] = config
            
            return result
            
        except Exception as e:
            self.logger.error(f"Data processing failed: {e}")
            return {
                'success': False,
                'error': str(e),
                'total_processing_time_s': time.time() - start_time
            }
            
    def _process_rosbag(self, config: Dict) -> Dict:
        """处理ROS bag数据"""
        self.logger.info("Processing ROS2 bag data...")
        
        try:
            return process_bag(config)
        except ImportError as e:
            error_msg = f"ROS2 libraries not available: {e}"
            self.logger.error(error_msg)
            return {'success': False, 'error': error_msg}
            
    def _process_carla(self, config: Dict) -> Dict:
        """处理CARLA仿真数据"""
        self.logger.info("Processing CARLA simulation data...")
        
        try:
            return process_carla(config)
        except ImportError as e:
            error_msg = f"CARLA library not available: {e}"
            self.logger.error(error_msg)
            return {'success': False, 'error': error_msg}
            
    def print_results(self, result: Dict):
        """打印处理结果摘要"""
        print("\n" + "="*80)
        print("处理结果摘要 / Processing Results Summary")
        print("="*80)
        
        if result['success']:
            print(f"✅ 处理状态: 成功 / Status: SUCCESS")
            print(f"📊 处理帧数: {result.get('frames_processed', 'N/A')} / Frames processed")
            
            if 'frames_collected' in result:
                print(f"📥 收集帧数: {result['frames_collected']} / Frames collected")
                
            if 'objects_detected' in result:
                print(f"🎯 检测物体数: {result['objects_detected']} / Objects detected")
                
            print(f"⏱️  处理耗时: {result.get('total_processing_time_s', 0):.2f}秒 / Processing time (seconds)")
            
            if 'output_directories' in result:
                print(f"📁 输出目录: / Output directories:")
                for name, path in result['output_directories'].items():
                    print(f"   {name}: {path}")
                    
        else:
            print(f"❌ 处理状态: 失败 / Status: FAILED")
            print(f"💥 错误信息: {result.get('error', 'Unknown error')} / Error")
            
        print("="*80)


def create_argument_parser():
    """创建命令行参数解析器"""
    parser = argparse.ArgumentParser(
        description="多模态数据工具链 - 跨平台数据收集与KITTI格式转换工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例 / Usage Examples:

1. 使用配置文件处理数据:
   python run_processing.py --config config/default_config.yaml

2. 处理ROS2 bag文件:
   python run_processing.py --source rosbag --bag-path /path/to/bag --output ./output

3. 连接CARLA仿真器:
   python run_processing.py --source carla --carla-host localhost --carla-port 2000 --output ./output

4. 显示详细帮助信息:
   python run_processing.py --help
        """
    )
    
    # 主要参数
    parser.add_argument(
        '--config', '-c',
        type=str,
        help='配置文件路径 (YAML格式) / Path to configuration file (YAML format)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        help='输出目录路径 / Output directory path'
    )
    
    # 数据源选择
    parser.add_argument(
        '--source', '-s',
        choices=['rosbag', 'carla'],
        help='数据源类型 / Data source type'
    )
    
    # ROS bag相关参数
    ros_group = parser.add_argument_group('ROS2 Bag选项 / ROS2 Bag Options')
    ros_group.add_argument(
        '--bag-path',
        type=str,
        help='ROS2 bag文件路径 / Path to ROS2 bag file'
    )
    ros_group.add_argument(
        '--image-topic',
        type=str,
        default='/camera/image_raw',
        help='图像话题名称 / Image topic name (default: /camera/image_raw)'
    )
    ros_group.add_argument(
        '--lidar-topic',
        type=str,
        default='/scan',
        help='激光雷达话题名称 / LiDAR topic name (default: /scan)'
    )
    
    # CARLA相关参数
    carla_group = parser.add_argument_group('CARLA选项 / CARLA Options')
    carla_group.add_argument(
        '--carla-host',
        type=str,
        default='localhost',
        help='CARLA服务器地址 / CARLA server host (default: localhost)'
    )
    carla_group.add_argument(
        '--carla-port',
        type=int,
        default=2000,
        help='CARLA服务器端口 / CARLA server port (default: 2000)'
    )
    carla_group.add_argument(
        '--carla-world',
        type=str,
        default='Town03',
        help='CARLA世界名称 / CARLA world name (default: Town03)'
    )
    
    # 其他选项
    parser.add_argument(
        '--max-frames',
        type=int,
        default=1000,
        help='最大处理帧数 / Maximum frames to process (default: 1000)'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='显示详细日志 / Enable verbose logging'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='仅验证配置，不执行处理 / Only validate configuration, do not process'
    )
    
    return parser


def build_config_from_args(args) -> Optional[Dict]:
    """从命令行参数构建配置字典"""
    if not args.source:
        return None
        
    config = {
        'source_type': args.source,
        'output_path': args.output or './output',
        'general_settings': {
            'max_frames': args.max_frames
        }
    }
    
    if args.source == 'rosbag':
        if not args.bag_path:
            raise ValueError("--bag-path is required when source is 'rosbag'")
            
        config['rosbag_settings'] = {
            'path': args.bag_path,
            'image_topic': args.image_topic,
            'lidar_topic': args.lidar_topic,
            'camera_frame_id': 'camera_link',
            'lidar_frame_id': 'base_scan'
        }
        
    elif args.source == 'carla':
        config['carla_settings'] = {
            'host': args.carla_host,
            'port': args.carla_port,
            'world': args.carla_world,
            'sensor_definitions': {
                'camera': {
                    'type': 'sensor.camera.rgb',
                    'image_size_x': 1242,
                    'image_size_y': 375,
                    'fov': 90,
                    'transform': {'x': 1.5, 'y': 0.0, 'z': 2.4, 'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0}
                },
                'lidar': {
                    'type': 'sensor.lidar.ray_cast',
                    'range': 85.0,
                    'points_per_second': 130000,
                    'rotation_frequency': 20,
                    'upper_fov': 10.0,
                    'lower_fov': -30.0,
                    'channels': 64,
                    'transform': {'x': 1.5, 'y': 0.0, 'z': 2.4, 'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0}
                }
            }
        }
        config['camera_intrinsics'] = {
            'fx': 621.0, 'fy': 621.0, 'cx': 621.0, 'cy': 187.5,
            'width': 1242, 'height': 375
        }
        
    return config


def main():
    """主函数"""
    # 解析命令行参数
    parser = create_argument_parser()
    args = parser.parse_args()
    
    # 创建数据处理器
    processor = MultimodalDataProcessor()
    
    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    try:
        # 加载配置
        config = None
        if args.config:
            # 从配置文件加载
            config = processor.load_config(args.config)
            # 命令行参数覆盖配置文件参数
            if args.output:
                config['output_path'] = args.output
            if args.max_frames:
                config.setdefault('general_settings', {})['max_frames'] = args.max_frames
        else:
            # 从命令行参数构建配置
            config = build_config_from_args(args)
            if config is None:
                print("错误: 必须提供 --config 或 --source 参数")
                print("Error: Either --config or --source must be provided")
                parser.print_help()
                return 1
        
        # 验证配置
        if not processor.validate_config(config):
            print("错误: 配置验证失败")
            print("Error: Configuration validation failed")
            return 1
            
        # 如果是dry-run，仅显示配置并退出
        if args.dry_run:
            print("配置验证成功 / Configuration validation successful")
            print(f"数据源类型 / Source type: {config['source_type']}")
            print(f"输出路径 / Output path: {config['output_path']}")
            return 0
        
        # 执行数据处理
        result = processor.process_data(config)
        
        # 显示结果
        processor.print_results(result)
        
        # 返回状态码
        return 0 if result['success'] else 1
        
    except KeyboardInterrupt:
        print("\n用户中断处理 / Processing interrupted by user")
        return 130
    except Exception as e:
        processor.logger.error(f"Unexpected error: {e}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
