"""
ROS2 Bag适配器模块

这个模块负责从ROS2 bag文件中读取传感器数据，并转换为统一的数据格式。
它复用了现有的KITTI转换器代码，但采用了新的模块化架构。

主要功能：
1. 从ROS2 bag中读取图像、激光雷达和TF数据
2. 执行时间同步
3. 转换为统一的SensorData格式
4. 调用核心处理逻辑保存数据
"""

import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import logging

# ROS2相关导入（只在这个适配器中使用）
try:
    import rclpy
    from rclpy.time import Time as ROSTime
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    from sensor_msgs.msg import Image, LaserScan
    from geometry_msgs.msg import TransformStamped
    import tf2_ros
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError as e:
    ROS2_AVAILABLE = False
    ROS2_IMPORT_ERROR = str(e)

import numpy as np

# 导入统一数据结构和核心处理逻辑
from ..core_logic.structures import (
    SensorData, 
    CameraPose, 
    CameraIntrinsics,
    SynchronizedFrame,
    DatasetMetadata
)
from ..core_logic.processing import (
    save_image,
    save_lidar_bin,
    save_labels,
    save_calibration_info,
    create_kitti_directories,
    validate_output_directory
)
from ..utils.transformations import (
    quaternion_to_rotation_matrix,
    laserscan_to_pointcloud,
    transform_points
)


class ROS2BagAdapter:
    """
    ROS2 Bag数据适配器
    
    负责从ROS2 bag文件读取数据并转换为统一格式
    """
    
    def __init__(self, config: Dict):
        """
        初始化适配器
        
        Args:
            config (Dict): 配置字典，包含bag路径、话题名称等信息
            
        Raises:
            RuntimeError: ROS2库不可用时抛出
            ValueError: 配置参数不正确时抛出
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError(f"ROS2 libraries not available: {ROS2_IMPORT_ERROR}")
            
        self.config = config
        self.logger = self._setup_logger()
        
        # 验证必需的配置参数
        self._validate_config()
        
        # 初始化CV Bridge（ROS图像转换）
        self.bridge = CvBridge()
        
        # 初始化统计信息
        self.stats = {
            'total_image_msgs': 0,
            'total_lidar_msgs': 0, 
            'total_tf_msgs': 0,
            'processed_frames': 0,
            'skipped_frames': 0
        }
        
    def _setup_logger(self) -> logging.Logger:
        """设置日志记录器"""
        logger = logging.getLogger('ROS2BagAdapter')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            
        return logger
        
    def _validate_config(self):
        """验证配置参数"""
        required_params = [
            'rosbag_settings.path',
            'rosbag_settings.image_topic', 
            'rosbag_settings.lidar_topic',
            'rosbag_settings.camera_frame_id',
            'rosbag_settings.lidar_frame_id',
            'output_path'
        ]
        
        for param_path in required_params:
            if not self._get_nested_config(param_path):
                raise ValueError(f"Required config parameter '{param_path}' not found")
                
        # 验证bag文件是否存在
        bag_path = self.config['rosbag_settings']['path']
        if not os.path.exists(bag_path):
            raise FileNotFoundError(f"ROS bag file not found: {bag_path}")
            
        # 验证输出目录
        output_path = self.config['output_path']
        if not validate_output_directory(output_path):
            raise PermissionError(f"Cannot write to output directory: {output_path}")
            
    def _get_nested_config(self, param_path: str):
        """获取嵌套配置参数"""
        keys = param_path.split('.')
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return None
        return value
        
    def process_bag(self) -> Dict:
        """
        处理ROS2 bag文件的主函数
        
        Returns:
            Dict: 处理结果统计信息
            
        Raises:
            RuntimeError: 处理过程中出现错误时抛出
        """
        try:
            self.logger.info("Starting ROS2 bag processing...")
            
            # 创建输出目录结构
            directories = create_kitti_directories(self.config['output_path'])
            self.logger.info(f"Created output directories: {directories}")
            
            # 从bag文件读取消息
            messages = self._read_messages_from_bag()
            
            if not messages['image'] or not messages['lidar']:
                raise RuntimeError("No image or lidar messages found in bag file")
                
            # 构建TF缓存
            tf_buffer = self._populate_tf_buffer(messages['tf'], messages['tf_static'])
            
            # 处理同步数据
            self._process_synchronized_data(messages, tf_buffer, directories)
            
            # 生成数据集元数据
            metadata = self._generate_dataset_metadata()
            metadata_path = os.path.join(directories['metadata'], 'dataset_info.json')
            
            self.logger.info("ROS2 bag processing completed successfully")
            self.logger.info(f"Processed {self.stats['processed_frames']} frames")
            self.logger.info(f"Skipped {self.stats['skipped_frames']} frames")
            
            return {
                'success': True,
                'processed_frames': self.stats['processed_frames'],
                'skipped_frames': self.stats['skipped_frames'],
                'output_directories': directories,
                'metadata_path': metadata_path
            }
            
        except Exception as e:
            self.logger.error(f"Error during bag processing: {e}")
            return {
                'success': False,
                'error': str(e),
                'processed_frames': self.stats['processed_frames']
            }
            
    def _read_messages_from_bag(self) -> Dict[str, List]:
        """
        从bag文件中读取并分类消息
        
        Returns:
            Dict[str, List]: 分类后的消息字典
        """
        bag_path = self.config['rosbag_settings']['path']
        image_topic = self.config['rosbag_settings']['image_topic']  
        lidar_topic = self.config['rosbag_settings']['lidar_topic']
        
        self.logger.info(f"Reading messages from bag: {bag_path}")
        
        # 设置存储选项
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # 获取话题类型映射
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        self.logger.info(f"Available topics: {list(type_map.keys())}")
        
        # 初始化消息容器
        messages = {
            'image': [],
            'lidar': [],
            'tf': [],
            'tf_static': []
        }
        
        relevant_topics = [image_topic, lidar_topic, '/tf', '/tf_static']
        
        # 读取消息
        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            
            if topic in relevant_topics:
                try:
                    msg_type = get_message(type_map[topic])
                    msg = deserialize_message(data, msg_type)
                    
                    if topic == image_topic:
                        messages['image'].append((timestamp_ns, msg))
                    elif topic == lidar_topic:
                        messages['lidar'].append((timestamp_ns, msg))
                    elif topic == '/tf':
                        messages['tf'].extend([(timestamp_ns, transform) for transform in msg.transforms])
                    elif topic == '/tf_static':
                        messages['tf_static'].extend([(timestamp_ns, transform) for transform in msg.transforms])
                        
                except Exception as e:
                    self.logger.warning(f"Failed to deserialize message on topic {topic}: {e}")
                    continue
        
        # 按时间戳排序
        messages['image'].sort(key=lambda x: x[0])
        messages['lidar'].sort(key=lambda x: x[0])
        
        # 更新统计信息
        self.stats['total_image_msgs'] = len(messages['image'])
        self.stats['total_lidar_msgs'] = len(messages['lidar'])
        self.stats['total_tf_msgs'] = len(messages['tf']) + len(messages['tf_static'])
        
        self.logger.info(f"Read {self.stats['total_image_msgs']} image messages")
        self.logger.info(f"Read {self.stats['total_lidar_msgs']} lidar messages") 
        self.logger.info(f"Read {self.stats['total_tf_msgs']} TF messages")
        
        return messages
        
    def _populate_tf_buffer(self, tf_msgs: List, tf_static_msgs: List) -> tf2_ros.Buffer:
        """
        构建TF变换缓存
        
        Args:
            tf_msgs (List): 动态TF消息列表
            tf_static_msgs (List): 静态TF消息列表
            
        Returns:
            tf2_ros.Buffer: 填充好的TF缓存
        """
        tf_buffer = tf2_ros.Buffer()
        
        # 添加静态变换
        for timestamp_ns, transform in tf_static_msgs:
            try:
                tf_buffer.set_transform_static(transform, "bag_adapter")
            except Exception as e:
                self.logger.warning(f"Failed to add static transform: {e}")
                
        # 添加动态变换
        for timestamp_ns, transform in tf_msgs:
            try:
                tf_buffer.set_transform(transform, "bag_adapter")
            except Exception as e:
                self.logger.warning(f"Failed to add dynamic transform: {e}")
                
        return tf_buffer
        
    def _process_synchronized_data(self, messages: Dict, tf_buffer: tf2_ros.Buffer, 
                                 directories: Dict):
        """
        处理时间同步的传感器数据
        
        Args:
            messages (Dict): 分类后的消息字典
            tf_buffer (tf2_ros.Buffer): TF变换缓存
            directories (Dict): 输出目录字典
        """
        sync_tolerance_ns = self.config.get('sync_tolerance_s', 0.02) * 1e9
        camera_frame = self.config['rosbag_settings']['camera_frame_id']
        lidar_frame = self.config['rosbag_settings']['lidar_frame_id']
        
        self.logger.info(f"Processing synchronized data with {sync_tolerance_ns/1e6:.1f}ms tolerance")
        
        frame_idx = 0
        lidar_idx = 0
        
        for img_timestamp_ns, img_msg in messages['image']:
            try:
                # 寻找最近的激光雷达数据
                best_lidar_msg, best_lidar_timestamp, min_dt = self._find_closest_lidar(
                    img_timestamp_ns, messages['lidar'], lidar_idx, sync_tolerance_ns
                )
                
                if best_lidar_msg is None:
                    self.logger.warning(f"Frame {frame_idx}: No matching lidar data found")
                    self.stats['skipped_frames'] += 1
                    frame_idx += 1
                    continue
                
                # 查找相机到激光雷达的变换
                transform = self._lookup_transform(
                    tf_buffer, camera_frame, lidar_frame, img_timestamp_ns
                )
                
                if transform is None:
                    self.logger.warning(f"Frame {frame_idx}: Transform not available") 
                    self.stats['skipped_frames'] += 1
                    frame_idx += 1
                    continue
                
                # 处理并保存数据
                self._process_frame(
                    frame_idx, img_msg, best_lidar_msg, transform, directories
                )
                
                self.logger.debug(f"Processed frame {frame_idx}: dt={min_dt/1e6:.1f}ms")
                self.stats['processed_frames'] += 1
                frame_idx += 1
                
            except Exception as e:
                self.logger.error(f"Error processing frame {frame_idx}: {e}")
                self.stats['skipped_frames'] += 1
                frame_idx += 1
                continue
                
    def _find_closest_lidar(self, target_timestamp: int, lidar_msgs: List,
                           start_idx: int, tolerance_ns: int) -> Tuple[Optional[LaserScan], Optional[int], float]:
        """
        寻找与目标时间戳最接近的激光雷达数据
        
        Args:
            target_timestamp (int): 目标时间戳（纳秒）
            lidar_msgs (List): 激光雷达消息列表
            start_idx (int): 搜索起始索引
            tolerance_ns (int): 时间容忍度（纳秒）
            
        Returns:
            Tuple: (最佳激光雷达消息, 最佳时间戳, 最小时间差)
        """
        best_msg = None
        best_timestamp = None
        min_dt = float('inf')
        
        # 从start_idx开始搜索
        for i in range(start_idx, len(lidar_msgs)):
            lidar_timestamp, lidar_msg = lidar_msgs[i]
            dt = abs(target_timestamp - lidar_timestamp)
            
            if dt < min_dt:
                min_dt = dt
                best_msg = lidar_msg
                best_timestamp = lidar_timestamp
                
            # 如果激光雷达时间戳已经超过目标时间戳太多，停止搜索
            if lidar_timestamp > target_timestamp and dt > min_dt:
                break
                
        # 检查是否在容忍范围内
        if min_dt > tolerance_ns:
            return None, None, min_dt
            
        return best_msg, best_timestamp, min_dt
        
    def _lookup_transform(self, tf_buffer: tf2_ros.Buffer, target_frame: str, 
                         source_frame: str, timestamp_ns: int) -> Optional[TransformStamped]:
        """
        查找TF变换
        
        Args:
            tf_buffer (tf2_ros.Buffer): TF缓存
            target_frame (str): 目标坐标系
            source_frame (str): 源坐标系  
            timestamp_ns (int): 时间戳（纳秒）
            
        Returns:
            Optional[TransformStamped]: 变换信息，找不到时返回None
        """
        try:
            ros_time = ROSTime(nanoseconds=timestamp_ns)
            transform = tf_buffer.lookup_transform(target_frame, source_frame, ros_time)
            return transform
        except Exception as e:
            self.logger.debug(f"Transform lookup failed: {e}")
            return None
            
    def _process_frame(self, frame_id: int, img_msg: Image, lidar_msg: LaserScan,
                      transform: TransformStamped, directories: Dict):
        """
        处理单帧数据
        
        Args:
            frame_id (int): 帧编号
            img_msg (Image): 图像消息
            lidar_msg (LaserScan): 激光雷达消息
            transform (TransformStamped): 变换信息
            directories (Dict): 输出目录字典
        """
        # 转换图像为SensorData格式
        image_data = self._convert_ros_image(img_msg)
        
        # 转换激光雷达为点云
        point_cloud = self._convert_laser_scan(lidar_msg, transform)
        
        # 保存图像
        save_image(directories['images'], frame_id, image_data)
        
        # 保存点云
        save_lidar_bin(directories['velodyne'], frame_id, point_cloud)
        
        # 保存标定信息（如果需要）
        if self.config.get('save_calibration', True):
            camera_intrinsics = self._get_camera_intrinsics()
            if camera_intrinsics:
                save_calibration_info(directories['calib'], frame_id, camera_intrinsics)
                
    def _convert_ros_image(self, img_msg: Image) -> SensorData:
        """
        将ROS图像消息转换为SensorData
        
        Args:
            img_msg (Image): ROS图像消息
            
        Returns:
            SensorData: 统一格式的图像数据
        """
        # 使用CV Bridge转换图像
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        
        # 提取时间戳（转换为纳秒）
        timestamp_ns = img_msg.header.stamp.sec * 1_000_000_000 + img_msg.header.stamp.nanosec
        
        return SensorData(
            timestamp_ns=timestamp_ns,
            data=cv_image,
            metadata={
                'encoding': img_msg.encoding,
                'width': img_msg.width,
                'height': img_msg.height,
                'frame_id': img_msg.header.frame_id
            }
        )
        
    def _convert_laser_scan(self, lidar_msg: LaserScan, transform: TransformStamped) -> np.ndarray:
        """
        将激光雷达扫描转换为变换后的点云
        
        Args:
            lidar_msg (LaserScan): 激光雷达消息
            transform (TransformStamped): 坐标变换信息
            
        Returns:
            np.ndarray: 变换后的点云，形状为(N, 3)
        """
        # 将LaserScan转换为点云
        point_cloud = laserscan_to_pointcloud(
            ranges=np.array(lidar_msg.ranges),
            angle_min=lidar_msg.angle_min,
            angle_increment=lidar_msg.angle_increment,
            range_min=lidar_msg.range_min,
            range_max=lidar_msg.range_max
        )
        
        # 提取变换参数
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        
        rotation_matrix = quaternion_to_rotation_matrix(quaternion)
        
        # 应用坐标变换
        transformed_points = transform_points(point_cloud, translation, rotation_matrix)
        
        return transformed_points
        
    def _get_camera_intrinsics(self) -> Optional[CameraIntrinsics]:
        """
        获取相机内参（从配置文件）
        
        Returns:
            Optional[CameraIntrinsics]: 相机内参，如果配置不存在则返回None
        """
        intrinsics_config = self.config.get('camera_intrinsics')
        if not intrinsics_config:
            return None
            
        try:
            return CameraIntrinsics(
                fx=intrinsics_config['fx'],
                fy=intrinsics_config['fy'],
                cx=intrinsics_config['cx'],
                cy=intrinsics_config['cy'],
                width=intrinsics_config.get('width', 640),
                height=intrinsics_config.get('height', 480)
            )
        except KeyError as e:
            self.logger.warning(f"Missing camera intrinsic parameter: {e}")
            return None
            
    def _generate_dataset_metadata(self) -> Dict:
        """
        生成数据集元数据
        
        Returns:
            Dict: 数据集元数据字典
        """
        import time
        
        metadata = {
            'dataset_name': f"ROS2_Bag_Dataset_{int(time.time())}",
            'description': "Dataset generated from ROS2 bag file using multimodal data toolkit",
            'creation_time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'source_type': 'rosbag2',
            'source_file': self.config['rosbag_settings']['path'],
            'total_frames': self.stats['processed_frames'],
            'skipped_frames': self.stats['skipped_frames'],
            'sensor_topics': {
                'image': self.config['rosbag_settings']['image_topic'],
                'lidar': self.config['rosbag_settings']['lidar_topic']
            },
            'coordinate_frames': {
                'camera_frame': self.config['rosbag_settings']['camera_frame_id'],
                'lidar_frame': self.config['rosbag_settings']['lidar_frame_id']
            },
            'sync_tolerance_s': self.config.get('sync_tolerance_s', 0.02)
        }
        
        return metadata


def process_bag(config: Dict) -> Dict:
    """
    ROS2 Bag适配器的主要入口函数
    
    Args:
        config (Dict): 完整的配置字典
        
    Returns:
        Dict: 处理结果字典
        
    Raises:
        RuntimeError: ROS2不可用或处理失败时抛出
    """
    if not ROS2_AVAILABLE:
        raise RuntimeError(f"ROS2 libraries not available: {ROS2_IMPORT_ERROR}")
        
    adapter = ROS2BagAdapter(config)
    return adapter.process_bag()


# 兼容性函数（保持与原有代码的接口一致）
def main(bag_path: str, output_path: str, **kwargs) -> Dict:
    """
    简化的主函数接口，保持向后兼容
    
    Args:
        bag_path (str): ROS bag文件路径
        output_path (str): 输出目录路径
        **kwargs: 其他可选参数
        
    Returns:
        Dict: 处理结果字典
    """
    config = {
        'output_path': output_path,
        'rosbag_settings': {
            'path': bag_path,
            'image_topic': kwargs.get('image_topic', '/camera/image_raw'),
            'lidar_topic': kwargs.get('lidar_topic', '/scan'),
            'camera_frame_id': kwargs.get('camera_frame_id', 'camera_link'),
            'lidar_frame_id': kwargs.get('lidar_frame_id', 'base_scan')
        },
        'sync_tolerance_s': kwargs.get('sync_tolerance_s', 0.02)
    }
    
    return process_bag(config)
