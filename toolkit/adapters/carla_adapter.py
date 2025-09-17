"""
CARLA仿真适配器模块

这个模块负责连接CARLA仿真器，收集同步的传感器数据和真值信息，
并转换为统一的数据格式。

主要功能：
1. 连接CARLA服务器并设置仿真环境
2. 创建车辆并附加相机和激光雷达传感器
3. 使用同步模式收集完美同步的传感器数据
4. 提取真值物体信息
5. 转换为统一数据格式并调用核心处理逻辑
"""

import os
import time
import random
from typing import Dict, List, Tuple, Optional
import logging
import threading
import queue

# CARLA相关导入（只在这个适配器中使用）
try:
    import carla
    import numpy as np
    CARLA_AVAILABLE = True
except ImportError as e:
    CARLA_AVAILABLE = False
    CARLA_IMPORT_ERROR = str(e)

# 导入统一数据结构和核心处理逻辑
from ..core_logic.structures import (
    SensorData,
    GroundTruthObject, 
    CameraPose,
    CameraIntrinsics,
    SynchronizedFrame,
    DatasetMetadata,
    KITTI_CLASS_MAPPING
)
from ..core_logic.processing import (
    save_image,
    save_lidar_bin,
    save_labels,
    save_calibration_info,
    generate_kitti_annotations,
    create_kitti_directories,
    validate_output_directory
)
from ..utils.transformations import (
    euler_to_rotation_matrix
)


class CARLAAdapter:
    """
    CARLA仿真数据适配器
    
    负责从CARLA仿真器收集数据并转换为统一格式
    """
    
    def __init__(self, config: Dict):
        """
        初始化适配器
        
        Args:
            config (Dict): 配置字典，包含CARLA连接参数、传感器配置等
            
        Raises:
            RuntimeError: CARLA库不可用时抛出
            ValueError: 配置参数不正确时抛出
        """
        if not CARLA_AVAILABLE:
            raise RuntimeError(f"CARLA library not available: {CARLA_IMPORT_ERROR}")
            
        self.config = config
        self.logger = self._setup_logger()
        
        # 验证配置参数
        self._validate_config()
        
        # CARLA相关对象
        self.client = None
        self.world = None
        self.vehicle = None
        self.camera_sensor = None
        self.lidar_sensor = None
        
        # 数据收集相关
        self.sensor_data = {
            'camera': queue.Queue(),
            'lidar': queue.Queue()
        }
        self.frame_count = 0
        self.is_collecting = False
        
        # 统计信息
        self.stats = {
            'frames_collected': 0,
            'frames_processed': 0,
            'objects_detected': 0,
            'simulation_time': 0.0
        }
        
    def _setup_logger(self) -> logging.Logger:
        """设置日志记录器"""
        logger = logging.getLogger('CARLAAdapter')
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
            'carla_settings.host',
            'carla_settings.port', 
            'carla_settings.world',
            'carla_settings.sensor_definitions.camera',
            'carla_settings.sensor_definitions.lidar',
            'camera_intrinsics',
            'output_path'
        ]
        
        for param_path in required_params:
            if not self._get_nested_config(param_path):
                raise ValueError(f"Required config parameter '{param_path}' not found")
                
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
        
    def process_carla(self) -> Dict:
        """
        处理CARLA仿真数据的主函数
        
        Returns:
            Dict: 处理结果统计信息
        """
        try:
            self.logger.info("Starting CARLA data collection...")
            
            # 创建输出目录结构
            directories = create_kitti_directories(self.config['output_path'])
            self.logger.info(f"Created output directories: {directories}")
            
            # 连接CARLA并设置仿真环境
            self._connect_to_carla()
            self._setup_simulation()
            
            # 收集数据
            max_frames = self.config.get('max_frames', 1000)
            self._collect_data(max_frames, directories)
            
            # 清理资源
            self._cleanup()
            
            # 生成数据集元数据
            metadata = self._generate_dataset_metadata()
            
            self.logger.info("CARLA data collection completed successfully")
            self.logger.info(f"Collected {self.stats['frames_processed']} frames")
            self.logger.info(f"Detected {self.stats['objects_detected']} objects")
            
            return {
                'success': True,
                'frames_collected': self.stats['frames_collected'],
                'frames_processed': self.stats['frames_processed'],
                'objects_detected': self.stats['objects_detected'],
                'simulation_time': self.stats['simulation_time'],
                'output_directories': directories,
                'metadata': metadata
            }
            
        except Exception as e:
            self.logger.error(f"Error during CARLA data collection: {e}")
            self._cleanup()
            return {
                'success': False,
                'error': str(e),
                'frames_processed': self.stats['frames_processed']
            }
            
    def _connect_to_carla(self):
        """连接到CARLA服务器"""
        host = self.config['carla_settings']['host']
        port = self.config['carla_settings']['port']
        
        self.logger.info(f"Connecting to CARLA server at {host}:{port}")
        
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(10.0)
            
            # 获取服务器版本信息
            version = self.client.get_server_version()
            self.logger.info(f"Connected to CARLA server version: {version}")
            
        except Exception as e:
            raise RuntimeError(f"Failed to connect to CARLA server: {e}")
            
    def _setup_simulation(self):
        """设置仿真环境"""
        world_name = self.config['carla_settings']['world']
        
        try:
            # 加载指定的世界
            self.logger.info(f"Loading world: {world_name}")
            self.world = self.client.load_world(world_name)
            
            # 设置同步模式
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.1  # 10 FPS，给传感器更多时间
            settings.no_rendering_mode = False  # 确保渲染模式开启
            self.world.apply_settings(settings)
            
            self.logger.info(f"Applied synchronous mode with {settings.fixed_delta_seconds}s delta")
            
            # 设置交通管理器为同步模式
            traffic_manager = self.client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
            
            # 创建车辆
            self._spawn_vehicle()
            
            # 创建传感器
            self._setup_sensors()
            
            self.logger.info("Simulation environment setup complete")
            
        except Exception as e:
            raise RuntimeError(f"Failed to setup simulation: {e}")
            
    def _spawn_vehicle(self):
        """在世界中生成车辆"""
        try:
            # 获取车辆蓝图
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bps = blueprint_library.filter('vehicle.*')
            if not vehicle_bps:
                raise RuntimeError("No vehicle blueprints available")
            
            # 选择一个常见的车辆类型
            vehicle_bp = None
            preferred_vehicles = ['vehicle.tesla.model3', 'vehicle.audi.a2', 'vehicle.bmw.grandtourer']
            for pref_vehicle in preferred_vehicles:
                found_bp = blueprint_library.find(pref_vehicle)
                if found_bp:
                    vehicle_bp = found_bp
                    break
            
            if vehicle_bp is None:
                vehicle_bp = vehicle_bps[0]  # 使用第一个可用车辆
            
            self.logger.info(f"Using vehicle blueprint: {vehicle_bp.id}")
            
            # 获取生成点
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError("No spawn points available in the world")
                
            # 尝试多个生成点，直到成功生成车辆
            max_attempts = 10
            for attempt in range(max_attempts):
                spawn_point = random.choice(spawn_points)
                try:
                    self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                    if self.vehicle is not None:
                        break
                except Exception as spawn_error:
                    self.logger.warning(f"Spawn attempt {attempt + 1} failed at {spawn_point.location}: {spawn_error}")
                    if attempt == max_attempts - 1:
                        raise RuntimeError(f"Failed to spawn vehicle after {max_attempts} attempts")
                    continue
            
            if self.vehicle is None:
                raise RuntimeError("Vehicle spawn returned None")
                
            # 验证车辆确实存在于世界中
            self.world.tick()  # 确保车辆已添加到世界
            time.sleep(0.1)    # 给系统时间处理
            
            # 检查车辆是否仍然存在
            all_vehicles = self.world.get_actors().filter('vehicle.*')
            vehicle_exists = any(v.id == self.vehicle.id for v in all_vehicles)
            if not vehicle_exists:
                raise RuntimeError("Vehicle was spawned but disappeared from world")
            
            self.logger.info(f"Successfully spawned vehicle (ID: {self.vehicle.id}) at {spawn_point.location}")
            
            # 启用自动驾驶（可选）
            if self.config.get('enable_autopilot', True):
                self.vehicle.set_autopilot(True)
                self.logger.info("Autopilot enabled")
                
        except Exception as e:
            raise RuntimeError(f"Failed to spawn vehicle: {e}")
            
    def _setup_sensors(self):
        """设置传感器"""
        try:
            blueprint_library = self.world.get_blueprint_library()
            
            # 设置相机传感器
            self._setup_camera_sensor(blueprint_library)
            
            # 设置激光雷达传感器
            self._setup_lidar_sensor(blueprint_library)
            
            # 等待传感器初始化完成
            self.logger.info("Waiting for sensors to initialize...")
            time.sleep(2.0)  # 给传感器足够的初始化时间
            
            # 验证传感器是否正确生成
            all_sensors = self.world.get_actors().filter('sensor.*')
            camera_exists = any(s.id == self.camera_sensor.id for s in all_sensors)
            lidar_exists = any(s.id == self.lidar_sensor.id for s in all_sensors)
            
            if not camera_exists:
                raise RuntimeError("Camera sensor not found in world after creation")
            if not lidar_exists:
                raise RuntimeError("LiDAR sensor not found in world after creation")
                
            self.logger.info(f"Sensors verified - Camera ID: {self.camera_sensor.id}, LiDAR ID: {self.lidar_sensor.id}")
            self.logger.info("Sensors setup complete")
            
        except Exception as e:
            raise RuntimeError(f"Failed to setup sensors: {e}")
            
    def _setup_camera_sensor(self, blueprint_library):
        """设置相机传感器"""
        camera_config = self.config['carla_settings']['sensor_definitions']['camera']
        
        # 获取相机蓝图
        camera_bp = blueprint_library.find(camera_config['type'])
        
        # 设置相机属性
        camera_bp.set_attribute('image_size_x', str(camera_config['image_size_x']))
        camera_bp.set_attribute('image_size_y', str(camera_config['image_size_y']))
        camera_bp.set_attribute('fov', str(camera_config['fov']))
        
        # 设置传感器变换
        transform_config = camera_config['transform']
        sensor_transform = carla.Transform(
            carla.Location(x=transform_config['x'], y=transform_config['y'], z=transform_config['z']),
            carla.Rotation(pitch=transform_config['pitch'], yaw=transform_config['yaw'], roll=transform_config['roll'])
        )
        
        # 生成相机传感器
        self.camera_sensor = self.world.spawn_actor(camera_bp, sensor_transform, attach_to=self.vehicle)
        
        # 设置数据收集回调
        self.camera_sensor.listen(self._camera_callback)
        self.logger.info("Camera sensor setup complete")
        
    def _setup_lidar_sensor(self, blueprint_library):
        """设置激光雷达传感器"""
        lidar_config = self.config['carla_settings']['sensor_definitions']['lidar']
        
        # 获取激光雷达蓝图
        lidar_bp = blueprint_library.find(lidar_config['type'])
        
        # 设置激光雷达属性
        lidar_bp.set_attribute('range', str(lidar_config['range']))
        lidar_bp.set_attribute('points_per_second', str(lidar_config['points_per_second']))
        lidar_bp.set_attribute('rotation_frequency', str(lidar_config['rotation_frequency']))
        lidar_bp.set_attribute('upper_fov', str(lidar_config['upper_fov']))
        lidar_bp.set_attribute('lower_fov', str(lidar_config['lower_fov']))
        lidar_bp.set_attribute('channels', str(lidar_config['channels']))
        
        # 设置传感器变换
        transform_config = lidar_config['transform']
        sensor_transform = carla.Transform(
            carla.Location(x=transform_config['x'], y=transform_config['y'], z=transform_config['z']),
            carla.Rotation(pitch=transform_config['pitch'], yaw=transform_config['yaw'], roll=transform_config['roll'])
        )
        
        # 生成激光雷达传感器
        self.lidar_sensor = self.world.spawn_actor(lidar_bp, sensor_transform, attach_to=self.vehicle)
        
        # 设置数据收集回调
        self.lidar_sensor.listen(self._lidar_callback)
        self.logger.info("LiDAR sensor setup complete")
        
    def _camera_callback(self, image):
        """相机数据回调函数"""
        if self.is_collecting:
            self.logger.debug(f"Camera callback triggered for frame {image.frame}")
            
            # 将CARLA图像转换为numpy数组
            image_array = np.frombuffer(image.raw_data, dtype=np.uint8)
            image_array = image_array.reshape((image.height, image.width, 4))  # BGRA
            image_array = image_array[:, :, :3]  # 移除alpha通道，转为BGR
            
            # 创建SensorData对象
            sensor_data = SensorData(
                timestamp_ns=image.timestamp * 1_000_000_000,  # 转换为纳秒
                data=image_array,
                metadata={
                    'width': image.width,
                    'height': image.height,
                    'fov': image.fov,
                    'transform': image.transform
                }
            )
            
            self.sensor_data['camera'].put((image.frame, sensor_data))
            self.logger.debug(f"Camera data queued for frame {image.frame}")
            
    def _lidar_callback(self, lidar_data):
        """激光雷达数据回调函数"""
        if self.is_collecting:
            self.logger.debug(f"LiDAR callback triggered for frame {lidar_data.frame}")
            
            # 将CARLA LiDAR数据转换为numpy数组
            points = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
            points = points.reshape((-1, 4))  # [x, y, z, intensity]
            points = points[:, :3]  # 只保留xyz坐标
            
            # 创建SensorData对象
            sensor_data = SensorData(
                timestamp_ns=lidar_data.timestamp * 1_000_000_000,  # 转换为纳秒
                data=points,
                metadata={
                    'channels': len(points),
                    'transform': lidar_data.transform
                }
            )
            
            self.sensor_data['lidar'].put((lidar_data.frame, sensor_data))
            self.logger.debug(f"LiDAR data queued for frame {lidar_data.frame}, points: {len(points)}")
            
    def _collect_data(self, max_frames: int, directories: Dict):
        """收集数据主循环"""
        self.logger.info(f"Starting data collection for {max_frames} frames")
        self.is_collecting = True
        
        # 清空传感器数据队列
        while not self.sensor_data['camera'].empty():
            self.sensor_data['camera'].get()
        while not self.sensor_data['lidar'].empty():
            self.sensor_data['lidar'].get()
        
        try:
            # 预热几帧，让传感器开始工作
            self.logger.info("Warming up sensors...")
            for _ in range(3):
                self.world.tick()
                time.sleep(0.1)
            
            for frame_id in range(max_frames):
                start_time = time.time()
                
                # 推进仿真一帧
                self.world.tick()
                self.stats['frames_collected'] += 1
                
                # 给传感器回调足够的时间处理数据
                time.sleep(0.1)  # 等待传感器数据到达
                
                # 等待传感器数据
                camera_data = self._wait_for_sensor_data('camera', frame_id)
                lidar_data = self._wait_for_sensor_data('lidar', frame_id)
                
                if camera_data is None:
                    self.logger.warning(f"Frame {frame_id}: Missing camera data")
                if lidar_data is None:
                    self.logger.warning(f"Frame {frame_id}: Missing lidar data")
                    
                if camera_data is None or lidar_data is None:
                    # 检查传感器队列状态
                    cam_size = self.sensor_data['camera'].qsize()
                    lidar_size = self.sensor_data['lidar'].qsize()
                    self.logger.debug(f"Frame {frame_id}: Queue sizes - Camera: {cam_size}, LiDAR: {lidar_size}")
                    continue
                
                # 获取真值对象信息
                ground_truth_objects = self._get_ground_truth_objects()
                
                # 获取相机位姿
                camera_pose = self._get_camera_pose()
                
                # 处理帧数据
                self._process_frame(
                    frame_id, camera_data, lidar_data, 
                    camera_pose, ground_truth_objects, directories
                )
                
                self.stats['frames_processed'] += 1
                self.stats['objects_detected'] += len(ground_truth_objects)
                
                # 计算处理时间
                process_time = time.time() - start_time
                self.stats['simulation_time'] += process_time
                
                if (frame_id + 1) % 10 == 0:
                    self.logger.info(f"Processed {frame_id + 1}/{max_frames} frames")
                    
        finally:
            self.is_collecting = False
            
    def _wait_for_sensor_data(self, sensor_type: str, frame_id: int, timeout: float = 2.0) -> Optional[SensorData]:
        """等待传感器数据"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                received_frame, data = self.sensor_data[sensor_type].get(timeout=0.05)
                
                # 检查帧ID匹配
                if received_frame == frame_id:
                    self.logger.debug(f"Found matching {sensor_type} data for frame {frame_id}")
                    return data
                elif received_frame > frame_id:
                    # 收到了后续帧的数据，说明当前帧数据丢失
                    self.logger.debug(f"Got future {sensor_type} frame {received_frame} while waiting for {frame_id}")
                    self.sensor_data[sensor_type].put((received_frame, data))  # 放回队列
                    return None
                else:
                    # 收到了过时的数据，丢弃
                    self.logger.debug(f"Discarding old {sensor_type} frame {received_frame} while waiting for {frame_id}")
                    continue
                    
            except queue.Empty:
                continue
                
        self.logger.debug(f"Timeout waiting for {sensor_type} data for frame {frame_id}")
        return None
        
    def _get_ground_truth_objects(self) -> List[GroundTruthObject]:
        """获取场景中的真值对象信息"""
        objects = []
        
        try:
            # 获取所有车辆
            vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicles:
                if vehicle.id == self.vehicle.id:
                    continue  # 跳过自己的车辆
                    
                # 获取车辆变换信息
                transform = vehicle.get_transform()
                bounding_box = vehicle.bounding_box
                
                # 创建GroundTruthObject
                obj = GroundTruthObject(
                    class_name='car',
                    position_world=np.array([
                        transform.location.x, 
                        transform.location.y, 
                        transform.location.z
                    ]),
                    dimensions=np.array([
                        2 * bounding_box.extent.x,  # length
                        2 * bounding_box.extent.y,  # width  
                        2 * bounding_box.extent.z   # height
                    ]),
                    rotation_z_world=np.radians(transform.rotation.yaw),
                    object_id=vehicle.id
                )
                objects.append(obj)
                
            # 获取行人
            pedestrians = self.world.get_actors().filter('walker.pedestrian.*')
            for pedestrian in pedestrians:
                transform = pedestrian.get_transform()
                bounding_box = pedestrian.bounding_box
                
                obj = GroundTruthObject(
                    class_name='pedestrian',
                    position_world=np.array([
                        transform.location.x,
                        transform.location.y, 
                        transform.location.z
                    ]),
                    dimensions=np.array([
                        2 * bounding_box.extent.x,
                        2 * bounding_box.extent.y,
                        2 * bounding_box.extent.z
                    ]),
                    rotation_z_world=np.radians(transform.rotation.yaw),
                    object_id=pedestrian.id
                )
                objects.append(obj)
                
        except Exception as e:
            self.logger.warning(f"Error getting ground truth objects: {e}")
            
        return objects
        
    def _get_camera_pose(self) -> CameraPose:
        """获取相机位姿信息"""
        camera_transform = self.camera_sensor.get_transform()
        
        # 将CARLA的旋转转换为旋转矩阵
        rotation = camera_transform.rotation
        rotation_matrix = euler_to_rotation_matrix(
            np.radians(rotation.roll),
            np.radians(rotation.pitch), 
            np.radians(rotation.yaw)
        )
        
        return CameraPose(
            position_world=np.array([
                camera_transform.location.x,
                camera_transform.location.y,
                camera_transform.location.z
            ]),
            rotation_matrix_world=rotation_matrix,
            timestamp_ns=int(time.time() * 1_000_000_000)
        )
        
    def _get_camera_intrinsics(self) -> CameraIntrinsics:
        """获取相机内参信息"""
        intrinsics_config = self.config['camera_intrinsics']
        
        return CameraIntrinsics(
            fx=intrinsics_config['fx'],
            fy=intrinsics_config['fy'],
            cx=intrinsics_config['cx'],
            cy=intrinsics_config['cy'],
            width=intrinsics_config.get('width', 1242),
            height=intrinsics_config.get('height', 375)
        )
        
    def _process_frame(self, frame_id: int, camera_data: SensorData, lidar_data: SensorData,
                      camera_pose: CameraPose, ground_truth_objects: List[GroundTruthObject],
                      directories: Dict):
        """处理单帧数据"""
        try:
            # 保存图像
            save_image(directories['images'], frame_id, camera_data)
            
            # 保存点云
            save_lidar_bin(directories['velodyne'], frame_id, lidar_data.data)
            
            # 生成和保存标注
            if self.config.get('generate_annotations', True) and ground_truth_objects:
                camera_intrinsics = self._get_camera_intrinsics()
                kitti_labels = generate_kitti_annotations(
                    ground_truth_objects, camera_pose, camera_intrinsics
                )
                save_labels(directories['labels'], frame_id, kitti_labels)
                
            # 保存标定信息
            if self.config.get('save_calibration', True):
                camera_intrinsics = self._get_camera_intrinsics()
                save_calibration_info(directories['calib'], frame_id, camera_intrinsics, camera_pose)
                
        except Exception as e:
            self.logger.error(f"Error processing frame {frame_id}: {e}")
            
    def _generate_dataset_metadata(self) -> Dict:
        """生成数据集元数据"""
        metadata = {
            'dataset_name': f"CARLA_Dataset_{int(time.time())}",
            'description': "Dataset generated from CARLA simulation using multimodal data toolkit",
            'creation_time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'source_type': 'carla',
            'carla_settings': self.config['carla_settings'],
            'total_frames': self.stats['frames_processed'],
            'total_objects': self.stats['objects_detected'],
            'simulation_time_s': self.stats['simulation_time'],
            'camera_intrinsics': self.config['camera_intrinsics']
        }
        
        return metadata
        
    def _cleanup(self):
        """清理资源"""
        try:
            if self.camera_sensor:
                self.camera_sensor.stop()
                self.camera_sensor.destroy()
                self.camera_sensor = None
                
            if self.lidar_sensor:
                self.lidar_sensor.stop()
                self.lidar_sensor.destroy()
                self.lidar_sensor = None
                
            if self.vehicle:
                self.vehicle.destroy()
                self.vehicle = None
                
            if self.world:
                # 恢复异步模式
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
                
            self.logger.info("CARLA resources cleaned up")
            
        except Exception as e:
            self.logger.warning(f"Error during cleanup: {e}")


def process_carla(config: Dict) -> Dict:
    """
    CARLA适配器的主要入口函数
    
    Args:
        config (Dict): 完整的配置字典
        
    Returns:
        Dict: 处理结果字典
        
    Raises:
        RuntimeError: CARLA不可用或处理失败时抛出
    """
    if not CARLA_AVAILABLE:
        raise RuntimeError(f"CARLA library not available: {CARLA_IMPORT_ERROR}")
        
    adapter = CARLAAdapter(config)
    return adapter.process_carla()


# 兼容性函数
def main(host: str = 'localhost', port: int = 2000, output_path: str = './output', **kwargs) -> Dict:
    """
    简化的主函数接口
    
    Args:
        host (str): CARLA服务器地址
        port (int): CARLA服务器端口
        output_path (str): 输出目录路径
        **kwargs: 其他可选参数
        
    Returns:
        Dict: 处理结果字典
    """
    config = {
        'output_path': output_path,
        'carla_settings': {
            'host': host,
            'port': port,
            'world': kwargs.get('world', 'Town03'),
            'sensor_definitions': {
                'camera': {
                    'type': 'sensor.camera.rgb',
                    'image_size_x': kwargs.get('image_width', 1242),
                    'image_size_y': kwargs.get('image_height', 375),
                    'fov': kwargs.get('fov', 90),
                    'transform': kwargs.get('camera_transform', {
                        'x': 1.5, 'y': 0.0, 'z': 2.4, 
                        'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0
                    })
                },
                'lidar': {
                    'type': 'sensor.lidar.ray_cast',
                    'range': kwargs.get('lidar_range', 85.0),
                    'points_per_second': kwargs.get('points_per_second', 130000),
                    'rotation_frequency': kwargs.get('rotation_frequency', 20),
                    'upper_fov': kwargs.get('upper_fov', 10.0),
                    'lower_fov': kwargs.get('lower_fov', -30.0),
                    'channels': kwargs.get('channels', 64),
                    'transform': kwargs.get('lidar_transform', {
                        'x': 1.5, 'y': 0.0, 'z': 2.4,
                        'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0
                    })
                }
            }
        },
        'camera_intrinsics': kwargs.get('camera_intrinsics', {
            'fx': 621.0, 'fy': 621.0, 'cx': 621.0, 'cy': 187.5,
            'width': 1242, 'height': 375
        }),
        'max_frames': kwargs.get('max_frames', 1000),
        'generate_annotations': kwargs.get('generate_annotations', True)
    }
    
    return process_carla(config)
