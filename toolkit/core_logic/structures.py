"""
统一数据结构定义模块

这个模块定义了平台无关的数据结构，作为不同适配器和核心逻辑之间的通信接口。
这些结构体不包含任何特定于平台的代码（如ROS或CARLA），确保了高度的可移植性。
"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np


@dataclass
class SensorData:
    """
    通用传感器数据结构
    
    用于封装来自不同传感器的数据，包括时间戳和实际的传感器数据。
    可以用于图像、点云等各种类型的传感器数据。
    
    Attributes:
        timestamp_ns (int): 传感器数据的时间戳，单位为纳秒
        data (np.ndarray): 传感器数据的Numpy数组表示
        metadata (dict): 可选的元数据字典，如传感器参数等
    """
    timestamp_ns: int
    data: np.ndarray
    metadata: Optional[dict] = None


@dataclass  
class GroundTruthObject:
    """
    通用真值对象表示
    
    用于描述场景中物体的真实位置、尺寸和类别信息。
    所有坐标都是在世界坐标系下的表示。
    
    Attributes:
        class_name (str): 物体类别名称，如'Car', 'Pedestrian', 'Cyclist'等
        position_world (np.ndarray): 物体在世界坐标系中的位置 [x, y, z]
        dimensions (np.ndarray): 物体的尺寸 [length, width, height]，单位为米
        rotation_z_world (float): 物体绕Z轴的旋转角度（偏航角），单位为弧度
        object_id (int): 可选的物体唯一标识符
        confidence (float): 可选的检测置信度，范围0-1
    """
    class_name: str
    position_world: np.ndarray  # Shape (3,), representing [x, y, z] in world frame
    dimensions: np.ndarray      # Shape (3,), representing [length, width, height] 
    rotation_z_world: float     # Yaw/rotation around Z-axis in world frame (radians)
    object_id: Optional[int] = None
    confidence: Optional[float] = None


@dataclass
class CameraPose:
    """
    相机位姿信息
    
    用于描述相机在世界坐标系中的位置和朝向。
    
    Attributes:
        position_world (np.ndarray): 相机在世界坐标系中的位置 [x, y, z]
        rotation_matrix_world (np.ndarray): 相机在世界坐标系中的旋转矩阵 (3x3)
        timestamp_ns (int): 位姿对应的时间戳，单位为纳秒
    """
    position_world: np.ndarray      # Shape (3,), camera position in world frame
    rotation_matrix_world: np.ndarray  # Shape (3, 3), rotation matrix from world to camera
    timestamp_ns: int


@dataclass
class CameraIntrinsics:
    """
    相机内参信息
    
    用于描述相机的内部参数，用于3D到2D的投影变换。
    
    Attributes:
        fx (float): X方向焦距
        fy (float): Y方向焦距  
        cx (float): 主点X坐标
        cy (float): 主点Y坐标
        width (int): 图像宽度（像素）
        height (int): 图像高度（像素）
        distortion (np.ndarray): 可选的畸变系数
    """
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: Optional[np.ndarray] = None
    
    def to_matrix(self) -> np.ndarray:
        """将内参转换为3x3相机矩阵"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])


@dataclass
class SynchronizedFrame:
    """
    时间同步的传感器数据帧
    
    用于封装在同一时刻采集的多种传感器数据。
    
    Attributes:
        frame_id (int): 帧的唯一标识符
        timestamp_ns (int): 数据帧的时间戳，单位为纳秒
        image_data (SensorData): 相机图像数据
        lidar_data (SensorData): 激光雷达点云数据
        camera_pose (CameraPose): 相机位姿信息
        objects (List[GroundTruthObject]): 场景中的真值物体列表
        sync_tolerance_ns (int): 时间同步容忍度，单位为纳秒
    """
    frame_id: int
    timestamp_ns: int
    image_data: SensorData
    lidar_data: Optional[SensorData] = None
    camera_pose: Optional[CameraPose] = None
    objects: Optional[List[GroundTruthObject]] = None
    sync_tolerance_ns: int = 20000000  # Default 20ms tolerance


@dataclass
class DatasetMetadata:
    """
    数据集元数据信息
    
    用于记录数据集的基本信息和统计数据。
    
    Attributes:
        name (str): 数据集名称
        description (str): 数据集描述
        total_frames (int): 总帧数
        creation_time (str): 创建时间
        source_type (str): 数据源类型，如'rosbag', 'carla'等
        sensor_config (dict): 传感器配置信息
    """
    name: str
    description: str
    total_frames: int
    creation_time: str
    source_type: str
    sensor_config: dict


# KITTI标签格式相关的常量定义
KITTI_LABEL_FIELDS = [
    'type',        # 物体类型
    'truncated',   # 截断标志 (0.0-1.0)
    'occluded',    # 遮挡程度 (0,1,2,3)
    'alpha',       # 观察角度 (-pi到pi)
    'bbox_left',   # 2D边界框左边界
    'bbox_top',    # 2D边界框上边界  
    'bbox_right',  # 2D边界框右边界
    'bbox_bottom', # 2D边界框下边界
    'height',      # 3D物体高度
    'width',       # 3D物体宽度
    'length',      # 3D物体长度
    'x',           # 3D物体中心x坐标（相机坐标系）
    'y',           # 3D物体中心y坐标（相机坐标系）
    'z',           # 3D物体中心z坐标（相机坐标系）
    'rotation_y'   # 3D物体绕y轴旋转角度
]

# 支持的物体类别映射（KITTI格式）
KITTI_CLASS_MAPPING = {
    'car': 'Car',
    'pedestrian': 'Pedestrian', 
    'cyclist': 'Cyclist',
    'truck': 'Truck',
    'tram': 'Tram',
    'misc': 'Misc',
    'dontcare': 'DontCare'
}
