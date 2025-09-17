"""
工具包初始化文件
"""

__version__ = "1.0.0"
__author__ = "Multimodal Data Toolkit Team"
__description__ = "跨平台多模态数据收集与KITTI格式转换工具包"

# 导入主要的数据结构
from .core_logic.structures import (
    SensorData,
    GroundTruthObject,
    CameraPose,
    CameraIntrinsics,
    SynchronizedFrame,
    DatasetMetadata,
    KITTI_LABEL_FIELDS,
    KITTI_CLASS_MAPPING
)

# 导出主要接口
__all__ = [
    'SensorData',
    'GroundTruthObject',
    'CameraPose', 
    'CameraIntrinsics',
    'SynchronizedFrame',
    'DatasetMetadata',
    'KITTI_LABEL_FIELDS',
    'KITTI_CLASS_MAPPING'
]
