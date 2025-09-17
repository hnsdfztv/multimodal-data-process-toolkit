"""
核心逻辑模块初始化文件
"""

from .structures import (
    SensorData,
    GroundTruthObject,
    CameraPose,
    CameraIntrinsics,
    SynchronizedFrame,
    DatasetMetadata
)

# processing模块的导入将在需要时进行，避免循环导入
__all__ = [
    'SensorData',
    'GroundTruthObject', 
    'CameraPose',
    'CameraIntrinsics',
    'SynchronizedFrame',
    'DatasetMetadata'
]
