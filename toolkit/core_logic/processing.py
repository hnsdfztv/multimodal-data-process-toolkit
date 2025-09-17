"""
核心数据处理逻辑模块

这个模块包含所有平台无关的纯Python数据处理函数。
这些函数只处理统一的数据结构，不包含任何特定于平台的代码。
主要功能包括数据保存、KITTI格式转换和标注生成等。
"""

import os
import json
from pathlib import Path
from typing import List, Tuple, Optional
import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: OpenCV not available. Image saving will be limited.")

from .structures import (
    SensorData, 
    GroundTruthObject, 
    CameraPose, 
    CameraIntrinsics,
    KITTI_LABEL_FIELDS
)
from ..utils.transformations import (
    world_to_camera_transform,
    project_3d_to_2d,
    compute_2d_bbox_from_3d
)


def create_kitti_directories(output_path: str) -> dict:
    """
    创建KITTI数据集标准目录结构
    
    Args:
        output_path (str): 输出根目录路径
        
    Returns:
        dict: 包含各个子目录路径的字典
        
    Raises:
        OSError: 目录创建失败时抛出异常
    """
    base_path = Path(output_path)
    
    directories = {
        'images': base_path / 'images',
        'velodyne': base_path / 'velodyne', 
        'labels': base_path / 'labels',
        'calib': base_path / 'calib',
        'metadata': base_path / 'metadata'
    }
    
    # 创建所有目录
    for dir_name, dir_path in directories.items():
        try:
            dir_path.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            raise OSError(f"Failed to create directory {dir_path}: {e}")
    
    return {k: str(v) for k, v in directories.items()}


def validate_output_directory(output_path: str) -> bool:
    """
    验证输出目录是否可写
    
    Args:
        output_path (str): 要验证的目录路径
        
    Returns:
        bool: 目录可写返回True，否则False
    """
    try:
        path = Path(output_path)
        path.mkdir(parents=True, exist_ok=True)
        
        # 尝试创建一个测试文件
        test_file = path / '.test_write_permission'
        test_file.touch()
        test_file.unlink()  # 删除测试文件
        
        return True
    except (OSError, PermissionError):
        return False


def save_image(output_path: str, frame_id: int, image_data: SensorData, 
               format: str = 'png') -> str:
    """
    保存图像数据为指定格式的文件
    
    Args:
        output_path (str): 输出目录路径
        frame_id (int): 帧编号，用于生成文件名
        image_data (SensorData): 包含图像数据的SensorData对象
        format (str): 图像格式，默认为'png'
        
    Returns:
        str: 保存的文件完整路径
        
    Raises:
        ValueError: 图像数据格式不正确时抛出
        RuntimeError: 保存失败时抛出
    """
    if not CV2_AVAILABLE:
        raise RuntimeError("OpenCV not available. Cannot save images.")
    
    # 验证图像数据
    if not isinstance(image_data.data, np.ndarray):
        raise ValueError("Image data must be a numpy array")
        
    if len(image_data.data.shape) not in [2, 3]:
        raise ValueError("Image data must be 2D (grayscale) or 3D (color)")
    
    # 生成文件名
    filename = f"{frame_id:06d}.{format}"
    filepath = Path(output_path) / filename
    
    # 确保输出目录存在
    filepath.parent.mkdir(parents=True, exist_ok=True)
    
    # 保存图像
    success = cv2.imwrite(str(filepath), image_data.data)
    if not success:
        raise RuntimeError(f"Failed to save image to {filepath}")
    
    return str(filepath)


def save_lidar_bin(output_path: str, frame_id: int, point_cloud: np.ndarray) -> str:
    """
    保存点云数据为KITTI格式的.bin文件
    
    KITTI点云格式: Nx4的float32数组，每一行为[x, y, z, intensity]
    
    Args:
        output_path (str): 输出目录路径
        frame_id (int): 帧编号，用于生成文件名
        point_cloud (np.ndarray): 点云数据，形状为(N, 3)或(N, 4)
        
    Returns:
        str: 保存的文件完整路径
        
    Raises:
        ValueError: 点云数据格式不正确时抛出
        RuntimeError: 保存失败时抛出
    """
    if not isinstance(point_cloud, np.ndarray):
        raise ValueError("Point cloud data must be a numpy array")
    
    if len(point_cloud.shape) != 2 or point_cloud.shape[1] not in [3, 4]:
        raise ValueError("Point cloud must be Nx3 or Nx4 array")
    
    # 如果没有强度信息，添加默认强度值
    if point_cloud.shape[1] == 3:
        intensities = np.ones((point_cloud.shape[0], 1), dtype=np.float32)
        kitti_points = np.hstack([point_cloud.astype(np.float32), intensities])
    else:
        kitti_points = point_cloud.astype(np.float32)
    
    # 生成文件名
    filename = f"{frame_id:06d}.bin"
    filepath = Path(output_path) / filename
    
    # 确保输出目录存在
    filepath.parent.mkdir(parents=True, exist_ok=True)
    
    # 保存为二进制文件
    try:
        kitti_points.tofile(str(filepath))
    except Exception as e:
        raise RuntimeError(f"Failed to save point cloud to {filepath}: {e}")
    
    return str(filepath)


def generate_kitti_annotations(
    objects: List[GroundTruthObject],
    camera_pose: CameraPose, 
    camera_intrinsics: CameraIntrinsics
) -> List[str]:
    """
    生成KITTI格式的标注字符串
    
    KITTI标注格式包含15个字段:
    type, truncated, occluded, alpha, bbox_left, bbox_top, bbox_right, bbox_bottom,
    height, width, length, x, y, z, rotation_y
    
    Args:
        objects (List[GroundTruthObject]): 世界坐标系下的物体列表
        camera_pose (CameraPose): 相机在世界坐标系下的位姿
        camera_intrinsics (CameraIntrinsics): 相机内参
        
    Returns:
        List[str]: KITTI格式的标注字符串列表
        
    Raises:
        ValueError: 输入参数不正确时抛出
    """
    if not objects:
        return []
    
    kitti_labels = []
    camera_matrix = camera_intrinsics.to_matrix()
    
    for obj in objects:
        try:
            # 1. 将物体坐标从世界坐标系转换到相机坐标系
            obj_pos_camera = world_to_camera_transform(
                obj.position_world, 
                camera_pose.position_world,
                camera_pose.rotation_matrix_world
            )
            
            # 2. 计算3D边界框的8个角点（相机坐标系下）
            corners_3d_camera = _compute_3d_bbox_corners(
                obj_pos_camera, obj.dimensions, obj.rotation_z_world
            )
            
            # 3. 将3D角点投影到2D图像平面
            corners_2d = project_3d_to_2d(corners_3d_camera, camera_matrix)
            
            # 4. 计算2D边界框
            bbox_2d = compute_2d_bbox_from_3d(
                corners_2d, camera_intrinsics.width, camera_intrinsics.height
            )
            
            # 如果边界框无效（物体不在相机视野内），跳过
            if bbox_2d is None:
                continue
            
            # 5. 生成KITTI标注字符串
            kitti_label = _format_kitti_label(
                obj, obj_pos_camera, bbox_2d, camera_pose.rotation_matrix_world
            )
            
            kitti_labels.append(kitti_label)
            
        except Exception as e:
            print(f"Warning: Failed to process object {obj.class_name}: {e}")
            continue
    
    return kitti_labels


def _compute_3d_bbox_corners(center: np.ndarray, dimensions: np.ndarray, 
                           rotation_y: float) -> np.ndarray:
    """
    计算3D边界框的8个角点坐标
    
    Args:
        center (np.ndarray): 物体中心坐标 [x, y, z]
        dimensions (np.ndarray): 物体尺寸 [length, width, height]
        rotation_y (float): 绕Y轴的旋转角度
        
    Returns:
        np.ndarray: 8个角点的坐标，形状为(8, 3)
    """
    l, w, h = dimensions
    
    # 定义本地坐标系下的8个角点（物体中心为原点）
    corners = np.array([
        [-l/2, -w/2, -h/2],  # 0: left-front-bottom
        [ l/2, -w/2, -h/2],  # 1: right-front-bottom  
        [ l/2,  w/2, -h/2],  # 2: right-rear-bottom
        [-l/2,  w/2, -h/2],  # 3: left-rear-bottom
        [-l/2, -w/2,  h/2],  # 4: left-front-top
        [ l/2, -w/2,  h/2],  # 5: right-front-top
        [ l/2,  w/2,  h/2],  # 6: right-rear-top
        [-l/2,  w/2,  h/2]   # 7: left-rear-top
    ])
    
    # 应用旋转变换
    cos_r, sin_r = np.cos(rotation_y), np.sin(rotation_y)
    rotation_matrix = np.array([
        [ cos_r, 0, sin_r],
        [     0, 1,     0],
        [-sin_r, 0, cos_r]
    ])
    
    # 旋转后平移到物体中心
    rotated_corners = corners @ rotation_matrix.T
    world_corners = rotated_corners + center
    
    return world_corners


def _format_kitti_label(obj: GroundTruthObject, obj_pos_camera: np.ndarray,
                       bbox_2d: Tuple[float, float, float, float],
                       camera_rotation: np.ndarray) -> str:
    """
    格式化KITTI标注字符串
    
    Args:
        obj (GroundTruthObject): 原始物体信息
        obj_pos_camera (np.ndarray): 物体在相机坐标系下的位置
        bbox_2d (tuple): 2D边界框 (left, top, right, bottom)
        camera_rotation (np.ndarray): 相机旋转矩阵
        
    Returns:
        str: KITTI格式的标注字符串
    """
    # KITTI字段值
    obj_type = obj.class_name.capitalize()
    truncated = 0.0  # 截断程度，暂时设为0
    occluded = 0     # 遮挡程度，暂时设为0（完全可见）
    
    # 计算alpha角度（观察角度）
    alpha = _compute_alpha_angle(obj.rotation_z_world, obj_pos_camera)
    
    # 2D边界框
    bbox_left, bbox_top, bbox_right, bbox_bottom = bbox_2d
    
    # 3D物体信息（相机坐标系）
    height, width, length = obj.dimensions  # KITTI格式：h, w, l
    x, y, z = obj_pos_camera  # 物体中心在相机坐标系下的坐标
    
    # 旋转角度（相机坐标系下绕Y轴的旋转）
    rotation_y = obj.rotation_z_world  # 简化处理，实际可能需要坐标系转换
    
    # 格式化为KITTI标注字符串
    kitti_label = (
        f"{obj_type} {truncated:.2f} {occluded:d} {alpha:.2f} "
        f"{bbox_left:.2f} {bbox_top:.2f} {bbox_right:.2f} {bbox_bottom:.2f} "
        f"{height:.2f} {width:.2f} {length:.2f} "
        f"{x:.2f} {y:.2f} {z:.2f} {rotation_y:.2f}"
    )
    
    return kitti_label


def _compute_alpha_angle(rotation_world: float, obj_pos_camera: np.ndarray) -> float:
    """
    计算KITTI格式的alpha观察角度
    
    Alpha = rotation_y - arctan2(x, z)
    其中rotation_y是物体的世界坐标系朝向，(x,z)是物体在相机坐标系下的位置
    
    Args:
        rotation_world (float): 物体在世界坐标系下绕Z轴的旋转角度
        obj_pos_camera (np.ndarray): 物体在相机坐标系下的位置
        
    Returns:
        float: Alpha角度值，范围在[-pi, pi]
    """
    x, _, z = obj_pos_camera
    view_angle = np.arctan2(x, z)
    alpha = rotation_world - view_angle
    
    # 确保角度在[-pi, pi]范围内
    while alpha > np.pi:
        alpha -= 2 * np.pi
    while alpha < -np.pi:
        alpha += 2 * np.pi
        
    return alpha


def save_labels(output_path: str, frame_id: int, kitti_labels: List[str]) -> str:
    """
    保存KITTI格式的标注文件
    
    Args:
        output_path (str): 输出目录路径
        frame_id (int): 帧编号，用于生成文件名
        kitti_labels (List[str]): KITTI格式标注字符串列表
        
    Returns:
        str: 保存的标注文件完整路径
        
    Raises:
        RuntimeError: 文件保存失败时抛出
    """
    filename = f"{frame_id:06d}.txt"
    filepath = Path(output_path) / filename
    
    # 确保输出目录存在
    filepath.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            for label in kitti_labels:
                f.write(label + '\n')
    except Exception as e:
        raise RuntimeError(f"Failed to save labels to {filepath}: {e}")
    
    return str(filepath)


def save_calibration_info(output_path: str, frame_id: int, 
                         camera_intrinsics: CameraIntrinsics,
                         camera_pose: Optional[CameraPose] = None) -> str:
    """
    保存KITTI格式的相机标定信息
    
    Args:
        output_path (str): 输出目录路径
        frame_id (int): 帧编号
        camera_intrinsics (CameraIntrinsics): 相机内参
        camera_pose (CameraPose, optional): 相机外参
        
    Returns:
        str: 保存的标定文件完整路径
    """
    filename = f"{frame_id:06d}.txt"
    filepath = Path(output_path) / filename
    
    # 确保输出目录存在  
    filepath.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            # P0: 左彩色相机投影矩阵 (3x4)
            P0 = np.zeros((3, 4))
            P0[:3, :3] = camera_intrinsics.to_matrix()
            f.write(f"P0: {' '.join(map(str, P0.flatten()))}\n")
            
            # P1: 左灰度相机投影矩阵 (通常与P0相同)
            f.write(f"P1: {' '.join(map(str, P0.flatten()))}\n")
            
            # P2: 左彩色相机投影矩阵 (通常与P0相同)  
            f.write(f"P2: {' '.join(map(str, P0.flatten()))}\n")
            
            # P3: 右彩色相机投影矩阵 (单目情况下与P0相同)
            f.write(f"P3: {' '.join(map(str, P0.flatten()))}\n")
            
            # R0_rect: 矫正旋转矩阵 (3x3, 单目情况下为单位矩阵)
            R0_rect = np.eye(3)
            f.write(f"R0_rect: {' '.join(map(str, R0_rect.flatten()))}\n")
            
            # Tr_velo_to_cam: 激光雷达到相机的变换矩阵 (3x4)
            if camera_pose is not None:
                Tr_velo_to_cam = np.eye(3, 4)  # 简化处理
            else:
                Tr_velo_to_cam = np.eye(3, 4)
            f.write(f"Tr_velo_to_cam: {' '.join(map(str, Tr_velo_to_cam.flatten()))}\n")
            
            # Tr_imu_to_velo: IMU到激光雷达的变换矩阵 (3x4)  
            Tr_imu_to_velo = np.eye(3, 4)
            f.write(f"Tr_imu_to_velo: {' '.join(map(str, Tr_imu_to_velo.flatten()))}\n")
            
    except Exception as e:
        raise RuntimeError(f"Failed to save calibration info to {filepath}: {e}")
    
    return str(filepath)


def save_metadata(output_path: str, metadata: dict) -> str:
    """
    保存数据集元数据信息
    
    Args:
        output_path (str): 输出目录路径  
        metadata (dict): 元数据字典
        
    Returns:
        str: 保存的元数据文件完整路径
    """
    filepath = Path(output_path) / "dataset_metadata.json"
    filepath.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
    except Exception as e:
        raise RuntimeError(f"Failed to save metadata to {filepath}: {e}")
    
    return str(filepath)
