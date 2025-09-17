"""
坐标变换和几何计算工具模块

这个模块提供各种坐标变换、几何投影和计算功能。
所有函数都是纯Python实现，不依赖于任何特定平台的库。
"""

from typing import Tuple, Optional, List
import numpy as np


def quaternion_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    """
    将四元数转换为旋转矩阵
    
    Args:
        quat (np.ndarray): 四元数 [x, y, z, w]
        
    Returns:
        np.ndarray: 3x3旋转矩阵
        
    Raises:
        ValueError: 四元数格式不正确时抛出
    """
    if quat.shape != (4,):
        raise ValueError("Quaternion must be a 4-element array [x, y, z, w]")
    
    x, y, z, w = quat
    
    # 归一化四元数
    norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
    if norm == 0:
        raise ValueError("Invalid quaternion: zero norm")
    
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    # 计算旋转矩阵
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    
    return rotation_matrix


def rotation_matrix_to_euler(rotation_matrix: np.ndarray) -> Tuple[float, float, float]:
    """
    将旋转矩阵转换为欧拉角 (roll, pitch, yaw)
    
    Args:
        rotation_matrix (np.ndarray): 3x3旋转矩阵
        
    Returns:
        Tuple[float, float, float]: 欧拉角 (roll, pitch, yaw) in radians
        
    Raises:
        ValueError: 旋转矩阵格式不正确时抛出
    """
    if rotation_matrix.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3")
    
    # 提取欧拉角
    sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = 0
    
    return roll, pitch, yaw


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    将欧拉角转换为旋转矩阵
    
    Args:
        roll (float): 绕X轴旋转角度（弧度）
        pitch (float): 绕Y轴旋转角度（弧度）  
        yaw (float): 绕Z轴旋转角度（弧度）
        
    Returns:
        np.ndarray: 3x3旋转矩阵
    """
    cos_r, sin_r = np.cos(roll), np.sin(roll)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    
    # 旋转矩阵 = Rz * Ry * Rx
    rotation_matrix = np.array([
        [cos_y*cos_p, cos_y*sin_p*sin_r - sin_y*cos_r, cos_y*sin_p*cos_r + sin_y*sin_r],
        [sin_y*cos_p, sin_y*sin_p*sin_r + cos_y*cos_r, sin_y*sin_p*cos_r - cos_y*sin_r],
        [-sin_p, cos_p*sin_r, cos_p*cos_r]
    ])
    
    return rotation_matrix


def transform_points(points: np.ndarray, translation: np.ndarray, 
                    rotation_matrix: np.ndarray) -> np.ndarray:
    """
    对点云应用刚体变换 (旋转 + 平移)
    
    Args:
        points (np.ndarray): 输入点云，形状为(N, 3)
        translation (np.ndarray): 平移向量，形状为(3,)
        rotation_matrix (np.ndarray): 旋转矩阵，形状为(3, 3)
        
    Returns:
        np.ndarray: 变换后的点云，形状为(N, 3)
        
    Raises:
        ValueError: 输入参数形状不正确时抛出
    """
    if len(points.shape) != 2 or points.shape[1] != 3:
        raise ValueError("Points must be Nx3 array")
    
    if translation.shape != (3,):
        raise ValueError("Translation must be 3-element array")
        
    if rotation_matrix.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3")
    
    # 应用旋转和平移: P' = R * P + t
    transformed_points = points @ rotation_matrix.T + translation
    
    return transformed_points


def world_to_camera_transform(world_points: np.ndarray, 
                            camera_position: np.ndarray,
                            camera_rotation: np.ndarray) -> np.ndarray:
    """
    将世界坐标系下的点转换到相机坐标系
    
    Args:
        world_points (np.ndarray): 世界坐标系下的点，可以是(3,)或(N, 3)
        camera_position (np.ndarray): 相机在世界坐标系下的位置，形状为(3,)
        camera_rotation (np.ndarray): 相机在世界坐标系下的旋转矩阵，形状为(3, 3)
        
    Returns:
        np.ndarray: 相机坐标系下的点，形状与输入相同
    """
    # 处理单个点的情况
    if len(world_points.shape) == 1:
        world_points = world_points.reshape(1, -1)
        single_point = True
    else:
        single_point = False
    
    # 相机到世界的变换的逆变换：世界到相机
    # P_camera = R_camera^T * (P_world - t_camera)
    relative_points = world_points - camera_position
    camera_points = relative_points @ camera_rotation  # R^T = R (正交矩阵)
    
    if single_point:
        return camera_points[0]
    return camera_points


def project_3d_to_2d(points_3d: np.ndarray, camera_matrix: np.ndarray,
                    distortion: Optional[np.ndarray] = None) -> np.ndarray:
    """
    将3D点投影到2D图像平面
    
    Args:
        points_3d (np.ndarray): 3D点坐标，形状为(N, 3)或(3,)
        camera_matrix (np.ndarray): 相机内参矩阵，形状为(3, 3)
        distortion (np.ndarray, optional): 畸变系数
        
    Returns:
        np.ndarray: 2D图像坐标，形状为(N, 2)或(2,)
        
    Raises:
        ValueError: 输入参数不正确时抛出
    """
    # 处理单个点的情况
    if len(points_3d.shape) == 1:
        points_3d = points_3d.reshape(1, -1)
        single_point = True
    else:
        single_point = False
        
    if points_3d.shape[1] != 3:
        raise ValueError("3D points must have shape (N, 3) or (3,)")
    
    if camera_matrix.shape != (3, 3):
        raise ValueError("Camera matrix must be 3x3")
    
    # 过滤掉z<=0的点（在相机后方）
    valid_mask = points_3d[:, 2] > 1e-6
    valid_points = points_3d[valid_mask]
    
    if len(valid_points) == 0:
        # 所有点都在相机后方
        if single_point:
            return np.array([np.nan, np.nan])
        else:
            return np.full((len(points_3d), 2), np.nan)
    
    # 投影到归一化图像平面
    normalized_coords = valid_points[:, :2] / valid_points[:, 2:3]
    
    # 应用畸变（如果提供）
    if distortion is not None:
        normalized_coords = _apply_distortion(normalized_coords, distortion)
    
    # 应用相机内参
    homogeneous_coords = np.column_stack([normalized_coords, np.ones(len(normalized_coords))])
    pixel_coords = homogeneous_coords @ camera_matrix.T
    pixel_coords = pixel_coords[:, :2]
    
    # 处理无效点
    if len(valid_points) < len(points_3d):
        result = np.full((len(points_3d), 2), np.nan)
        result[valid_mask] = pixel_coords
        pixel_coords = result
    
    if single_point:
        return pixel_coords[0]
    return pixel_coords


def _apply_distortion(normalized_coords: np.ndarray, 
                     distortion: np.ndarray) -> np.ndarray:
    """
    对归一化图像坐标应用径向和切向畸变
    
    Args:
        normalized_coords (np.ndarray): 归一化图像坐标，形状为(N, 2)
        distortion (np.ndarray): 畸变系数 [k1, k2, p1, p2, k3, ...]
        
    Returns:
        np.ndarray: 应用畸变后的归一化坐标
    """
    if len(distortion) < 4:
        return normalized_coords  # 畸变参数不足，跳过
    
    k1, k2, p1, p2 = distortion[:4]
    k3 = distortion[4] if len(distortion) > 4 else 0
    
    x, y = normalized_coords[:, 0], normalized_coords[:, 1]
    r2 = x**2 + y**2
    r4 = r2**2
    r6 = r2 * r4
    
    # 径向畸变
    radial_distortion = 1 + k1*r2 + k2*r4 + k3*r6
    
    # 切向畸变
    tangential_x = 2*p1*x*y + p2*(r2 + 2*x**2)
    tangential_y = p1*(r2 + 2*y**2) + 2*p2*x*y
    
    # 应用畸变
    x_distorted = x * radial_distortion + tangential_x
    y_distorted = y * radial_distortion + tangential_y
    
    return np.column_stack([x_distorted, y_distorted])


def compute_2d_bbox_from_3d(corners_2d: np.ndarray, image_width: int, 
                           image_height: int) -> Optional[Tuple[float, float, float, float]]:
    """
    从3D边界框的2D投影计算2D边界框
    
    Args:
        corners_2d (np.ndarray): 3D边界框8个角点的2D投影，形状为(8, 2)
        image_width (int): 图像宽度
        image_height (int): 图像高度
        
    Returns:
        Optional[Tuple[float, float, float, float]]: 2D边界框 (left, top, right, bottom)，
                                                   如果边界框无效则返回None
    """
    if corners_2d.shape != (8, 2):
        raise ValueError("Corners must be 8x2 array")
    
    # 过滤无效点（NaN或在图像外）
    valid_mask = (
        ~np.isnan(corners_2d).any(axis=1) &
        (corners_2d[:, 0] >= 0) & (corners_2d[:, 0] < image_width) &
        (corners_2d[:, 1] >= 0) & (corners_2d[:, 1] < image_height)
    )
    
    valid_corners = corners_2d[valid_mask]
    
    if len(valid_corners) == 0:
        return None  # 所有角点都在图像外
    
    # 计算边界框
    left = np.min(valid_corners[:, 0])
    top = np.min(valid_corners[:, 1])  
    right = np.max(valid_corners[:, 0])
    bottom = np.max(valid_corners[:, 1])
    
    # 确保边界框在图像内
    left = max(0, left)
    top = max(0, top)
    right = min(image_width - 1, right)
    bottom = min(image_height - 1, bottom)
    
    # 检查边界框是否有效
    if right <= left or bottom <= top:
        return None
    
    return (left, top, right, bottom)


def laserscan_to_pointcloud(ranges: np.ndarray, angle_min: float, 
                           angle_increment: float, range_min: float,
                           range_max: float) -> np.ndarray:
    """
    将ROS LaserScan消息转换为3D点云
    
    Args:
        ranges (np.ndarray): 距离测量值数组
        angle_min (float): 最小角度（弧度）
        angle_increment (float): 角度增量（弧度）
        range_min (float): 最小有效距离
        range_max (float): 最大有效距离
        
    Returns:
        np.ndarray: 3D点云，形状为(N, 3)，每行为[x, y, z]
    """
    # 生成角度数组
    angles = np.arange(len(ranges)) * angle_increment + angle_min
    
    # 过滤有效的距离值
    valid_mask = (ranges >= range_min) & (ranges <= range_max) & np.isfinite(ranges)
    valid_ranges = ranges[valid_mask]
    valid_angles = angles[valid_mask]
    
    # 转换为笛卡尔坐标（假设激光雷达在xy平面扫描，z=0）
    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    z = np.zeros_like(x)  # 2D激光雷达，z坐标为0
    
    return np.column_stack([x, y, z])


def compute_transform_matrix(translation: np.ndarray, 
                           rotation_matrix: np.ndarray) -> np.ndarray:
    """
    构建4x4齐次变换矩阵
    
    Args:
        translation (np.ndarray): 平移向量，形状为(3,)
        rotation_matrix (np.ndarray): 旋转矩阵，形状为(3, 3)
        
    Returns:
        np.ndarray: 4x4齐次变换矩阵
    """
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation
    
    return transform_matrix


def invert_transform(transform_matrix: np.ndarray) -> np.ndarray:
    """
    计算齐次变换矩阵的逆
    
    Args:
        transform_matrix (np.ndarray): 4x4齐次变换矩阵
        
    Returns:
        np.ndarray: 4x4逆变换矩阵
        
    Raises:
        ValueError: 输入不是4x4矩阵时抛出
    """
    if transform_matrix.shape != (4, 4):
        raise ValueError("Transform matrix must be 4x4")
    
    R = transform_matrix[:3, :3]
    t = transform_matrix[:3, 3]
    
    # 对于刚体变换：T^-1 = [R^T, -R^T*t; 0, 1]
    inv_transform = np.eye(4)
    inv_transform[:3, :3] = R.T
    inv_transform[:3, 3] = -R.T @ t
    
    return inv_transform


def interpolate_poses(pose1: Tuple[np.ndarray, np.ndarray], 
                     pose2: Tuple[np.ndarray, np.ndarray],
                     timestamp1: int, timestamp2: int, 
                     target_timestamp: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    在两个位姿之间进行时间插值
    
    Args:
        pose1 (Tuple): 第一个位姿 (position, rotation_matrix)
        pose2 (Tuple): 第二个位姿 (position, rotation_matrix) 
        timestamp1 (int): 第一个位姿的时间戳
        timestamp2 (int): 第二个位姿的时间戳
        target_timestamp (int): 目标时间戳
        
    Returns:
        Tuple[np.ndarray, np.ndarray]: 插值后的位姿 (position, rotation_matrix)
        
    Raises:
        ValueError: 时间戳不在合理范围内时抛出
    """
    if not (timestamp1 <= target_timestamp <= timestamp2):
        raise ValueError("Target timestamp must be between pose timestamps")
    
    # 计算插值权重
    if timestamp2 == timestamp1:
        alpha = 0.0
    else:
        alpha = (target_timestamp - timestamp1) / (timestamp2 - timestamp1)
    
    pos1, rot1 = pose1
    pos2, rot2 = pose2
    
    # 位置线性插值
    interpolated_position = (1 - alpha) * pos1 + alpha * pos2
    
    # 旋转矩阵球面线性插值（SLERP的简化版本）
    # 这里使用简单的线性插值，实际应用中可能需要更精确的SLERP
    interpolated_rotation = (1 - alpha) * rot1 + alpha * rot2
    
    # 重新正交化旋转矩阵
    U, _, Vt = np.linalg.svd(interpolated_rotation)
    interpolated_rotation = U @ Vt
    
    return interpolated_position, interpolated_rotation
