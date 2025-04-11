import os
import numpy as np
from pypcd4 import PointCloud
import open3d as o3d
from typing import Tuple

def pcd_from_path(file_path: str) -> np.ndarray:
    """
    Loads point clouds from PCD files.
    
    Parameters:
        file_path (str): Path to a .pcd file.
    
    Returns:
        np.ndarray: Numpy array representing the point cloud, shape [n_points, m_channels].
    """
    try:
        from pypcd4 import PointCloud
    except ImportError:
        raise ImportError("Please install pypcd4 using: pip install pypcd4")
        
    if not file_path.endswith(".pcd"):
        raise ValueError('Only ".pcd" format is accepted.')
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"The file {file_path} does not exist.")
    
    pc = PointCloud.from_path(file_path)
    return pc.numpy()


def downsample_voxel(point_cloud: np.ndarray, voxel_size: float) -> Tuple[np.ndarray, dict]:
    """
    Downsamples a pointcloud using a voxel grid.
    
    Args:
        point_cloud (np.ndarray): Input point cloud as an Nx3 numpy array.
        voxel_size (float): Desired voxel size for downsampling.
    
    Returns:
        Tuple[np.ndarray, dict]: Downsampled point cloud and a dictionary with voxel configuration.
    """
    # Convert np array to open3D pointcloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
    
    # Use open3D voxelization utility with desired voxel_size
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    
    # Get coordinates of the voxels that are points of the downsampled grid
    indices = np.array([voxel.grid_index for voxel in voxel_grid.get_voxels()], dtype=float) * voxel_size

    # Calculate voxel configuration details
    max_bound = voxel_grid.get_max_bound()
    min_bound = voxel_grid.get_min_bound()
    voxel_config = {
        'voxel_bounds': {'min': 0, 'max': indices.max(axis=0)},
        'real_bounds': {'min': min_bound, 'max': max_bound}
    }

    # Adjust indices to represent real-world coordinates (center of each voxel)
    indices += min_bound + voxel_size / 2

    return indices, voxel_config