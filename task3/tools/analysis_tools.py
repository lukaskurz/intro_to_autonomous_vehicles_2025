import numpy as np
from typing import Tuple

def calc_error(gt_position: np.ndarray, gt_orientation: np.ndarray, 
               estimated_position: np.ndarray, estimated_orientation: np.ndarray) -> Tuple[float, float]:
    """
    Calculates lateral error and yaw angle error based on ground truth and estimated values.
    
    Args:
        gt_position (np.ndarray): Ground truth displacements in [x, y, z] format.
        gt_orientation (np.ndarray): Ground truth angles in [roll, pitch, yaw] format.
        estimated_position (np.ndarray): Estimated displacements in [x, y, z] format.
        estimated_orientation (np.ndarray): Estimated angles in [roll, pitch, yaw] format.
    
    Returns:
        Tuple[float, float]: Lateral error in meters and yaw angle error in degrees.
    """
    if not (gt_position.shape == (3,) and estimated_position.shape == (3,)):
        raise ValueError("Position inputs should be 1D numpy arrays with 3 elements [x, y, z].")

    if not (len(gt_orientation) == 3 and len(estimated_orientation) == 3):
        raise ValueError("Orientation inputs should have 3 elements [roll, pitch, yaw].")

    # Calculate lateral error in the XY plane
    position_error = np.linalg.norm(estimated_position[:2] - gt_position[:2])

    # Calculate yaw angle error in radians
    yaw_error = gt_orientation[2] - estimated_orientation[2]

    return position_error, yaw_error