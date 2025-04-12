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

def calculate_lateral_errors(gt_positions, estimated_positions):
    """
    Calculate lateral errors between ground truth and estimated positions.
    
    Lateral error is the perpendicular distance from the estimated position
    to the intended path (as defined by the ground truth trajectory).
    
    Args:
        gt_positions: Ground truth positions (numpy array of shape Nx3)
        estimated_positions: Estimated positions (numpy array of shape Nx3)
        
    Returns:
        lateral_errors: Array of lateral errors for each position
    """
    
    lateral_errors = []
    
    # For each point, calculate heading and lateral error
    for i in range(len(gt_positions)):
        # Calculate the heading vector
        if i < len(gt_positions) - 1:
            # Use the vector to the next point
            heading = gt_positions[i+1] - gt_positions[i]
        elif i > 0:
            # For the last point, use the vector from the previous point
            heading = gt_positions[i] - gt_positions[i-1]
        else:
            # If there's only one point, we can't calculate lateral error
            continue
            
        # Only use x and y components for 2D lateral error
        heading_xy = heading[:2]
        
        # Normalize the heading vector
        if np.linalg.norm(heading_xy) > 0:
            heading_xy = heading_xy / np.linalg.norm(heading_xy)
        else:
            continue
        
        # Calculate error vector (estimated - ground truth)
        error_vector = estimated_positions[i] - gt_positions[i]
        error_vector_xy = error_vector[:2]
        
        # Calculate the lateral component (perpendicular to heading)
        # For 2D vectors, perpendicular is [-y, x]
        perpendicular = np.array([-heading_xy[1], heading_xy[0]])
        
        # Project error onto perpendicular vector to get lateral error
        lateral_error = np.abs(np.dot(error_vector_xy, perpendicular))
        lateral_errors.append(lateral_error)
    
    return np.array(lateral_errors)