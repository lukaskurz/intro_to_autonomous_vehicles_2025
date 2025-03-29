import numpy as np
import matplotlib.pyplot as plt
from easydict import EasyDict as edict

def _draw_box(config, ax, pos, scale, rotation, color, linestyle='-', label=None):
        x, y = pos[0], pos[1]  # Forward, left coordinates
        l, w = scale[0], scale[1]  # Length, width
        
        # Create rectangle points (in world coordinates)
        corners = np.array([
            [-l/2, -w/2],
            [l/2, -w/2],
            [l/2, w/2],
            [-l/2, w/2]
        ])
        
        # Rotate corners
        rot_matrix = np.array([
            [np.cos(rotation[2]), -np.sin(rotation[2])],
            [np.sin(rotation[2]), np.cos(rotation[2])]
        ])
        rotated_corners = corners @ rot_matrix.T
        
        # Convert each corner to pixel coordinates
        pixel_corners = []
        for dx, dy in rotated_corners:
            px, py = _world_to_pixel(x + dx, y + dy, config)
            pixel_corners.append([px, py])
        
        # Draw the box
        pixel_corners = np.array(pixel_corners)
        pixel_corners = np.vstack([pixel_corners, pixel_corners[0]])  # Close the rectangle
        ax.plot(pixel_corners[:, 1], pixel_corners[:, 0],  # Note: x and y are swapped for imshow
                color=color, linestyle=linestyle, label=label)

# Helper function to convert real-world coordinates to pixel coordinates
def _world_to_pixel(x, y, config):
    """Convert world coordinates to pixel coordinates based on config limits"""
    x_discretization = (config.lims.x[1] - config.lims.x[0]) / config.bev_height
    y_discretization = (config.lims.y[1] - config.lims.y[0]) / config.bev_width
    
    # Convert x coordinate (forward direction)
    # Invert the x-coordinate mapping to match the BEV generation
    pixel_x = int((config.lims.x[1] - x) / x_discretization)
    
    # Convert y coordinate (lateral direction)
    # Add width/2 to center the coordinate system
    pixel_y = int(y / y_discretization + config.bev_width / 2)
    
    return pixel_x, pixel_y

def visualize_bev(bev: np.ndarray, config: edict = None, predictions: np.ndarray = None, ground_truth: np.ndarray = None):
    """
    Visualize Bird's Eye View (BEV) data both as a combined map and individual channels
    
    Args:
        bev: numpy array of shape (channels, height, width)
        config: configuration parameters containing:
            lims: dictionary with detection limits
                x: [min_x, max_x] x-axis limits in meters
                y: [min_y, max_y] y-axis limits in meters 
                z: [min_z, max_z] z-axis limits in meters
                intensity: [min_intensity, max_intensity] intensity value limits
        predictions: list of prediction dictionaries (optional)
        ground_truth: list of ground truth dictionaries (optional)
    """

    # if predictions or ground_truth are provided, then config is required
    if predictions is not None or ground_truth is not None:
        if config is None:
            raise ValueError("config is required when predictions or ground_truth are provided")
    

    # Create a figure with a grid layout
    plt.figure(figsize=(12, 15), constrained_layout=True)
    
    # Create grid spec to control subplot sizes
    gs = plt.GridSpec(2, 3, height_ratios=[2, 1], width_ratios=[1, 1, 1], hspace=0.1, wspace=0.1)
    
    # Plot combined BEV map spanning full width
    ax1 = plt.subplot(gs[0, :])
    bev_map_display = np.transpose(bev, (1, 2, 0))
    im1 = ax1.imshow(bev_map_display)
    plt.colorbar(im1, ax=ax1)
    ax1.set_title("Combined Bird's Eye View Map")

    # Draw ground truth boxes
    if ground_truth:
        for gt in ground_truth:
            _draw_box(
                config,
                ax1,
                gt['pos'], 
                gt['scale'], 
                gt['rot'],
                color='r',
                linestyle='-',
                label='Ground Truth' if gt == ground_truth[0] else None
            )
    
    # Draw prediction boxes
    if predictions:
        for pred in predictions:
            _draw_box(
                config,
                ax1,
                pred['pos'], 
                pred['scale'], 
                pred['rot'],
                color='r',
                linestyle='--',
                label='Prediction' if pred == predictions[0] else None
            )
    
    plt.title("Bird's Eye View with Detections")
    plt.xlabel("Y (lateral)")
    plt.ylabel("X (forward)")
    
    if predictions or ground_truth:
        plt.legend()
    
    # Plot individual channels in bottom row
    titles = ['Density', 'Height', 'Intensity']
    for i, title in enumerate(titles):
        ax = plt.subplot(gs[1, i])
        im = ax.imshow(bev[i])
        plt.colorbar(im, ax=ax)
        ax.set_title(title)
    
    plt.show()

def pcl_to_bev(pcl: np.ndarray, config: edict) -> np.ndarray:
    """SFA3D-style BEV map creation while maintaining compatibility with your config structure
    
    Args:
        pcl: pointcloud as numpy array [n_points, m_channels]
        configs: configuration parameters containing:
            lims: dictionary with detection limits
                x: [min_x, max_x] x-axis limits in meters
                y: [min_y, max_y] y-axis limits in meters 
                z: [min_z, max_z] z-axis limits in meters
                intensity: [min_intensity, max_intensity] intensity value limits
            bev_height: height of output BEV map in pixels
            bev_width: width of output BEV map in pixels
    Returns:
        RGB_Map: BEV map as [3, height, width] array
    """
    # Create copy to avoid modifying original
    points = np.copy(pcl)
    
    # Filter points within detection range
    mask = np.where((points[:, 0] >= config.lims.x[0]) & (points[:, 0] <= config.lims.x[1]) &
                    (points[:, 1] >= config.lims.y[0]) & (points[:, 1] <= config.lims.y[1]) &
                    (points[:, 2] >= config.lims.z[0]) & (points[:, 2] <= config.lims.z[1]))
    points = points[mask]
    
    # Adjust height relative to ground (as in your implementation)
    points[:, 2] = points[:, 2] - config.lims.z[0]
    
    height = config.bev_height + 1
    width = config.bev_width + 1
    
    # Calculate discretization (similar to your bev_x_discret and bev_y_discret)
    x_discretization = (config.lims.x[1] - config.lims.x[0]) / config.bev_height
    y_discretization = (config.lims.y[1] - config.lims.y[0]) / config.bev_width
    
    # Discretize Feature Map (SFA3D style)
    points[:, 0] = np.int_(np.floor((config.lims.x[1] - points[:, 0]) / x_discretization))  # Inverted x-coordinate mapping
    points[:, 1] = np.int_(np.floor(points[:, 1] / y_discretization) + width / 2)

    
    # Sort points (SFA3D style)
    sorted_indices = np.lexsort((-points[:, 2], points[:, 1], points[:, 0]))
    points = points[sorted_indices]
    _, unique_indices, unique_counts = np.unique(points[:, 0:2], axis=0, return_index=True, return_counts=True)
    points_top = points[unique_indices]
    
    # Initialize maps
    heightMap = np.zeros((height, width))
    intensityMap = np.zeros((height, width))
    densityMap = np.zeros((height, width))
    
    # Fill maps (SFA3D style)
    max_height = float(np.abs(config.lims.z[1] - config.lims.z[0]))
    heightMap[np.int_(points_top[:, 0]), np.int_(points_top[:, 1])] = points_top[:, 2] / max_height
    # Clip intensity values to reasonable range and normalize
    clipped_intensities = np.clip(points_top[:, 3], config.lims.intensity[0], config.lims.intensity[1])
    intensityMap[np.int_(points_top[:, 0]), np.int_(points_top[:, 1])] = clipped_intensities / (config.lims.intensity[0] + config.lims.intensity[1])
    densityMap[np.int_(points_top[:, 0]), np.int_(points_top[:, 1])] = np.minimum(1.0, np.log(unique_counts + 1) / np.log(64))

    # Create RGB Map (same channel ordering as SFA3D)
    bev = np.zeros((3, height - 1, width - 1))
    bev[0, :, :] = densityMap[:config.bev_height, :config.bev_width]  # r_map
    bev[1, :, :] = heightMap[:config.bev_height, :config.bev_width]   # g_map
    bev[2, :, :] = intensityMap[:config.bev_height, :config.bev_width]# b_map
    
    return bev