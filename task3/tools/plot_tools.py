import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import open3d as o3d
import ipywidgets as widgets
from IPython.display import display, clear_output

from . import image_tools

COLOR_DICT = {
    "confirmed" : "green",
    "tentative" : "yellow",
    "initialized" : "red"
}

LABEL_DICT = {
    "confirmed" : "Confirmed track",
    "tentative" : "Tentative track",
    "initialized" : "Initialized track",
}

def plot_tracks(img, tracks, measurements, lidar_detections, camera, state=None):
    fig, (ax1, ax2) = plt.subplots(1,2)

    camera_mtx = np.array(camera.K).reshape(3,3)
    dist = np.array(camera.D)
    img1, img2 = image_tools.undistort(img, camera_mtx, dist)

    T = np.asarray(camera.T)
    T = np.reshape(T, (4,4))
    ax2.imshow(img1)

    for track in tracks:
        if state == None or track.state == state:
            color = COLOR_DICT[track.state]
            plt_label = LABEL_DICT[track.state]
            
            # get current track state
            w = track.w
            h = track.h
            l = track.l

            x0 = track.x[0] - l/2
            y0 = track.x[1] + w/2
            z0 = track.x[2] - h/2
            angle = track.yaw

            # plot bbox of track in bird eye view
            bbox = plt.Rectangle((-y0,x0),w,l, color=color, angle=angle, alpha=0.2)
            ax1.add_patch(bbox)

            # plot track position
            ax1.text(-track.x[1], track.x[0]+1, track.id)
            ax1.scatter(-track.x[1], track.x[0], color=color, marker="x", label = plt_label)

            # plot bbox on image
            # project veh pos on image
            veh_pos = np.ones((3,1))
            veh_pos[0:3,0] = track.x[0:3]

                        
            # bounding box corners
            x_corners = [-l/2, l/2, l/2, l/2, l/2, -l/2, -l/2, -l/2]  
            y_corners = [-w/2, -w/2, -w/2, w/2, w/2, w/2, w/2, -w/2]  
            z_corners = [-h/2, -h/2, h/2, h/2, -h/2, -h/2, h/2, h/2]  

            # bounding box
            corners_3D = np.array([x_corners, y_corners, z_corners])

            # translate
            corners_3D += veh_pos

            # translate
            homogeneous_coord = np.ones((corners_3D.shape[0]+1, corners_3D.shape[1]))
            homogeneous_coord[:3,:] = corners_3D

            #transform corners to camera frame of reference
            corners_3D = np.dot(T,homogeneous_coord)
            corners_3D = corners_3D[:3,:]
            
            depth = corners_3D[-1:]
            scaled_corners_3D = corners_3D / depth


            newcammtx, x, y, w, h = image_tools.get_offsets(img, camera_mtx, dist)  # getting offsets and new camera matrix
            corners_3D = np.dot(newcammtx, scaled_corners_3D).T  # rotation to fit the pointcloud with pixels

            ch_offset_pcd = []  # changing the offset of the pointcloud to match the cropped undistorted image
            for row in corners_3D:
                ch_offset_pcd.append([row[0]-x, row[1]+y, row[2]])

            ch_offset_pcd = np.array(ch_offset_pcd)
            ch_offset_pcd[:, -1] = depth

            cropped_pcd = ch_offset_pcd  # cropping the pointcloud so that only points within the image remain
            cropped_pcd = cropped_pcd.T

            # remove bounding boxes that include negative x, projection makes no sense
            if np.any(corners_3D[2,:] <= 0):
                continue
            
            # project to image
            corners_2D = np.zeros((2,8))
            corners_2D[:2,:] = cropped_pcd[:2,:]
            draw_line_indices = [0, 1, 2, 3, 4, 5, 6, 7, 0, 5, 4, 1, 2, 7, 6, 3]

            paths_2D = np.transpose(corners_2D[:, draw_line_indices])
            
            codes = [Path.LINETO]*paths_2D.shape[0]
            codes[0] = Path.MOVETO
            path = Path(paths_2D, codes)
                
            # plot bounding box in image
            p = patches.PathPatch(
                path, fill=False, color=color, linewidth=3)
            ax2.add_patch(p)


    # plot groud truth positions of vehicles
    for detection in lidar_detections:
        lx = detection.pos[0]
        ly = -detection.pos[1]
        ax1.scatter(ly, lx, color="gray", s=80, marker='+', label="ground truth")

    #Axes configurations
    ax1.set_xlabel('y [m]')
    ax1.set_ylabel('x [m]')
    ax1.set_aspect('equal')
    ax1.set_ylim(0, 50) 
    ax1.set_xlim(-10, 10)

def show_pcl(pcl, boxes=None, use_plotly=False, **kwargs):

    if use_plotly:
        _show_pcl(pcl, boxes, **kwargs)
        return
    
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(pcl[:,:3])

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pointcloud)

    if boxes is not None:
        for box in boxes:
            bbox_center = box[0]
            bbox_rot = box[1]
            bbox_size = box[2]
            # Check the shape
            if bbox_center.shape != (3,): 
                print("Error: Box center should be of shape (3,)")
                continue

            if bbox_rot.shape != (3,3): 
                print("Error: Box rotation should be a Rotation matrix of shape (3,3)")
                continue

            if bbox_size.shape != (3,): 
                print("Error: Box rot should be a Rotation matrix of shape (3,3)")
                continue
            
            bbox = o3d.geometry.OrientedBoundingBox(bbox_center, bbox_rot, bbox_size)
            bbox.color = (1,0,0)
            vis.add_geometry(bbox)

    opt = vis.get_render_option()
    opt.background_color = (0, 0, 0)
    opt.point_size = 2

    vis.run()
    vis.destroy_window()
    del opt  # Delete to avoid having  [Open3D ERROR] GLFW Error: The GLFW library is not initialized
    del vis  # Delete to avoid having  [Open3D ERROR] GLFW Error: The GLFW library is not initialized

def _show_pcl(pcl, boxes=None, color_by=2, colorscale='Turbo', camera_pos=None):
    """
    Visualize point cloud with proper depth sorting
    
    Args:
        pcl: Numpy array of shape (N, 3+) containing point coordinates (x,y,z)
        boxes: List of tuples, each containing (center, rotation_matrix, size) for each box
        color_by: Column index to use for coloring points (default: 2, which is z-coordinate)
        colorscale: Plotly colorscale to use
        camera_pos: Camera position for depth sorting (default: [0,0,0])
    """
    import plotly.graph_objects as go
    import IPython.display as display
    
    
    # Default camera position if not specified
    if camera_pos is None:
        camera_pos = np.array([0, 0, 0])
    
    # Calculate distance from camera to each point (for depth sorting)
    camera_distances = np.sqrt(
        (pcl[:, 0] - camera_pos[0])**2 +
        (pcl[:, 1] - camera_pos[1])**2 +
        (pcl[:, 2] - camera_pos[2])**2
    )
    
    # Sort points by distance (furthest first for proper occlusion)
    sort_indices = np.argsort(-camera_distances)
    sorted_pcl = pcl[sort_indices]
    
    # Create figure
    fig = go.Figure()
    
    # Add the depth-sorted point cloud with coloring
    fig.add_trace(go.Scatter3d(
        x=sorted_pcl[:, 0],
        y=sorted_pcl[:, 1],
        z=sorted_pcl[:, 2],
        mode='markers',
        marker=dict(
            size=2,
            color=sorted_pcl[:, color_by],
            colorscale=colorscale,
            opacity=0.8
        ),
        name='Point Cloud'
    ))
    
    
    # Add bounding boxes if provided
    if boxes is not None:
        for i, box in enumerate(boxes):
            bbox_center = box[0]
            bbox_rot = box[1]
            bbox_size = box[2]
            
            # Validate box parameters
            if bbox_center.shape != (3,):
                print(f"Error: Box center should be of shape (3,), got {bbox_center.shape}")
                continue
            if bbox_rot.shape != (3, 3):
                print(f"Error: Box rotation should be a Rotation matrix of shape (3,3), got {bbox_rot.shape}")
                continue
            if bbox_size.shape != (3,):
                print(f"Error: Box size should be of shape (3,), got {bbox_size.shape}")
                continue
                
            # Create the 8 corners of the bounding box in local coordinates
            local_vertices = np.array([
                [bbox_size[0]/2, bbox_size[1]/2, bbox_size[2]/2],
                [bbox_size[0]/2, bbox_size[1]/2, -bbox_size[2]/2],
                [bbox_size[0]/2, -bbox_size[1]/2, bbox_size[2]/2],
                [bbox_size[0]/2, -bbox_size[1]/2, -bbox_size[2]/2],
                [-bbox_size[0]/2, bbox_size[1]/2, bbox_size[2]/2],
                [-bbox_size[0]/2, bbox_size[1]/2, -bbox_size[2]/2],
                [-bbox_size[0]/2, -bbox_size[1]/2, bbox_size[2]/2],
                [-bbox_size[0]/2, -bbox_size[1]/2, -bbox_size[2]/2]
            ])
            
            # Transform to global coordinates using rotation matrix and center
            global_vertices = np.array([bbox_rot @ vertex + bbox_center for vertex in local_vertices])
            
            # Define the 12 edges of the box by connecting vertices
            edge_indices = [
                # Bottom face
                [0, 1], [1, 3], [3, 2], [2, 0],
                # Top face
                [4, 5], [5, 7], [7, 6], [6, 4],
                # Connecting edges
                [0, 4], [1, 5], [2, 6], [3, 7]
            ]
            
            # Create a trace for each edge of the box
            for edge in edge_indices:
                x_vals = [global_vertices[edge[0], 0], global_vertices[edge[1], 0], None]
                y_vals = [global_vertices[edge[0], 1], global_vertices[edge[1], 1], None]
                z_vals = [global_vertices[edge[0], 2], global_vertices[edge[1], 2], None]
                
                fig.add_trace(go.Scatter3d(
                    x=x_vals, 
                    y=y_vals, 
                    z=z_vals,
                    mode='lines',
                    line=dict(color='red', width=4),
                    name=f'Box {i}',
                    showlegend=False
                ))
    
    # Layout settings
    fig.update_layout(
        scene=dict(
            xaxis=dict(showbackground=False),
            yaxis=dict(showbackground=False),
            zaxis=dict(showbackground=False),
            aspectmode='data',
            bgcolor='black',
        ),
        width=800,
        height=800,
        margin=dict(l=0, r=0, b=0, t=0),
        paper_bgcolor='black',
        legend=dict(font=dict(color='white'))
    )
    
    # Display the figure
    display.clear_output(wait=True)
    display.display(fig)

def plot_pcd(ax: plt.Axes, points: np.ndarray, **kwargs):
    """
    Plots point cloud data on a given matplotlib axis.
    
    Parameters:
        ax (matplotlib.axes.Axes): The axes on which to plot the point cloud.
        points (np.ndarray): The point cloud data, expected shape [n_points, at least 2].
    """
    if points.ndim < 2 or points.shape[1] < 2:
        raise ValueError("The 'points' array must have at least two dimensions [n_points, at least 2].")

    return ax.scatter(points[:, 0], points[:, 1], **kwargs)

def view_frames(frames):
    # Create output widget to manage display
    output = widgets.Output()
    
    # Current frame index
    current_idx = widgets.IntSlider(
        value=0,
        min=0,
        max=len(frames)-1,
        step=1,
        description='Frame:',
        continuous_update=False
    )
    
    def view_frame(idx):
        with output:
            clear_output(wait=True)
            fig, ax = plt.subplots()
            plot_pcd(ax, frames[idx], s=0.1)
            plt.title(f'Frame {idx}')
            plt.show()
    
    # Button handlers
    def on_prev_button_clicked(b):
        current_idx.value = max(0, current_idx.value - 1)
    
    def on_next_button_clicked(b):
        current_idx.value = min(len(frames) - 1, current_idx.value + 1)
    
    # Create buttons
    prev_button = widgets.Button(description='← Previous')
    next_button = widgets.Button(description='Next →')
    
    # Attach button click handlers
    prev_button.on_click(on_prev_button_clicked)
    next_button.on_click(on_next_button_clicked)
    
    # Layout for buttons
    buttons = widgets.HBox([prev_button, next_button])
    
    # Create the interactive widget
    widgets.interactive(view_frame, idx=current_idx)
    
    # Display everything
    display(widgets.VBox([
        current_idx,
        buttons,
        output
    ]))
    
    # Show initial frame
    view_frame(0)


def plot_map_and_trajectories(map_pcd, gt_positions, estimated_positions, save_path=None):
    """
    Plot the map point cloud, ground truth positions, and estimated positions in 2D.
    
    Args:
        map_pcd: The map point cloud (numpy array of shape Nx3)
        gt_positions: Ground truth positions (numpy array of shape Mx3)
        estimated_positions: Estimated positions (numpy array of shape Mx3)
        save_path: Optional path to save the figure
    """
    import matplotlib.pyplot as plt
    import numpy as np
    
    # Create figure with adequate size
    plt.figure(figsize=(14, 10))
    
    # Plot map points (with low alpha to not overwhelm the plot)
    # Only use a subset of points to make the plot more efficient
    if len(map_pcd) > 10000:
        # Randomly sample points to reduce density
        indices = np.random.choice(len(map_pcd), 10000, replace=False)
        map_subset = map_pcd[indices]
    else:
        map_subset = map_pcd
    
    # Plot the map as a scatter plot (top-down view, so X and Y coordinates)
    plt.scatter(map_subset[:, 0], map_subset[:, 1], c='lightgray', s=1, alpha=0.3, label='Map')
    
    # Plot ground truth trajectory
    plt.plot(gt_positions[:, 0], gt_positions[:, 1], 'g-', linewidth=4, label='Ground Truth')
    plt.scatter(gt_positions[:, 0], gt_positions[:, 1], c='green', s=40, alpha=0.6)
    
    # Plot estimated trajectory
    plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], 'r-', linewidth=1, label='Estimated')
    plt.scatter(estimated_positions[:, 0], estimated_positions[:, 1], c='r', s=10, alpha=0.6)
    
    # Highlight start and end points
    plt.scatter(gt_positions[0, 0], gt_positions[0, 1], c='darkgreen', s=100, marker='^', label='Start')
    plt.scatter(gt_positions[-1, 0], gt_positions[-1, 1], c='darkred', s=100, marker='s', label='End')
    
    # Draw lines connecting ground truth and estimated positions to visualize errors
    for i in range(len(gt_positions)):
        plt.plot([gt_positions[i, 0], estimated_positions[i, 0]], 
                 [gt_positions[i, 1], estimated_positions[i, 1]], 
                 'r-', alpha=0.2, linewidth=0.5)
    
    # Calculate the range of the map to set proper aspect ratio
    x_min, x_max = np.min(map_subset[:, 0]), np.max(map_subset[:, 0])
    y_min, y_max = np.min(map_subset[:, 1]), np.max(map_subset[:, 1])
    
    # Add some padding around the map
    padding = 5  # meters
    plt.xlim(x_min - padding, x_max + padding)
    plt.ylim(y_min - padding, y_max + padding)
    
    # Set equal aspect ratio
    plt.gca().set_aspect('equal')
    
    # Add title, labels, legend
    plt.title('Map and Vehicle Trajectory', fontsize=16)
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    # Calculate and display statistics
    errors = np.linalg.norm(gt_positions - estimated_positions, axis=1)
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    
    plt.figtext(0.02, 0.02, f'Mean Error: {mean_error:.2f}m, Max Error: {max_error:.2f}m', 
                fontsize=12, bbox=dict(facecolor='white', alpha=0.8))
    
    # Adjust layout
    plt.tight_layout()
    
    # Save if requested
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    # Show the plot
    plt.show()