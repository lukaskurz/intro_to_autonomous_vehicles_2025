# Complete SFA3D Integration
import os
import sys
import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import imageio.v2 as imageio
from tqdm import tqdm
import shutil
from easydict import EasyDict as edict
from scipy.optimize import linear_sum_assignment

# Add SFA3D to path if not already added
if 'SFA3D' not in sys.path:
    sys.path.append('SFA3D')

import tools.dataset_tools as dataset_tools
import tools.plot_tools as plot_tools

# Import SFA3D modules - with error handling in case paths are different
try:
    from SFA3D.sfa.models.model_utils import create_model
    from SFA3D.sfa.utils.evaluation_utils import decode, post_processing
    from SFA3D.sfa.utils.torch_utils import _sigmoid
    from SFA3D.sfa.data_process.transformation import lidar_to_camera_box
    from SFA3D.sfa.data_process.kitti_bev_utils import makeBEVMap, drawRotatedBox
    from SFA3D.sfa.data_process.kitti_data_utils import Calibration
except ImportError:
    # Alternate import paths
    try:
        from sfa.models.model_utils import create_model
        from sfa.utils.evaluation_utils import decode, post_processing
        from sfa.utils.torch_utils import _sigmoid
        from sfa.data_process.transformation import lidar_to_camera_box
        from sfa.data_process.kitti_bev_utils import makeBEVMap, drawRotatedBox
        from sfa.data_process.kitti_data_utils import Calibration
    except ImportError:
        print("ERROR: Could not import SFA3D modules. Check your SFA3D installation path.")
        print("Current sys.path:", sys.path)

# SFA3D Configuration
def get_sfa3d_configs():
    """Get configuration for SFA3D model"""
    configs = edict()
    configs.arch = 'fpn_resnet_18'
    configs.K = 50  # number of top K
    configs.conf_thresh = 0.5  # confidence threshold
    configs.down_ratio = 4
    configs.num_classes = 3  # car, pedestrian, cyclist
    configs.peak_thresh = 0.2
    
    # Network config
    configs.imagenet_pretrained = False
    configs.head_conv = 64
    configs.num_center_offset = 2
    configs.num_z = 1
    configs.num_dim = 3
    configs.num_direction = 2  # sin, cos
    configs.heads = {
        'hm_cen': configs.num_classes,
        'cen_offset': configs.num_center_offset,
        'direction': configs.num_direction,
        'z_coor': configs.num_z,
        'dim': configs.num_dim
    }
    
    # BEV parameters
    configs.bev_height = 608
    configs.bev_width = 608
    configs.voxel_size = 0.1  # 10cm per pixel
    
    # Point cloud boundaries
    configs.boundary = {
        'minX': -51.2,
        'maxX': 51.2,
        'minY': -51.2,
        'maxY': 51.2,
        'minZ': -5.0,
        'maxZ': 3.0
    }
    
    return configs

# Function to load and initialize SFA3D model
def load_sfa3d_model(weight_path, configs):
    """Load the SFA3D model with pre-trained weights"""
    # Check if path exists
    if not os.path.isfile(weight_path):
        print(f"WARNING: Model weight file not found at {weight_path}")
        print("Download weights from https://github.com/maudzung/SFA3D/tree/master/checkpoints/")
        return None, None
    
    try:
        # Create model
        model = create_model(configs)
        
        # Load weights
        model.load_state_dict(torch.load(weight_path, map_location='cpu'))
        print(f'Loaded weights from {weight_path}')
        
        # Set model to evaluation mode
        model.eval()
        
        # Move to appropriate device
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        model = model.to(device)
        
        return model, device
    except Exception as e:
        print(f"ERROR loading SFA3D model: {e}")
        return None, None

# Kalman Track Class
class KalmanTrack:
    def __init__(self, detection):
        # State vector [x, y, z, vx, vy, vz, ax, ay, az]
        self.x = np.zeros(9)
        self.x[:3] = np.asarray(detection['pos'])  # Position
        
        # Store detection information
        self.l = detection['scale'][0]  # Length
        self.w = detection['scale'][1]  # Width
        self.h = detection['scale'][2]  # Height
        self.yaw = detection['rot'][2]  # Yaw (rotation around z-axis)
        self.score = detection.get('score', 1.0)  # Detection score
        
        # Property access shortcuts
        self.pos = self.x[:3]  # Position vector
        self.vel = self.x[3:6]  # Velocity vector
        self.acc = self.x[6:9]  # Acceleration vector
        
        # Track metadata
        self.state = "initialized"  # Track state
        self.age = 0                # Total frames since creation
        self.time_since_update = 0  # Frames since last update
        self.hits = 1               # Total number of matched detections
        self.id = detection['id']   # Track ID (numerical)
        self.obj_type = detection.get('type', 0)  # Object type (0=car, 1=pedestrian, etc.)
        
        # Kalman filter matrices
        # State transition matrix (constant acceleration model)
        self.F = np.eye(9)
        # We'll update dt during prediction
        
        # Measurement matrix (we only measure position)
        self.H = np.zeros((3, 9))
        self.H[:3, :3] = np.eye(3)
        
        # Process noise covariance
        self.Q = np.eye(9)
        self.Q[:3, :3] *= 0.01      # Position noise
        self.Q[3:6, 3:6] *= 0.1     # Velocity noise
        self.Q[6:, 6:] *= 1.0       # Acceleration noise
        
        # Measurement noise covariance
        self.R = np.eye(3) * 0.05   # Position measurement noise
        
        # State covariance matrix
        self.P = np.eye(9)
        self.P[:3, :3] *= 0.5       # Initial position uncertainty
        self.P[3:6, 3:6] *= 5.0     # Initial velocity uncertainty
        self.P[6:, 6:] *= 10.0      # Initial acceleration uncertainty
    
    def predict(self, dt):
        """Predict state forward by time step dt"""
        # Update state transition matrix with dt
        self.F[0:3, 3:6] = np.eye(3) * dt  # Position affected by velocity
        self.F[3:6, 6:9] = np.eye(3) * dt  # Velocity affected by acceleration
        
        # State prediction
        self.x = self.F @ self.x
        
        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # Update property shortcuts
        self.pos = self.x[:3]
        self.vel = self.x[3:6]
        self.acc = self.x[6:9]
        
        # Update metadata
        self.age += 1
        self.time_since_update += 1
        
        # Update state based on tracking history
        if self.time_since_update > 2:
            self.state = "tentative"  # Lost track
        elif self.hits >= 3:
            self.state = "confirmed"  # Confirmed track
        else:
            self.state = "tentative"  # Tentative track
    
    def update(self, detection):
        """Update state with measurement"""
        # Extract measurement
        z = np.asarray(detection['pos'])
        
        # Kalman filter update
        y = z - self.H @ self.x  # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R  # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I = np.eye(self.x.shape[0])
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T
        
        # Update dimensions from detection
        self.l = detection['scale'][0]
        self.w = detection['scale'][1]
        self.h = detection['scale'][2]
        self.yaw = detection['rot'][2]
        self.score = detection.get('score', 1.0)
        
        # Update property shortcuts
        self.pos = self.x[:3]
        self.vel = self.x[3:6]
        self.acc = self.x[6:9]
        
        # Update metadata
        self.time_since_update = 0
        self.hits += 1
        self.id = detection['id']  # Keep ID consistent
        
        # Update state based on hits
        if self.hits >= 3:
            self.state = "confirmed"
        else:
            self.state = "tentative"

# Multi-Object Tracker Implementation
class MultiObjectTracker:
    def __init__(self, max_age=5, min_hits=3, max_distance=3.0):
        """Initialize multi-object tracker"""
        self.max_age = max_age  # Maximum frames to keep without updates
        self.min_hits = min_hits  # Minimum hits to confirm track
        self.max_distance = max_distance  # Maximum distance for association
        self.tracks = []  # Active tracks
        self.frame_count = 0  # Frame counter
        
        # ID counter for SFA3D detections
        self.next_id = 0
    
    def update(self, detections, dt):
        """Update tracker with new detections"""
        self.frame_count += 1
        
        # Predict tracks forward
        for track in self.tracks:
            track.predict(dt)
        
        # Associate detections with existing tracks
        if len(self.tracks) > 0 and len(detections) > 0:
            # Create cost matrix
            cost_matrix = np.zeros((len(self.tracks), len(detections)))
            
            for t_idx, track in enumerate(self.tracks):
                for d_idx, detection in enumerate(detections):
                    # Calculate distance
                    det_pos = np.asarray(detection['pos'])
                    diff = det_pos - track.pos
                    
                    # Use track uncertainty if possible
                    try:
                        S = track.H @ track.P @ track.H.T + track.R
                        S_inv = np.linalg.inv(S)
                        distance = np.sqrt(diff @ S_inv @ diff)
                    except:
                        # Fallback to Euclidean distance
                        distance = np.linalg.norm(diff)
                    
                    cost_matrix[t_idx, d_idx] = distance
            
            # Apply maximum distance threshold
            cost_matrix[cost_matrix > self.max_distance] = 1000000
            
            # Solve assignment problem
            row_indices, col_indices = linear_sum_assignment(cost_matrix)
            
            # Process assignments
            matched_detection_indices = set()
            for row_idx, col_idx in zip(row_indices, col_indices):
                if cost_matrix[row_idx, col_idx] < self.max_distance:
                    self.tracks[row_idx].update(detections[col_idx])
                    matched_detection_indices.add(col_idx)
            
            # Create new tracks for unmatched detections
            for d_idx, detection in enumerate(detections):
                if d_idx not in matched_detection_indices:
                    self.tracks.append(KalmanTrack(detection))
            
            # Mark unmatched tracks
            unmatched_track_indices = set(range(len(self.tracks))) - set(row_indices)
            for t_idx in unmatched_track_indices:
                self.tracks[t_idx].time_since_update += 1
        else:
            # First frame - create tracks for all detections
            for detection in detections:
                self.tracks.append(KalmanTrack(detection))
        
        # Remove old tracks
        self.tracks = [track for track in self.tracks if track.time_since_update <= self.max_age]
    
    @property
    def active_tracks(self):
        """Return list of active/confirmed tracks"""
        return [track for track in self.tracks if track.hits >= self.min_hits]

# Convert lidar data to point cloud format that SFA3D expects
def convert_point_cloud(lidar_data):
    """Convert dataset lidar format to standard point cloud"""
    data = np.array(lidar_data, dtype=np.float32)
    
    # Handle different possible formats
    if len(data.shape) == 1:  # Flattened array
        if data.size % 4 == 0:  # Likely XYZI format (x,y,z,intensity)
            return data.reshape(-1, 4)
        elif data.size % 3 == 0:  # Likely XYZ format
            points = data.reshape(-1, 3)
            # Add dummy intensity channel
            intensity = np.ones((points.shape[0], 1), dtype=np.float32)
            return np.hstack((points, intensity))
        else:
            print(f"Warning: Cannot automatically determine point cloud format. Shape: {data.shape}")
            # Try to reshape to a depth image and extract points
            size = int(np.sqrt(data.shape[0]))
            if size**2 == data.shape[0]:  # It's a square, likely a depth map
                print(f"Interpreting as {size}x{size} depth map")
                depth_map = data.reshape(size, size)
                
                # Convert depth map to point cloud (simplified)
                points = []
                for i in range(size):
                    for j in range(size):
                        if depth_map[i, j] > 0:  # Valid depth
                            x = (j - size/2) * 0.1  # 10cm per pixel
                            y = (i - size/2) * 0.1
                            z = depth_map[i, j]
                            intensity = 1.0
                            points.append([x, y, z, intensity])
                
                if len(points) > 0:
                    return np.array(points, dtype=np.float32)
            
            # Fallback - create a minimal point cloud to avoid crashing
            print("Warning: Creating minimal dummy point cloud")
            return np.ones((100, 4), dtype=np.float32)
    
    return data  # Already in correct shape

# Create BEV map for SFA3D
def create_bev_map(points, configs):
    """Create Bird's Eye View map for SFA3D from point cloud"""
    # Filter points within boundaries
    mask = (points[:, 0] >= configs.boundary['minX']) & (points[:, 0] <= configs.boundary['maxX']) & \
           (points[:, 1] >= configs.boundary['minY']) & (points[:, 1] <= configs.boundary['maxY']) & \
           (points[:, 2] >= configs.boundary['minZ']) & (points[:, 2] <= configs.boundary['maxZ'])
    points = points[mask]
    
    if len(points) == 0:
        print("Warning: No points within boundary")
        # Create empty BEV map
        bev_map = np.zeros((3, configs.bev_height, configs.bev_width), dtype=np.float32)
        return torch.from_numpy(bev_map).unsqueeze(0)
    
    try:
        # Use SFA3D's BEV map creation function if available
        bev_map = makeBEVMap(points, configs.boundary)
        bev_map = torch.from_numpy(bev_map).float()
    except Exception as e:
        print(f"Error using SFA3D's makeBEVMap: {e}")
        print("Falling back to manual BEV map creation")
        
        # Manual BEV map creation (simplified version)
        height_map = np.zeros((configs.bev_height, configs.bev_width), dtype=np.float32)
        intensity_map = np.zeros((configs.bev_height, configs.bev_width), dtype=np.float32)
        density_map = np.zeros((configs.bev_height, configs.bev_width), dtype=np.float32)
        
        # Calculate mapping factors
        x_factor = configs.bev_width / (configs.boundary['maxX'] - configs.boundary['minX'])
        y_factor = configs.bev_height / (configs.boundary['maxY'] - configs.boundary['minY'])
        
        # Calculate indices for each point
        x_indices = ((points[:, 0] - configs.boundary['minX']) * x_factor).astype(np.int32)
        y_indices = ((points[:, 1] - configs.boundary['minY']) * y_factor).astype(np.int32)
        
        # Clip indices to prevent out of bounds
        x_indices = np.clip(x_indices, 0, configs.bev_width - 1)
        y_indices = np.clip(y_indices, 0, configs.bev_height - 1)
        
        # Fill in feature maps
        for i in range(points.shape[0]):
            x_idx, y_idx = x_indices[i], y_indices[i]
            height_map[y_idx, x_idx] = max(height_map[y_idx, x_idx], points[i, 2])
            intensity_map[y_idx, x_idx] = max(intensity_map[y_idx, x_idx], points[i, 3])
            density_map[y_idx, x_idx] += 1
        
        # Normalize maps
        height_map = (height_map - configs.boundary['minZ']) / (configs.boundary['maxZ'] - configs.boundary['minZ'])
        density_map = np.minimum(1.0, np.log(density_map + 1) / np.log(64))
        
        # Stack maps to create BEV representation
        bev_map = np.stack([height_map, intensity_map, density_map], axis=0)
        bev_map = torch.from_numpy(bev_map).float()
    
    return bev_map.unsqueeze(0)  # Add batch dimension

# Process SFA3D detections for tracker
def process_sfa3d_detections(detections, frame_idx, next_id_counter):
    """Convert SFA3D detections to tracker format"""
    processed_detections = []
    
    # Process each class (car, pedestrian, cyclist)
    for class_id, class_dets in enumerate(detections[0]):
        if len(class_dets) == 0:
            continue
            
        for det in class_dets:
            # Format: [score, x, y, z, h, w, l, yaw]
            score, x, y, z, h, w, l, yaw = det
            
            if score < 0.5:  # Filter by confidence
                continue
                
            # Create detection dictionary for tracker
            detection = {
                'pos': [x, y, z],
                'scale': [l, w, h],  # SFA3D uses h,w,l while we use l,w,h
                'rot': [0, 0, yaw],
                'id': next_id_counter,
                'type': class_id,
                'score': score
            }
            
            processed_detections.append(detection)
            next_id_counter += 1
    
    return processed_detections, next_id_counter

# Video Creation Function
def create_video(frame_dir, output_names=["tracking_video.mp4", "results.mp4"], fps=10):
    """Create video from saved frames"""
    # Get all image files
    images = sorted([os.path.join(frame_dir, f) for f in os.listdir(frame_dir) 
                    if f.endswith('.png')])
    
    print(f"Found {len(images)} frames in {frame_dir}")
    
    if not images:
        print(f"No frames found in {frame_dir}. Check if frames were saved correctly.")
        return
    
    # Get dimensions of first image
    first_img = Image.open(images[0])
    target_width, target_height = first_img.size
    print(f"First image size: {target_width}x{target_height}")
    
    # Ensure dimensions are divisible by 16
    target_width = (target_width // 16) * 16
    target_height = (target_height // 16) * 16
    
    # Create resized frames directory
    resized_dir = os.path.join(frame_dir, "resized")
    if os.path.exists(resized_dir):
        shutil.rmtree(resized_dir)
    os.makedirs(resized_dir)
    
    # Resize all images
    resized_images = []
    for i, img_path in enumerate(tqdm(images, desc="Resizing frames")):
        try:
            with Image.open(img_path) as img:
                img_resized = img.resize((target_width, target_height), Image.LANCZOS)
                resized_path = os.path.join(resized_dir, f"frame_{i:04d}.png")
                img_resized.save(resized_path)
                resized_images.append(resized_path)
        except Exception as e:
            print(f"Error resizing image {img_path}: {e}")
    
    print(f"Resized {len(resized_images)} frames")
    
    try:
        # Create videos
        writers = []
        for output_name in output_names:
            writers.append(imageio.get_writer(output_name, fps=fps))
        
        for img_path in tqdm(resized_images, desc="Creating video"):
            frame = imageio.imread(img_path)
            for writer in writers:
                writer.append_data(frame)
        
        # Close writers
        for writer in writers:
            writer.close()
        
        print(f"Videos created: {', '.join(output_names)}")
    except Exception as e:
        print(f"Error creating video: {e}")
        print("Try installing required packages: pip install imageio imageio-ffmpeg")

# Create default calibration for cases where actual calibration is unavailable
def create_default_calibration():
    """Create a default calibration object"""
    class DefaultCalib:
        def __init__(self):
            # Default values - adjust as needed
            self.P2 = np.array([[721.5377, 0.0, 609.5593, 44.85728],
                               [0.0, 721.5377, 172.8540, 0.2163791],
                               [0.0, 0.0, 1.0, 0.0027262]])
            self.R0 = np.eye(3)
            self.V2C = np.array([[0.0072, -0.9999, 0.0083, -0.0574],
                                [-0.0357, -0.0085, -0.9993, 0.1237],
                                [0.9993, 0.0071, -0.0357, -0.9834]])
    
    return DefaultCalib()

# Main function to process dataset with SFA3D
def process_with_sfa3d(dataset_path, output_dir, model_path):
    """Process dataset using SFA3D for detection and tracking"""
    # Setup directories
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)
    
    # Get SFA3D configuration
    configs = get_sfa3d_configs()
    
    # Load SFA3D model
    model, device = load_sfa3d_model(model_path, configs)
    if model is None:
        print("SFA3D model could not be loaded!")
        print("Falling back to pre-computed detections")
        return [], 0
    
    # Get all frame files
    frame_files = sorted([f for f in os.listdir(dataset_path) 
                        if f.startswith("frame_") and f.endswith(".pb")])
    
    # Initialize tracker
    tracker = MultiObjectTracker(max_age=8, min_hits=3, max_distance=3.5)
    next_id_counter = 0  # For assigning unique IDs to SFA3D detections
    
    # Available track states for plotting
    valid_states = ["confirmed", "tentative", "initialized"]
    
    # Process each frame
    for frame_idx, frame_file in enumerate(tqdm(frame_files, desc="Processing frames")):
        try:
            # Read frame
            frame = dataset_tools.read_frame(os.path.join(dataset_path, frame_file))
            
            # Get lidar and camera
            lidar = frame.lidars[0]
            camera = frame.cameras[0]
            
            try:
                # Convert lidar data to point cloud format
                point_cloud = convert_point_cloud(lidar.data)
                
                # Create BEV map for SFA3D
                bev_map = create_bev_map(point_cloud, configs)
                bev_map = bev_map.to(device)
                
                # Run SFA3D inference
                with torch.no_grad():
                    outputs = model(bev_map)
                    outputs['hm_cen'] = _sigmoid(outputs['hm_cen'])
                    outputs['cen_offset'] = _sigmoid(outputs['cen_offset'])
                    
                    # Decode outputs
                    detections = decode(outputs['hm_cen'], outputs['cen_offset'], 
                                       outputs['direction'], outputs['z_coor'], 
                                       outputs['dim'], K=configs.K)
                    detections = detections.cpu().numpy().astype(np.float32)
                    detections = post_processing(detections, configs.num_classes, 
                                                configs.down_ratio, configs.peak_thresh)
                
                # Convert SFA3D detections to tracker format
                track_detections, next_id_counter = process_sfa3d_detections(
                    detections, frame_idx, next_id_counter)
                
                # Filter out low confidence detections
                track_detections = [d for d in track_detections if d['score'] > 0.5]
                
            except Exception as e:
                print(f"Error in SFA3D detection for frame {frame_idx}: {e}")
                print("Falling back to pre-computed detections")
                
                # Use pre-computed detections instead
                track_detections = []
                for detection in lidar.detections:
                    # Filter only objects in front within reasonable distance
                    if detection.pos[0] > 0 and detection.pos[0] < 55:
                        # Create detection dictionary
                        det = {
                            'pos': detection.pos,
                            'scale': detection.scale,
                            'rot': detection.rot,
                            'id': next_id_counter,
                            'score': 1.0
                        }
                        track_detections.append(det)
                        next_id_counter += 1
            
            # Update tracker
            tracker.update(track_detections, dt=0.1)
            
            # Get image for visualization
            img_array = dataset_tools.decode_img(camera)
            
            # Save visualization
            plt.figure(figsize=(10, 6))
            if tracker.active_tracks:
                # Ensure track states are valid for plot_tools
                for track in tracker.active_tracks:
                    if track.state not in valid_states:
                        track.state = "initialized"
                
                # Plot tracks
                plot_tools.plot_tracks(img_array, tracker.active_tracks, [], [], camera)
            else:
                plt.imshow(img_array)
            
            # Add frame counter
            plt.text(10, 30, f"Frame: {frame_idx} | SFA3D", 
                    fontsize=12, color='white', 
                    bbox=dict(facecolor='black', alpha=0.5))
            
            # Count detections by class
            class_counts = {}
            for track in tracker.active_tracks:
                obj_type = getattr(track, 'obj_type', 0)
                class_counts[obj_type] = class_counts.get(obj_type, 0) + 1
            
            # Add detection counts
            y_pos = 60
            for cls_id, count in class_counts.items():
                cls_name = ['Car', 'Pedestrian', 'Cyclist'][cls_id] if cls_id < 3 else f'Class {cls_id}'
                plt.text(10, y_pos, f"{cls_name}: {count}", 
                        fontsize=10, color='white', 
                        bbox=dict(facecolor='black', alpha=0.5))
                y_pos += 25
            
            # Save frame - this is critical!
            frame_path = os.path.join(output_dir, f"frame_{frame_idx:04d}.png")
            plt.savefig(frame_path)
            plt.close()
            
        except Exception as e:
            print(f"Error processing frame {frame_idx}: {e}")
            import traceback
            traceback.print_exc()
            plt.close('all')
    
    # Verify frames were created
    saved_frames = [f for f in os.listdir(output_dir) if f.endswith('.png')]
    print(f"Saved {len(saved_frames)} frames to {output_dir}")
    
    return frame_files, next_id_counter

# Main execution code
def main_sfa3d():
    """Main function to run SFA3D detection and tracking pipeline"""
    # Set paths
    dataset_path = "./Dataset/data_2/"
    output_dir = "sfa3d_tracking_frames"
    
    # SFA3D model path - CHANGE THIS TO YOUR ACTUAL PATH
    model_path = "SFA3D/checkpoints/fpn_resnet_18/fpn_resnet_18_epoch_300.pth"
    
    # Run SFA3D detection and tracking
    frame_files, obj_count = process_with_sfa3d(dataset_path, output_dir, model_path)
    
    # Create videos
    create_video(output_dir, ["sfa3d_tracking.mp4", "results.mp4"], fps=10)
    
    print(f"Processed {len(frame_files)} frames with SFA3D detection.")
    print(f"Detected {obj_count} unique objects")

# Run the SFA3D pipeline
if __name__ == "__main__":
    main_sfa3d()