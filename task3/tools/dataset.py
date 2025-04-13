import os
import numpy as np
import pandas as pd
from tools.pcd_tools import pcd_from_path


class FrameDataset():
    def __init__(self, frames_dir: str, ground_truth_path: str, zero_z: bool = False):
        """
        Initialize the FrameDataset object.

        Args:
            frames_dir (str): The path to the frames directory.
            ground_truth_path (str): The path to the ground truth CSV file.
        """
        self.frames_dir = frames_dir
        self.ground_truth_path = ground_truth_path
        self.frames = self._load_frames()
        self.ground_truth = self._load_ground_truth()
        self.zero_z = zero_z
    def _load_frames(self):
        frames = []
        for file in os.listdir(self.frames_dir):
            if file.endswith(".pcd"):
                frames.append(os.path.join(self.frames_dir, file))
        
        # sort frames by number, frames are named like this: frame_0.pcd
        frames.sort(key=lambda x: int(x.split("_")[-1].split(".")[0]))
        return frames

    def _load_ground_truth(self):
        gts = pd.read_csv(self.ground_truth_path, skipinitialspace=True)
        return gts
    
    def __len__(self):
        return len(self.frames)
    
    def __getitem__(self, idx):
        """
        Get the frame, ground truth position and orientation for the given index.

        Args:
            idx (int): The index of the frame to get.

        Returns:
            pcd (open3d.geometry.PointCloud): The point cloud of the frame.
            gt_pos (numpy.ndarray): The ground truth position of the frame.
            gt_orientation (numpy.ndarray): The ground truth orientation of the frame.
        """
        frame = self.frames[idx]
        pcd = pcd_from_path(frame)

        ground_truth = self.ground_truth.iloc[idx]
        gt_pos = ground_truth[['x', 'y', 'z']].to_numpy()
        gt_orientation = ground_truth[['roll', 'pitch', 'yaw']].to_numpy()

        if self.zero_z:
            pcd = np.c_[pcd[:, 0], pcd[:, 1], np.zeros(pcd.shape[0])]
            gt_pos = np.array([gt_pos[0], gt_pos[1], 0.0])
            gt_orientation = np.array([0.0, 0.0, gt_orientation[2]])
            


        return pcd, gt_pos, gt_orientation