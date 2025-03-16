# Object Detection for Autonomous Vehicles

This project implements an object detection system for autonomous vehicles using YOLOv5. The model is trained to detect and classify vehicles, pedestrians, and cyclists in urban environments.

## Project Structure

- `object_detection_project.ipynb`: Main Jupyter notebook containing code and documentation
- `requirements.txt`: Required dependencies
- `data/`: Directory for datasets (will be created when running the notebook)

## Setup Instructions

1. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Open and run the Jupyter notebook:
   ```bash
   jupyter notebook object_detection_project.ipynb
   ```

3. Follow the steps in the notebook to:
   - Download and prepare the KITTI dataset
   - Train the YOLOv5 model
   - Evaluate model performance

## Dataset

This project uses the KITTI Vision Benchmark Suite dataset, which contains street-level images with annotations for vehicles, pedestrians, and cyclists. The notebook includes code to download and prepare this dataset.

## Model

We use YOLOv5, a state-of-the-art object detection model. The notebook includes code to:
- Use a pretrained YOLOv5 model as baseline
- Fine-tune the model on the KITTI dataset
- Compare the performance of both models
