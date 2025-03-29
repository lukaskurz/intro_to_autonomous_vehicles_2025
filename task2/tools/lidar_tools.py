import torch


def convert_ground_truth_to_dict(detections):
    """
    Convert protobuf LidarDetection messages to dictionary format
    
    Args:
        detections: List of LidarDetection protobuf messages
    
    Returns:
        List of dictionaries with detection information
    """
    formatted_detections = []
    
    for det in detections:
        detection = {
            'id': det.id,
            'type': det.type,
            'pos': list(det.pos),
            'rot': list(det.rot),
            'scale': list(det.scale)
        }
        formatted_detections.append(detection)
    
    return formatted_detections

def convert_predictions_to_lidar_format(predictions, confidence_threshold=0.3):
    """
    Convert SFA3D predictions to LidarDetection format
    
    Args:
        predictions: tensor of shape [1, 50, 10] where 10 is:
            [score, x, y, z, dim1, dim2, dim3, dir1, dir2, class]
        confidence_threshold: minimum confidence score to keep detection
    
    Returns:
        List of dictionaries matching LidarDetection format
    """
    # Remove batch dimension
    predictions = predictions.squeeze(0)
    
    formatted_detections = []
    
    for pred in predictions:
        confidence = pred[0].item()
        
        # Skip low confidence predictions
        if confidence < confidence_threshold:
            continue
            
        # Calculate rotation angle from direction vectors
        rotation = torch.atan2(pred[7], pred[8]).item()
            
        detection = {
            'id': f'pred_{len(formatted_detections)}',
            'type': int(pred[9].item()),  # Class ID
            'pos': [pred[1].item(), pred[2].item(), pred[3].item()],  # x, y, z
            'rot': [0.0, 0.0, rotation],  # Converting direction vectors to angle
            'scale': [pred[4].item(), pred[5].item(), pred[6].item()],  # dimensions
            'confidence': confidence
        }
        
        formatted_detections.append(detection)
    
    return formatted_detections