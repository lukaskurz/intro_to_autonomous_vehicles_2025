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
        
        # Transform coordinates to match visualization coordinate system
        x = pred[1].item()  # Forward direction
        y = pred[2].item()  # Lateral direction
        z = pred[3].item()  # Height
            
        detection = {
            'id': f'pred_{len(formatted_detections)}',
            'type': int(pred[9].item()),  # Class ID
            'pos': [-x, y, z],  # Invert x to match visualization coordinate system
            'rot': [0.0, 0.0, rotation],
            'scale': [pred[4].item(), pred[5].item(), pred[6].item()],  # dimensions
            'confidence': confidence
        }
        
        formatted_detections.append(detection)
    
    return formatted_detections