import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from langchain_core.tools import tool
from typing import Dict, List, Optional
import numpy as np
import threading
import time
from collections import deque
from datetime import datetime, timedelta

class DetectionTool(Node):
    def __init__(self):
        super().__init__('detection_tool')
        
        # Store detection history with timestamps
        self.detection_history = deque(maxlen=300)  # Store last 300 detection states (up from 50)
        self.history_window = timedelta(seconds=30)  # Keep 30 seconds of history (up from 5)
        self.latest_image = None
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        
        # Parameters for temporal smoothing
        self.position_smoothing = 0.3  # Weight for position updates
        self.confidence_threshold = 0.5  # Minimum confidence to track
        self.spatial_threshold = 0.3  # 30cm threshold for considering objects the same
        self.tracked_objects = {}  # Store smoothed object tracks
        
        # Subscribe to detection topics
        self.marker_sub = self.create_subscription(
            MarkerArray, 
            '/detection_markers', 
            self.marker_callback, 
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/detection_image',
            self.image_callback,
            10
        )
    
    def _update_tracked_objects(self, detections: Dict):
        """Update tracked objects with temporal smoothing"""
        current_time = datetime.now()
        
        # Update existing tracks and add new ones
        for class_name, objects in detections.items():
            if class_name not in self.tracked_objects:
                self.tracked_objects[class_name] = []
            
            # Match detections to existing tracks
            for obj in objects:
                if obj['confidence'] < self.confidence_threshold:
                    continue
                    
                matched = False
                for track in self.tracked_objects[class_name]:
                    # Check if detection matches existing track
                    track_pos = np.array([track['position']['x'], 
                                        track['position']['y'], 
                                        track['position']['z']])
                    obj_pos = np.array([obj['position']['x'], 
                                      obj['position']['y'], 
                                      obj['position']['z']])
                    distance = np.linalg.norm(track_pos - obj_pos)
                    
                    if distance < self.spatial_threshold:
                        # Update track with smoothing
                        alpha = self.position_smoothing
                        track['position']['x'] = (1-alpha) * track['position']['x'] + alpha * obj['position']['x']
                        track['position']['y'] = (1-alpha) * track['position']['y'] + alpha * obj['position']['y']
                        track['position']['z'] = (1-alpha) * track['position']['z'] + alpha * obj['position']['z']
                        track['distance'] = np.sqrt(sum(x*x for x in track['position'].values()))
                        track['confidence'] = max(track['confidence'], obj['confidence'])
                        track['last_seen'] = current_time
                        matched = True
                        break
                
                if not matched:
                    # Create new track
                    obj['last_seen'] = current_time
                    self.tracked_objects[class_name].append(obj)
        
        # Remove old tracks
        for class_name in list(self.tracked_objects.keys()):
            self.tracked_objects[class_name] = [
                track for track in self.tracked_objects[class_name]
                if (current_time - track['last_seen']).total_seconds() < 2.0  # Keep tracks for 2 seconds
            ]
            if not self.tracked_objects[class_name]:
                del self.tracked_objects[class_name]
    
    def marker_callback(self, msg: MarkerArray):
        with self._lock:
            # Group markers by class name
            detections = {}
            for marker in msg.markers:
                if marker.ns == "detections":  # Skip text markers
                    # Extract class name and confidence from the corresponding text marker
                    text_marker = next(
                        (m for m in msg.markers if m.ns == "labels" and m.id == marker.id + 10000),
                        None
                    )
                    if text_marker:
                        class_name, confidence = text_marker.text.rsplit(' ', 1)
                        confidence = float(confidence)
                        
                        if class_name not in detections:
                            detections[class_name] = []
                            
                        detections[class_name].append({
                            'position': {
                                'x': marker.pose.position.x,
                                'y': marker.pose.position.y,
                                'z': marker.pose.position.z
                            },
                            'confidence': confidence,
                            'distance': np.sqrt(
                                marker.pose.position.x**2 + 
                                marker.pose.position.y**2 + 
                                marker.pose.position.z**2
                            ),
                            'timestamp': datetime.now()
                        })
            
            # Update tracked objects
            if detections:
                self._update_tracked_objects(detections)
                self.detection_history.append(detections)
                self._clean_history()
    
    def _clean_history(self):
        """Remove detections older than history_window"""
        current_time = datetime.now()
        while self.detection_history and any(
            det.get('timestamp', current_time) < current_time - self.history_window 
            for detections in self.detection_history[0].values() 
            for det in detections
        ):
            self.detection_history.popleft()
    
    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
    
    def get_detection_context(self) -> Dict:
        """Get context about recent detections for RAG-style queries"""
        with self._lock:
            if not self.detection_history:
                return {"current": {}, "recent": {}, "summary": "No objects detected recently."}
            
            # Get current detections (most recent)
            current = self.tracked_objects.copy()  # Use tracked objects instead of raw detections
            
            # Aggregate recent history
            recent = {}
            for detection_state in self.detection_history:
                for class_name, objects in detection_state.items():
                    if class_name not in recent:
                        recent[class_name] = []
                    recent[class_name].extend(objects)
            
            # Create summary
            summary = []
            for class_name, objects in current.items():
                # Count unique instances (using position clustering)
                unique_positions = set()
                for obj in objects:
                    pos = (
                        round(obj['position']['x'], 1),
                        round(obj['position']['y'], 1),
                        round(obj['position']['z'], 1)
                    )
                    unique_positions.add(pos)
                
                avg_distance = np.mean([obj['distance'] for obj in objects])
                summary.append(
                    f"{len(objects)} {class_name}(s) at average distance of {avg_distance:.2f}m"
                )
            
            return {
                "current": current,
                "recent": recent,
                "summary": "\n".join(summary) if summary else "No objects currently detected."
            }

# Global instance
_detection_tool = None
_node = None

def init_detection_tool():
    global _detection_tool, _node
    if _detection_tool is None:
        if not rclpy.ok():
            rclpy.init()
        _detection_tool = DetectionTool()
        # Start spinning in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(_detection_tool,))
        thread.daemon = True
        thread.start()
        # Wait a bit for subscriptions to be established
        time.sleep(1.0)

def format_detection_details(objects: List[Dict]) -> str:
    """Format detection details in a clear, readable way"""
    details = []
    for obj in objects:
        pos = obj['position']
        details.append(
            f"  - Distance: {obj['distance']:.2f}m"
            f", Position: (x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f})"
            f", Confidence: {obj['confidence']:.2%}"
        )
    return "\n".join(details)

@tool
def get_detected_objects(query: str = "") -> str:
    """Get information about detected objects. You can ask natural language questions about:
    - Currently visible objects
    - Objects seen recently (within last 30 seconds)
    - Specific object locations and movements
    - Changes in object positions
    - Closest/farthest objects
    - Object counts and frequencies
    """
    
    # Initialize tool if needed
    if _detection_tool is None:
        init_detection_tool()
    
    # Get detection context
    context = _detection_tool.get_detection_context()
    
    if not context["current"] and not context["recent"]:
        return "No objects have been detected recently."
    
    # If no specific query, list all current detections with details
    if not query:
        response = ["Currently tracked objects:"]
        for class_name, objects in context["current"].items():
            response.append(f"\n{class_name} ({len(objects)}):")
            response.append(format_detection_details(objects))
        return "\n".join(response) if len(response) > 1 else "No objects currently tracked."
    
    query = query.lower()
    
    # Handle temporal queries
    if any(word in query for word in ['now', 'current', 'currently', 'visible', 'see']):
        response = ["Currently tracked objects:"]
        for class_name, objects in context["current"].items():
            response.append(f"\n{class_name} ({len(objects)}):")
            response.append(format_detection_details(objects))
        return "\n".join(response) if len(response) > 1 else "No objects are currently tracked."
    
    # Handle historical queries
    if any(word in query for word in ['recent', 'lately', 'past', 'been', 'have']):
        response = ["Objects seen in the last 30 seconds:"]
        for class_name, objects in context["recent"].items():
            # Count unique instances
            unique_positions = set()
            for obj in objects:
                pos = (
                    round(obj['position']['x'], 1),
                    round(obj['position']['y'], 1),
                    round(obj['position']['z'], 1)
                )
                unique_positions.add(pos)
            
            response.append(f"\n{class_name}:")
            response.append(f"  - Seen {len(unique_positions)} unique instance(s)")
            response.append(f"  - Total detections: {len(objects)}")
            
            # Get current instances of this class
            current_objects = context["current"].get(class_name, [])
            if current_objects:
                response.append("  - Currently tracked:")
                response.append(format_detection_details(current_objects))
        
        return "\n".join(response)
    
    # Handle specific object queries
    for class_name in context["current"].keys():
        if class_name.lower() in query:
            objects = context["current"][class_name]
            if not objects:
                return f"No {class_name} currently tracked."
            
            response = [f"Currently tracked {class_name}(s) ({len(objects)}):"]
            response.append(format_detection_details(objects))
            return "\n".join(response)
    
    # Handle distance-related queries
    if any(word in query for word in ['closest', 'nearest', 'farthest', 'distance']):
        all_objects = []
        for class_name, objects in context["current"].items():
            for obj in objects:
                all_objects.append((class_name, obj))
        
        if not all_objects:
            return "No objects are currently tracked to measure distance to."
        
        if 'farthest' in query:
            obj = max(all_objects, key=lambda x: x[1]['distance'])
        else:  # Default to closest
            obj = min(all_objects, key=lambda x: x[1]['distance'])
        
        class_name, obj_data = obj
        response = [
            f"The {'farthest' if 'farthest' in query else 'closest'} object is a {class_name}:",
            format_detection_details([obj_data])
        ]
        return "\n".join(response)
    
    # Default to current state if query isn't specific
    return get_detected_objects() 