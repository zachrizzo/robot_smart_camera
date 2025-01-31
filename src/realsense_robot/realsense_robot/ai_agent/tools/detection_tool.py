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

class DetectionTool(Node):
    def __init__(self):
        super().__init__('detection_tool')
        
        # Store latest detections
        self.latest_detections: Dict[str, List[Dict]] = {}
        self.latest_image = None
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        
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
                            )
                        })
            
            self.latest_detections = detections
    
    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
    
    def get_detections(self) -> Dict[str, List[Dict]]:
        with self._lock:
            return self.latest_detections.copy()

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

@tool
def get_detected_objects(query: str = "") -> str:
    """Get information about detected objects. You can optionally specify a query to filter objects.
    The query can be a specific object class or a question about distances/positions."""
    
    # Initialize tool if needed
    if _detection_tool is None:
        init_detection_tool()
    
    # Get latest detections
    detections = _detection_tool.get_detections()
    
    if not detections:
        return "No objects currently detected."
    
    # If no specific query, summarize all detections
    if not query:
        summary = []
        for class_name, objects in detections.items():
            for obj in objects:
                dist = obj['distance']
                pos = obj['position']
                summary.append(
                    f"{class_name} detected at {dist:.2f}m distance "
                    f"(position: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f})"
                )
        return "\n".join(summary)
    
    # Handle specific queries
    query = query.lower()
    
    # Check for specific object class
    for class_name, objects in detections.items():
        if class_name.lower() in query:
            summary = []
            for obj in objects:
                dist = obj['distance']
                pos = obj['position']
                summary.append(
                    f"{class_name} detected at {dist:.2f}m distance "
                    f"(position: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f})"
                )
            return "\n".join(summary) if summary else f"No {class_name} currently detected."
    
    # Handle distance-related queries
    if any(word in query for word in ['closest', 'nearest', 'distance']):
        closest_obj = None
        min_dist = float('inf')
        for class_name, objects in detections.items():
            for obj in objects:
                if obj['distance'] < min_dist:
                    min_dist = obj['distance']
                    closest_obj = (class_name, obj)
        
        if closest_obj:
            class_name, obj = closest_obj
            dist = obj['distance']
            pos = obj['position']
            return (
                f"The closest object is a {class_name} at {dist:.2f}m distance "
                f"(position: x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f})"
            )
    
    # Default to summarizing all detections
    return get_detected_objects() 