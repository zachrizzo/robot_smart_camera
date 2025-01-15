#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
import threading
import base64
from io import BytesIO

app = Flask(__name__)

# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>RealSense Robot Vision</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            text-align: center;
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        .video-container {
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        img {
            max-width: 100%;
            height: auto;
            border-radius: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>RealSense Robot Vision</h1>
        <div class="video-container">
            <img src="{{ url_for('video_feed') }}" alt="Robot Vision">
        </div>
    </div>
</body>
</html>
"""

class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Store latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Subscribe to the visualization topic
        self.create_subscription(
            Image,
            '/detection_visualization',
            self.image_callback,
            10)
            
        self.get_logger().info('Web interface node initialized')
        
        # Start Flask app in a separate thread
        threading.Thread(target=self.run_flask, daemon=True).start()
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
            
    def run_flask(self):
        app.config['web_interface'] = self
        app.run(host='0.0.0.0', port=8080)
        
    def get_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

def generate_frames():
    while True:
        web_interface = app.config['web_interface']
        frame = web_interface.get_frame()
        
        if frame is not None:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    rclpy.init(args=args)
    node = WebInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 