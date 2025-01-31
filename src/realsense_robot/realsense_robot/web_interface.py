#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import base64
from io import BytesIO
from std_msgs.msg import String
import asyncio
import websockets
import json
import os
from aiohttp import web

class WebInterface(Node):
    def __init__(self, loop):
        super().__init__('web_interface')
        
        # Store event loop
        self.loop = loop
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Store latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Store connected websocket clients
        self._ws_clients = set()
        
        # Create publishers and subscribers
        self.agent_input_pub = self.create_publisher(String, '/agent/user_input', 10)
        self.agent_response_sub = self.create_subscription(
            String,
            '/agent/response',
            self.agent_response_callback,
            10
        )
        self.agent_status_sub = self.create_subscription(
            String,
            '/agent/status',
            self.agent_status_callback,
            10
        )
        
        # Create model selection publisher
        self.model_selection_pub = self.create_publisher(String, '/agent/model_selection', 10)
        
        # Subscribe to the visualization topic
        self.create_subscription(
            Image,
            '/detection_image',
            self.image_callback,
            10)
        
        self.get_logger().info('Web interface node initialized')
            
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
    
    def get_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def agent_response_callback(self, msg):
        """Handle responses from the AI agent"""
        try:
            self.get_logger().info(f'Received agent response: {msg.data}')
            # Format the response as a proper message object
            response = {
                'type': 'agent_response',
                'content': msg.data,
                'timestamp': self.get_clock().now().to_msg().sec
            }
            # Run the broadcast in the event loop
            asyncio.run_coroutine_threadsafe(
                self.broadcast_message(response), 
                self.loop
            ).result()  # Wait for the result to ensure it's sent
        except Exception as e:
            self.get_logger().error(f'Error in agent_response_callback: {str(e)}')
        
    def agent_status_callback(self, msg):
        """Handle status updates from the AI agent"""
        try:
            self.get_logger().info(f'Received agent status: {msg.data}')
            status = {
                'type': 'agent_status',
                'content': msg.data,
                'timestamp': self.get_clock().now().to_msg().sec
            }
            asyncio.run_coroutine_threadsafe(
                self.broadcast_message(status),
                self.loop
            ).result()  # Wait for the result to ensure it's sent
        except Exception as e:
            self.get_logger().error(f'Error in agent_status_callback: {str(e)}')
        
    async def broadcast_message(self, message):
        """Send message to all connected clients"""
        try:
            if not self._ws_clients:
                self.get_logger().warn('No WebSocket clients connected to broadcast message to')
                return
                
            self.get_logger().info(f'Broadcasting message to {len(self._ws_clients)} clients: {message}')
            
            # Ensure message is properly formatted
            if isinstance(message, str):
                message = {
                    'type': 'agent_response',
                    'content': message,
                    'timestamp': self.get_clock().now().to_msg().sec
                }
            
            message_json = json.dumps(message)
            
            # Keep track of clients to remove
            clients_to_remove = set()
            
            for client in self._ws_clients:
                try:
                    await client.send_str(message_json)
                    self.get_logger().info(f'Successfully sent message to client')
                except Exception as e:
                    self.get_logger().error(f'Error sending to client: {str(e)}')
                    clients_to_remove.add(client)
            
            # Remove any disconnected clients
            for client in clients_to_remove:
                self._ws_clients.remove(client)
                
        except Exception as e:
            self.get_logger().error(f'Error in broadcast_message: {str(e)}')
    
    async def websocket_handler(self, request):
        """Handle websocket connections"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self._ws_clients.add(ws)
        self.get_logger().info(f'New WebSocket client connected. Total clients: {len(self._ws_clients)}')
        
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        self.get_logger().info(f'Received WebSocket message: {data}')
                        
                        if data['type'] == 'user_input':
                            # Publish user input to ROS topic
                            ros_msg = String()
                            ros_msg.data = data['content']
                            self.agent_input_pub.publish(ros_msg)
                            self.get_logger().info(f'Published user input: {ros_msg.data}')
                            
                            # Send immediate acknowledgment
                            await ws.send_str(json.dumps({
                                'type': 'status',
                                'content': 'Message received, waiting for response...'
                            }))
                            
                        elif data['type'] == 'model_change':
                            # Publish model selection to ROS topic
                            ros_msg = String()
                            ros_msg.data = data['content']
                            self.model_selection_pub.publish(ros_msg)
                            self.get_logger().info(f'Published model change: {ros_msg.data}')
                            
                    except Exception as e:
                        self.get_logger().error(f'Error processing message: {str(e)}')
                        await ws.send_str(json.dumps({
                            'type': 'error',
                            'content': f'Error processing message: {str(e)}'
                        }))
                        
                elif msg.type == web.WSMsgType.ERROR:
                    self.get_logger().error(f'WebSocket error: {ws.exception()}')
        finally:
            self._ws_clients.remove(ws)
            self.get_logger().info(f'WebSocket client disconnected. Remaining clients: {len(self._ws_clients)}')
        return ws
    
    async def video_feed_handler(self, request):
        """Handle video feed requests"""
        response = web.StreamResponse()
        response.content_type = 'multipart/x-mixed-replace; boundary=frame'
        await response.prepare(request)
        
        try:
            while True:
                frame = self.get_frame()
                if frame is not None:
                    _, jpeg = cv2.imencode('.jpg', frame)
                    await response.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n'
                    )
                await asyncio.sleep(0.033)  # ~30 fps
        except ConnectionResetError:
            pass
        
        return response

def spin_ros(node):
    """Function to spin ROS node in a separate thread"""
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Error in ROS spin: {str(e)}')
    finally:
        node.destroy_node()

async def init_web_app(node):
    """Initialize and run the web application"""
    # Create web application
    app = web.Application()
    
    # Add routes
    static_path = os.path.join(os.path.dirname(__file__), 'static')
    app.router.add_get('/', lambda r: web.FileResponse(os.path.join(static_path, 'index.html')))
    app.router.add_get('/video_feed', node.video_feed_handler)
    app.router.add_get('/ws', node.websocket_handler)
    app.router.add_static('/static', static_path)
    
    # Start web server
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8000)
    await site.start()
    
    return runner, site

def main():
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create event loop
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        # Create node with event loop
        node = WebInterface(loop)
        
        # Start ROS spinning in a separate thread
        ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
        ros_thread.start()
        
        # Run web application in the event loop
        runner = None
        try:
            runner, site = loop.run_until_complete(init_web_app(node))
            node.get_logger().info('Web server started at http://0.0.0.0:8000')
            loop.run_forever()
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down...')
        finally:
            if runner:
                loop.run_until_complete(runner.cleanup())
            loop.close()
            
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 