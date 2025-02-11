#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import json
import os
from aiohttp import web
import socketio
import base64
import asyncio
from starlette.applications import Starlette
from starlette.staticfiles import StaticFiles
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route
from .ai_agent.agents.voice_agent import VoiceAgent
from .ai_agent.agents.claude_agent import ClaudeAgent
from .ai_agent.agents.openai_agent import OpenAIAgent
from .ai_agent.agents.ollama_agent import OllamaAgent

class WebInterface(Node):
    def __init__(self):
        super().__init__('web_interface')
        
        # Set up static directory
        self.static_dir = os.path.join(os.path.dirname(__file__), 'static')
        if not os.path.exists(self.static_dir):
            os.makedirs(self.static_dir)
            self.get_logger().info(f'Created static directory at {self.static_dir}')
        
        # Initialize Socket.IO
        self.sio = socketio.AsyncServer(cors_allowed_origins='*', async_mode='asgi')
        
        # Set up routes
        routes = [
            Route('/', endpoint=self.index),
            Route('/agents', endpoint=self.list_agents)
        ]
        
        # Initialize Starlette app with routes
        self.app = Starlette(routes=routes)
        
        # Mount static files
        self.app.mount('/static', StaticFiles(directory=self.static_dir), name='static')
        
        # Add Socket.IO middleware
        self.app = socketio.ASGIApp(self.sio, self.app)
        
        # Initialize agents
        self.setup_agents()
        
        # Set up socket events
        self.setup_socket_events()
        
        # Create event loop for async operations
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        # Start server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info('Web interface initialized')

    def setup_agents(self):
        """Initialize all available agents"""
        self.agents = {}
        self.current_agent = None
        
        # Get API keys from environment
        openai_key = os.getenv('OPENAI_API_KEY')
        anthropic_key = os.getenv('ANTHROPIC_API_KEY')
        
        # Initialize agents if API keys are available
        if openai_key:
            self.agents['voice'] = VoiceAgent(self, api_key=openai_key)
            self.agents['openai'] = OpenAIAgent(self)
            self.current_agent = self.agents['voice']  # Default to voice agent
            
        if anthropic_key:
            self.agents['claude'] = ClaudeAgent(self)
            if not self.current_agent:
                self.current_agent = self.agents['claude']
                
        # Ollama doesn't need an API key
        self.agents['ollama'] = OllamaAgent(self)
        if not self.current_agent:
            self.current_agent = self.agents['ollama']
            
        if not self.agents:
            self.get_logger().error('No agents could be initialized')
            raise ValueError('No agents could be initialized')

    async def index(self, request):
        """Serve the index.html file"""
        return FileResponse(os.path.join(self.static_dir, 'index.html'))

    async def list_agents(self, request):
        """Return list of available agents"""
        agent_list = [{'id': agent_id, 'name': agent.name} for agent_id, agent in self.agents.items()]
        return JSONResponse({'agents': agent_list, 'current': self.current_agent.name})

    def setup_socket_events(self):
        """Set up Socket.IO event handlers"""
        @self.sio.event
        async def connect(sid, environ):
            self.get_logger().info(f'Client connected: {sid}')
            
        @self.sio.event
        async def disconnect(sid):
            self.get_logger().info(f'Client disconnected: {sid}')
                
        @self.sio.event
        async def audio_data(sid, data):
            """Handle incoming audio data"""
            try:
                if isinstance(self.current_agent, VoiceAgent):
                    audio_bytes = base64.b64decode(data['audio'])
                    async for response in self.current_agent.stream(audio_bytes):
                        if response["type"] == "text":
                            await self.sio.emit('text_response', {'text': response["data"]}, room=sid)
                        elif response["type"] == "audio":
                            await self.sio.emit('audio_response', {'audio': base64.b64encode(response["data"]).decode('utf-8')}, room=sid)
                        elif response["type"] == "error":
                            await self.sio.emit('error', {'message': response["data"]}, room=sid)
                else:
                    await self.sio.emit('error', {'message': 'Current agent does not support audio'}, room=sid)
            except Exception as e:
                self.get_logger().error(f'Error handling audio: {str(e)}')
                await self.sio.emit('error', {'message': str(e)}, room=sid)
                
        @self.sio.event
        async def text_message(sid, data):
            """Handle text messages"""
            try:
                async for response in self.current_agent.generate(data['message']):
                    if response["type"] == "text":
                        await self.sio.emit('text_response', {'text': response["data"]}, room=sid)
                    elif response["type"] == "audio":
                        await self.sio.emit('audio_response', {'audio': base64.b64encode(response["data"]).decode('utf-8')}, room=sid)
                    elif response["type"] == "error":
                        await self.sio.emit('error', {'message': response["data"]}, room=sid)
            except Exception as e:
                self.get_logger().error(f'Error handling text: {str(e)}')
                await self.sio.emit('error', {'message': str(e)}, room=sid)

        @self.sio.event
        async def switch_agent(sid, data):
            """Switch the current agent"""
            try:
                agent_id = data['agent']
                if agent_id in self.agents:
                    self.current_agent = self.agents[agent_id]
                    await self.sio.emit('agent_switched', {'agent': self.current_agent.name}, room=sid)
                else:
                    await self.sio.emit('error', {'message': f'Unknown agent: {agent_id}'}, room=sid)
            except Exception as e:
                self.get_logger().error(f'Error switching agent: {str(e)}')
                await self.sio.emit('error', {'message': str(e)}, room=sid)

    def run_server(self):
        """Run the ASGI server with uvicorn"""
        try:
            import uvicorn
            config = uvicorn.Config(
                app=self.app,
                host='0.0.0.0',
                port=8080,
                loop=self.loop,
                log_level='info'
            )
            server = uvicorn.Server(config)
            self.loop.run_until_complete(server.serve())
        except Exception as e:
            self.get_logger().error(f'Error running server: {str(e)}')

    def shutdown(self):
        """Clean up resources"""
        if hasattr(self, 'agents'):
            for agent in self.agents.values():
                if hasattr(agent, 'close'):
                    self.loop.run_until_complete(agent.close())
        if hasattr(self, 'loop'):
            self.loop.close()
        super().shutdown()

def main():
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create node
        node = WebInterface()
        
        # Spin ROS node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        # Cleanup
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 