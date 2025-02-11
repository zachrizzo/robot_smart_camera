from .base_agent import BaseAgent
import asyncio
import json
import threading
from typing import AsyncGenerator, Optional, Dict, Any
import websockets
import base64
import numpy as np
from rclpy.node import Node
import rclpy
import uuid
from datetime import datetime, timedelta

class VoiceAgent(BaseAgent):
    def __init__(self, node: Node, model: str = "gpt-4o-realtime-preview-2024-12-17", api_key: Optional[str] = None):
        super().__init__("voice_agent")
        self.node = node
        self.model = model
        self.api_key = api_key
        self.websocket = None
        self._lock = threading.Lock()
        self.audio_buffer = []
        self.conversation_history = []
        
    @property
    def name(self) -> str:
        """Return the name of this agent"""
        return "Voice Agent"

    async def generate(self, prompt: str) -> AsyncGenerator[Dict[str, Any], None]:
        """Generate a response to the given prompt"""
        async for response in self.send_text(prompt):
            yield response

    async def stream(self, audio_data: bytes) -> AsyncGenerator[Dict[str, Any], None]:
        """Stream audio data and get responses"""
        async for response in self.process_audio(audio_data):
            yield response

    async def _init_websocket(self):
        """Initialize WebSocket connection to OpenAI Realtime API"""
        if not self.websocket:
            uri = f"wss://api.openai.com/v1/realtime?model={self.model}"
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "OpenAI-Beta": "realtime=v1"
            }
            try:
                self.websocket = await websockets.connect(uri, extra_headers=headers)
                self.node.get_logger().info("Connected to OpenAI Realtime API")
                
                # Send initial configuration
                await self.websocket.send(json.dumps({
                    "type": "response.create",
                    "response": {
                        "modalities": ["text", "audio"],
                        "instructions": """You are a helpful robot assistant. Be concise but friendly in your responses."""
                    }
                }))
            except Exception as e:
                self.node.get_logger().error(f"Error initializing websocket: {str(e)}")
                raise

    async def process_audio(self, audio_data: bytes) -> AsyncGenerator[Dict[str, Any], None]:
        """Process audio input and get streaming response"""
        try:
            if not self.websocket:
                await self._init_websocket()
                
            # Send audio data
            audio_b64 = base64.b64encode(audio_data).decode('utf-8')
            await self.websocket.send(json.dumps({
                "type": "input_audio_buffer.append",
                "audio": audio_b64
            }))
            
            # Signal end of audio input
            await self.websocket.send(json.dumps({
                "type": "input_audio_buffer.commit"
            }))
            
            # Create response
            await self.websocket.send(json.dumps({
                "type": "response.create",
                "response": {
                    "modalities": ["text", "audio"]
                }
            }))
            
            # Process streaming response
            while True:
                try:
                    response = await self.websocket.recv()
                    data = json.loads(response)
                    event_type = data["type"]
                    
                    if event_type == "response.text.delta":
                        yield {"type": "text", "data": data["delta"]}
                    elif event_type == "response.audio.delta":
                        audio_bytes = base64.b64decode(data["delta"])
                        yield {"type": "audio", "data": audio_bytes}
                    elif event_type == "response.done":
                        yield {"type": "complete", "data": data}
                        break
                    elif event_type == "error":
                        error_msg = f"Error: {data.get('message', 'Unknown error')}"
                        self.node.get_logger().error(error_msg)
                        yield {"type": "error", "data": error_msg}
                        break
                        
                except websockets.exceptions.ConnectionClosed:
                    self.node.get_logger().error("WebSocket connection closed unexpectedly")
                    break
                except Exception as e:
                    self.node.get_logger().error(f"Error processing response: {str(e)}")
                    yield {"type": "error", "data": str(e)}
                    break
        except Exception as e:
            self.node.get_logger().error(f"Error in process_audio: {str(e)}")
            yield {"type": "error", "data": str(e)}

    async def send_text(self, text: str) -> AsyncGenerator[Dict[str, Any], None]:
        """Send text input and get streaming response"""
        try:
            if not self.websocket:
                await self._init_websocket()
                
            # Create text input
            await self.websocket.send(json.dumps({
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": text
                        }
                    ]
                }
            }))
            
            # Create response
            await self.websocket.send(json.dumps({
                "type": "response.create",
                "response": {
                    "modalities": ["text", "audio"]
                }
            }))
            
            # Process streaming response
            while True:
                try:
                    response = await self.websocket.recv()
                    data = json.loads(response)
                    event_type = data["type"]
                    
                    if event_type == "response.text.delta":
                        yield {"type": "text", "data": data["delta"]}
                    elif event_type == "response.audio.delta":
                        audio_bytes = base64.b64decode(data["delta"])
                        yield {"type": "audio", "data": audio_bytes}
                    elif event_type == "response.done":
                        yield {"type": "complete", "data": data}
                        break
                    elif event_type == "error":
                        error_msg = f"Error: {data.get('message', 'Unknown error')}"
                        self.node.get_logger().error(error_msg)
                        yield {"type": "error", "data": error_msg}
                        break
                        
                except websockets.exceptions.ConnectionClosed:
                    self.node.get_logger().error("WebSocket connection closed unexpectedly")
                    break
                except Exception as e:
                    self.node.get_logger().error(f"Error processing response: {str(e)}")
                    yield {"type": "error", "data": str(e)}
                    break
        except Exception as e:
            self.node.get_logger().error(f"Error in send_text: {str(e)}")
            yield {"type": "error", "data": str(e)}

    async def close(self):
        """Close the WebSocket connection"""
        if self.websocket:
            try:
                await self.websocket.close()
            except Exception as e:
                self.node.get_logger().error(f"Error closing websocket: {str(e)}")
            finally:
                self.websocket = None 