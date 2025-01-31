import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import os
from threading import Thread
from queue import Queue

from .ai_agent.graph import MultiAgentSystem
from .ai_agent.agents.claude_agent import ClaudeAgent
from .ai_agent.agents.openai_agent import OpenAIAgent
from .ai_agent.agents.ollama_agent import OllamaAgent

class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')
        
        # Initialize multi-agent system
        self.agent_system = MultiAgentSystem()
        
        # Message queue for async processing
        self.message_queue = Queue()
        
        # Store latest detection information
        self.latest_detections = ""
        
        # Declare parameters
        self.declare_parameter('anthropic_api_key', os.getenv('ANTHROPIC_API_KEY', ''))
        self.declare_parameter('openai_api_key', os.getenv('OPENAI_API_KEY', ''))
        self.declare_parameter('ollama_base_url', os.getenv('OLLAMA_BASE_URL', 'http://localhost:11434'))
        
        # Initialize publishers
        self.response_pub = self.create_publisher(String, '/agent/response', 10)
        self.status_pub = self.create_publisher(String, '/agent/status', 10)
        
        # Initialize subscribers
        self.user_input_sub = self.create_subscription(
            String,
            '/agent/user_input',
            self.user_input_callback,
            10
        )
        self.switch_agent_sub = self.create_subscription(
            String,
            '/agent/switch',
            self.switch_agent_callback,
            10
        )
        self.detections_sub = self.create_subscription(
            String,
            '/detection/objects',
            self.detections_callback,
            10
        )
        
        # Initialize agents
        self._setup_agents()
        
        # Start async processing thread
        self.async_thread = Thread(target=self._run_async_loop, daemon=True)
        self.async_thread.start()
        
        # Publish initial status
        self._publish_status()
        self.get_logger().info('Agent node initialized')
        
    def _setup_agents(self):
        """Set up all available agents using parameters"""
        # Claude agent
        anthropic_key = self.get_parameter('anthropic_api_key').value
        if anthropic_key:
            self.agent_system.add_agent(ClaudeAgent(api_key=anthropic_key))
            self.get_logger().info('Added Claude agent')
            
        # OpenAI agent
        openai_key = self.get_parameter('openai_api_key').value
        if openai_key:
            self.agent_system.add_agent(OpenAIAgent(api_key=openai_key))
            self.get_logger().info('Added OpenAI agent')
            
        # Ollama agent
        ollama_url = self.get_parameter('ollama_base_url').value
        if ollama_url:
            self.agent_system.add_agent(OllamaAgent(base_url=ollama_url))
            self.get_logger().info('Added Ollama agent')
            
        if not self.agent_system.available_agents:
            self.get_logger().warning('No agents were initialized. Please check parameters.')
        else:
            agents = ', '.join(self.agent_system.available_agents)
            self.get_logger().info(f'Available agents: {agents}')
            self.get_logger().info(f'Current agent: {self.agent_system.current_agent}')
            
    def _publish_status(self):
        """Publish current agent status"""
        status_msg = String()
        available = ', '.join(self.agent_system.available_agents)
        current = self.agent_system.current_agent or 'None'
        status_msg.data = f'Available agents: {available}\nCurrent agent: {current}'
        self.status_pub.publish(status_msg)
        
    def user_input_callback(self, msg: String):
        """Handle user input messages by queueing them for async processing"""
        self.message_queue.put(('chat', msg.data))
        
    def switch_agent_callback(self, msg: String):
        """Handle agent switching requests"""
        agent_name = msg.data.strip().lower()
        success = self.agent_system.switch_agent(agent_name)
        
        status_msg = String()
        if success:
            status_msg.data = f'Switched to agent: {agent_name}'
        else:
            available = ', '.join(self.agent_system.available_agents)
            status_msg.data = f'Failed to switch to agent: {agent_name} (available: {available})'
            
        self.status_pub.publish(status_msg)
        self._publish_status()
        
    def detections_callback(self, msg: String):
        """Store latest detection information"""
        self.latest_detections = msg.data
        
    async def _process_message(self, content: str) -> str:
        """Process a message with context about detected objects"""
        # Always include the latest detection information in the context
        if self.latest_detections:
            context = f"Current object detections from the robot's camera:\n{self.latest_detections}\n\nUser question: {content}"
            return await self.agent_system.process_message(context)
        
        return await self.agent_system.process_message(content)
        
    def _run_async_loop(self):
        """Run the async event loop in a separate thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        while rclpy.ok():
            try:
                # Process messages from queue
                if not self.message_queue.empty():
                    msg_type, content = self.message_queue.get()
                    
                    if msg_type == 'chat':
                        # Process chat message
                        response = loop.run_until_complete(
                            self._process_message(content)
                        )
                        
                        # Publish response
                        response_msg = String()
                        response_msg.data = response
                        self.response_pub.publish(response_msg)
                        
                # Small sleep to prevent busy waiting
                loop.run_until_complete(asyncio.sleep(0.1))
                
            except Exception as e:
                self.get_logger().error(f'Error in async loop: {str(e)}')
                
        loop.close()

def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 