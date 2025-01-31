import os
import asyncio
from src.realsense_robot.realsense_robot.ai_agent.graph import MultiAgentSystem
from src.realsense_robot.realsense_robot.ai_agent.agents.claude_agent import ClaudeAgent
from src.realsense_robot.realsense_robot.ai_agent.agents.openai_agent import OpenAIAgent
from src.realsense_robot.realsense_robot.ai_agent.agents.ollama_agent import OllamaAgent

class AgentInterface:
    def __init__(self):
        # Initialize multi-agent system
        self.agent_system = MultiAgentSystem()
        
        # Initialize agents with environment variables
        self._setup_agents()
        
    def _setup_agents(self):
        """Set up all available agents using environment variables"""
        # Claude agent
        if os.getenv('ANTHROPIC_API_KEY'):
            self.agent_system.add_agent(ClaudeAgent(api_key=os.getenv('ANTHROPIC_API_KEY')))
            print("Added Claude agent")
            
        # OpenAI agent
        if os.getenv('OPENAI_API_KEY'):
            self.agent_system.add_agent(OpenAIAgent(api_key=os.getenv('OPENAI_API_KEY')))
            print("Added OpenAI agent")
            
        # Ollama agent
        if os.getenv('OLLAMA_BASE_URL'):
            self.agent_system.add_agent(OllamaAgent(base_url=os.getenv('OLLAMA_BASE_URL')))
            print("Added Ollama agent")
            
        if not self.agent_system.available_agents:
            print("Warning: No agents were initialized. Please check your environment variables.")
        else:
            print(f"Available agents: {', '.join(self.agent_system.available_agents)}")
            print(f"Current agent: {self.agent_system.current_agent}")

    async def chat(self, message: str) -> str:
        """Process a message and return the response"""
        try:
            return await self.agent_system.process_message(message)
        except Exception as e:
            return f"Error processing message: {str(e)}"

    def switch_agent(self, agent_name: str) -> str:
        """Switch to a different agent"""
        success = self.agent_system.switch_agent(agent_name)
        if success:
            return f"Switched to {agent_name}"
        return f"Failed to switch to {agent_name} (available agents: {', '.join(self.agent_system.available_agents)})"

async def main():
    # Initialize the agent interface
    agent = AgentInterface()
    
    print("\nType 'switch <agent_name>' to switch agents")
    print("Type 'exit' or 'quit' to end the conversation")
    print("Type 'agents' to see available agents\n")
    
    # Example conversation
    while True:
        try:
            message = input("You: ").strip()
            
            # Handle special commands
            if message.lower() in ['exit', 'quit']:
                break
            elif message.lower() == 'agents':
                print(f"Available agents: {', '.join(agent.agent_system.available_agents)}")
                print(f"Current agent: {agent.agent_system.current_agent}")
                continue
            elif message.lower().startswith('switch '):
                new_agent = message[7:].strip()
                print(agent.switch_agent(new_agent))
                continue
                
            # Process regular message
            response = await agent.chat(message)
            print(f"Agent: {response}")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {str(e)}")
            continue

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nGoodbye!") 