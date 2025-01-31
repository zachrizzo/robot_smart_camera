from typing import Dict, List, Optional, Any, TypedDict, Annotated
from langchain_core.messages import AIMessage, HumanMessage
from langgraph.graph import StateGraph, END
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolExecutor

from .agents.base_agent import BaseAgent
from .agents.claude_agent import ClaudeAgent
from .agents.openai_agent import OpenAIAgent
from .agents.ollama_agent import OllamaAgent

class AgentState(TypedDict):
    messages: Annotated[List[Any], add_messages]

class MultiAgentSystem:
    def __init__(self):
        self.agents: Dict[str, BaseAgent] = {}
        self.current_agent: Optional[str] = None
        self.workflow = self._create_workflow()
        
    def add_agent(self, agent: BaseAgent, api_key: Optional[str] = None) -> None:
        """Add an agent to the system."""
        self.agents[agent.name] = agent
        if self.current_agent is None:
            self.current_agent = agent.name
            
    def switch_agent(self, agent_name: str) -> bool:
        """Switch to a different agent."""
        if agent_name in self.agents:
            self.current_agent = agent_name
            return True
        return False
        
    def _create_workflow(self) -> StateGraph:
        """Create the LangGraph workflow."""
        workflow = StateGraph(AgentState)
        
        # Define the nodes
        async def agent_node(state: AgentState) -> Dict:
            """Process input through the current agent."""
            if not self.current_agent or self.current_agent not in self.agents:
                return {"messages": state["messages"] + [AIMessage(content="No agent selected")]}
                
            agent = self.agents[self.current_agent]
            messages = state["messages"]
            last_message = messages[-1]
            
            if isinstance(last_message, HumanMessage):
                response = await agent.generate(last_message.content)
                return {"messages": messages + [AIMessage(content=response)]}
                
            return {"messages": messages}
            
        # Add nodes to the graph
        workflow.add_node("agent", agent_node)
        
        # Add edges
        workflow.add_edge("agent", END)
        
        # Set entry point
        workflow.set_entry_point("agent")
        
        # Compile the workflow
        return workflow.compile()
        
    async def process_message(self, message: str) -> str:
        """Process a message through the current agent."""
        if not self.current_agent:
            return "No agent selected"
            
        state = {"messages": [HumanMessage(content=message)]}
        result = await self.workflow.ainvoke(state)
        return result["messages"][-1].content
        
    @property
    def available_agents(self) -> List[str]:
        """Get list of available agents."""
        return list(self.agents.keys())
