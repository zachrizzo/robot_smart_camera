from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, AsyncIterator

class BaseAgent(ABC):
    """Base class for all LLM agents."""
    
    def __init__(self, model_name: str, api_key: Optional[str] = None):
        self.model_name = model_name
        self.api_key = api_key
        
    @abstractmethod
    async def generate(self, prompt: str, **kwargs) -> str:
        """Generate a response from the LLM."""
        pass
    
    @abstractmethod
    async def stream(self, prompt: str, **kwargs) -> AsyncIterator[str]:
        """Stream a response from the LLM."""
        pass
    
    @property
    @abstractmethod
    def name(self) -> str:
        """Return the name of the agent."""
        pass 