from typing import AsyncIterator, Optional
import anthropic
from .base_agent import BaseAgent

class ClaudeAgent(BaseAgent):
    def __init__(self, model_name: str = "claude-3-sonnet-20240229", api_key: Optional[str] = None):
        super().__init__(model_name, api_key)
        self.client = anthropic.AsyncAnthropic(api_key=api_key)
        
    async def generate(self, prompt: str, **kwargs) -> str:
        message = await self.client.messages.create(
            model=self.model_name,
            max_tokens=kwargs.get("max_tokens", 1024),
            messages=[{"role": "user", "content": prompt}]
        )
        return message.content[0].text
        
    async def stream(self, prompt: str, **kwargs) -> AsyncIterator[str]:
        stream = await self.client.messages.create(
            model=self.model_name,
            max_tokens=kwargs.get("max_tokens", 1024),
            messages=[{"role": "user", "content": prompt}],
            stream=True
        )
        async for chunk in stream:
            if chunk.content:
                yield chunk.content[0].text
                
    @property
    def name(self) -> str:
        return "Claude" 