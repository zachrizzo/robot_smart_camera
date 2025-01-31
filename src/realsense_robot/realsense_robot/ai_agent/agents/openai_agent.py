from typing import AsyncIterator, Optional
from openai import AsyncOpenAI
from .base_agent import BaseAgent

class OpenAIAgent(BaseAgent):
    def __init__(self, model_name: str = "gpt-4-turbo-preview", api_key: Optional[str] = None):
        super().__init__(model_name, api_key)
        self.client = AsyncOpenAI(api_key=api_key)
        
    async def generate(self, prompt: str, **kwargs) -> str:
        response = await self.client.chat.completions.create(
            model=self.model_name,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=kwargs.get("max_tokens", 1024)
        )
        return response.choices[0].message.content
        
    async def stream(self, prompt: str, **kwargs) -> AsyncIterator[str]:
        stream = await self.client.chat.completions.create(
            model=self.model_name,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=kwargs.get("max_tokens", 1024),
            stream=True
        )
        async for chunk in stream:
            if chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content
                
    @property
    def name(self) -> str:
        return "OpenAI" 