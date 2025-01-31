from typing import AsyncIterator, Optional
import aiohttp
from .base_agent import BaseAgent

class OllamaAgent(BaseAgent):
    def __init__(self, model_name: str = "llama3.1", base_url: str = "http://localhost:11434"):
        super().__init__(model_name, None)
        self.base_url = base_url
        
    async def generate(self, prompt: str, **kwargs) -> str:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{self.base_url}/api/generate",
                json={
                    "model": self.model_name,
                    "prompt": prompt,
                    "stream": False
                }
            ) as response:
                result = await response.json()
                return result["response"]
        
    async def stream(self, prompt: str, **kwargs) -> AsyncIterator[str]:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{self.base_url}/api/generate",
                json={
                    "model": self.model_name,
                    "prompt": prompt,
                    "stream": True
                }
            ) as response:
                async for line in response.content:
                    if line:
                        result = line.decode().strip()
                        if result:
                            yield result
                
    @property
    def name(self) -> str:
        return "Ollama" 