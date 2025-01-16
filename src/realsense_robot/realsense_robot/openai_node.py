import openai
import rclpy
from rclpy.node import Node

class OpenAINode(Node):
    def __init__(self):
        super().__init__('openai_node')
        self.client = openai.OpenAI()

    def run(self):
        # Example usage of OpenAI API
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": "Hello, how are you?"}]
        )
        self.get_logger().info(f"OpenAI response: {response.choices[0].message.content}")


    def ask_openai(self, question):
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": question}]
        )
        return response.choices[0].message.content

    