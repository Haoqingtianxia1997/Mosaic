"""
LLM Interface Module - Code generation using OpenAI API
"""
import os
from openai import OpenAI


class LLMInterface:
    """OpenAI LLM Interface Class"""
    
    def __init__(self, api_key=None, model="gpt-4o"):
        """
        Initialize LLM interface
        
        Args:
            api_key: OpenAI API key, reads from environment variable if None
            model: Model name to use
        """
        if api_key is None:
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key is None:
                raise ValueError("Please set OPENAI_API_KEY environment variable or provide api_key parameter")
        
        self.client = OpenAI(api_key=api_key)
        self.model = model
        print(f"LLM interface initialized successfully, using model: {self.model}")
    
    def generate_code(self, prompt, system_prompt=None, temperature=0.7, max_tokens=2000):
        """
        Generate code
        
        Args:
            prompt: User prompt
            system_prompt: System prompt
            temperature: Temperature parameter controlling randomness
            max_tokens: Maximum number of tokens
            
        Returns:
            Generated code string
        """
        messages = []
        
        if system_prompt:
            messages.append({
                "role": "system",
                "content": system_prompt
            })
        
        messages.append({
            "role": "user",
            "content": prompt
        })
        
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens
            )
            
            generated_code = response.choices[0].message.content
            return generated_code
            
        except Exception as e:
            print(f"Error calling LLM: {e}")
            raise
    
    def extract_code_block(self, response):
        """
        Extract code block from LLM response
        
        Args:
            response: Full LLM response
            
        Returns:
            Extracted pure code string
        """
        # Look for code block markers
        if "```python" in response:
            start = response.find("```python") + len("```python")
            end = response.find("```", start)
            if end != -1:
                return response[start:end].strip()
        elif "```" in response:
            start = response.find("```") + len("```")
            end = response.find("```", start)
            if end != -1:
                return response[start:end].strip()
        
        # If no code block markers found, return entire response
        return response.strip()


# Test function
def test_llm_interface():
    """Test LLM interface"""
    try:
        # Read API key from environment variable
        llm = LLMInterface()
        
        # Simple test
        prompt = "Write a simple Python function that adds two numbers."
        response = llm.generate_code(prompt)
        print("LLM response:")
        print(response)
        print("\nExtracted code:")
        print(llm.extract_code_block(response))
        
    except Exception as e:
        print(f"Test failed: {e}")


if __name__ == "__main__":
    test_llm_interface()
