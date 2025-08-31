import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from execute.actions import execute_action_sequence
from mistral_ai.mistral import Mistralmodel
vlm_client = Mistralmodel()
if __name__ == "__main__":
    actions = [
        {"type": "open", "target": "home", "parameters": {}},
        {"type": "perceive", "target": "cucumber", "parameters": {}},
        {"type": "move", "target": "cucumber", "parameters": {}},
        {"type": "grasp_otherthings", "target": "cucumber", "parameters": {}},
        {"type": "perceive", "target": "soup pot", "parameters": {}},
        {"type": "move", "target": "soup pot", "parameters": {}},
        {"type": "stir", "target": "soup pot", "parameters": {"stir_time": 10}},
        {"type": "return_back", "target": "cucumber", "parameters": {}},
        {"type": "perceive", "target": "tomato", "parameters": {}},
        {"type": "move", "target": "tomato", "parameters": {}},
        {"type": "grasp_otherthings", "target": "tomato", "parameters": {}},
        {"type": "perceive", "target": "user person", "parameters": {}},
        {"type": "move", "target": "user person", "parameters": {}},
        {"type": "open", "target": "user person", "parameters": {}},
        {"type": "close", "target": "user person", "parameters": {}},
        {"type": "add", "target": "soup pot", "parameters": {"times": 2}},
        {"type": "reset", "target": "home", "parameters": {}}
    ]

    execute_action_sequence(actions,vlm_client)