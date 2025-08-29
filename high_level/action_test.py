import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from execute.actions import execute_action_sequence
from mistral_ai.mistral import Mistralmodel
vlm_client = Mistralmodel()
if __name__ == "__main__":
    actions = [
        {"type": "perceive", "target": "soup pot", "parameters": {}},
        {"type": "move", "target": "soup pot", "parameters": {}},
        {"type": "stir", "target": "soup pot", "parameters": {"stir_time": 10}},
        {"type": "return_back", "target": "spoon", "parameters": {}},
        {"type": "reset", "target": "home", "parameters": {}}
    ]

    execute_action_sequence(actions,vlm_client)