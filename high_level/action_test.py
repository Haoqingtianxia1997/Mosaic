import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from execute.actions import execute_action_sequence
from mistral_ai.mistral import Mistralmodel
vlm_client = Mistralmodel()
if __name__ == "__main__":
    actions = [
        {"type": "perceive", "target": "cup", "parameters": {}},
        # {"type": "move", "target": "bottle", "parameters": {}},
        # {"type": "grasp_otherthings", "target": "bottle", "parameters": {}},
        # {"type": "perceive", "target": "user person", "parameters": {}},
        # {"type": "move", "target": "user person", "parameters": {}},
        # {"type": "open", "target": "user person", "parameters": {}},
        # {"type": "reset", "target": "home", "parameters": {}}
    ]

    execute_action_sequence(actions,vlm_client)