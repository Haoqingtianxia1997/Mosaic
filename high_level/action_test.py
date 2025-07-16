import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from execute.actions import execute_action_sequence

if __name__ == "__main__":
    actions = [
        {"type": "perceive", "target": "tomato", "parameters": {}},
        {"type": "move", "target": "tomato", "parameters": {}},
        {"type": "grasp_flavoring", "target": "tomato", "parameters": {}},
        {"type": "perceive", "target": "user person", "parameters": {}},
        {"type": "move", "target": "user person", "parameters": {}},
        {"type": "open", "target": "user person", "parameters": {}},
        {"type": "reset", "target": "home", "parameters": {}}
    ]

    execute_action_sequence(actions)