import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from execute.actions import execute_action_sequence
from mistral_ai.mistral import Mistralmodel
vlm_client = Mistralmodel()
if __name__ == "__main__":
    actions = [
        # {"type": "open", "target": "home", "parameters": {}},
        # {"type": "perceive", "target": "cucumber", "parameters": {}},
        # {"type": "move", "target": "cucumber", "parameters": {}},
        # {"type": "grasp_otherthings", "target": "cucumber", "parameters": {}},
        # {"type": "perceive", "target": "soup pot", "parameters": {}},
        # {"type": "move", "target": "soup pot", "parameters": {}},
        # {"type": "stir", "target": "soup pot", "parameters": {"stir_time": 10}},
        # {"type": "return_back", "target": "cucumber", "parameters": {}},
        # {"type": "reset", "target": "home", "parameters": {}},
        # {"type": "perceive", "target": "cucumber", "parameters": {}},
        # {"type": "move", "target": "cucumber", "parameters": {}},
        # {"type": "grasp_otherthings", "target": "cucumber", "parameters": {}},
        # {"type": "perceive", "target": "user person", "parameters": {}},
        # {"type": "move", "target": "user person", "parameters": {}},
        # {"type": "open", "target": "user person", "parameters": {}},
        # {"type": "close", "target": "user person", "parameters": {}},
        # {"type": "add", "target": "soup pot", "parameters": {"times": 2}},
        # {"type": "reset", "target": "home", "parameters": {}}

        # # Test grasp detection
        # {"type": "close", "target": "user person", "parameters": {}},
        # {"type": "grasp_detection", "target": "user person", "parameters": {}},
        # {"type": "open", "target": "user person", "parameters": {}},
        # {"type": "grasp_detection", "target": "user person", "parameters": {}},

        # # Test multiple grasps return
        # {"type": "perceive", "target": "cucumber", "parameters": {}},
        # {"type": "get_grasps", "target": "cucumber", "parameters": {}},

        # test move offset
        {"type": "move_offset", "target": "user", "parameters": {"direction": "z", "delta": 0.1}},

        # # test get_objects_pos
        # {"type": "get_objects_pos", "target": "tomato", "parameters": {}},
    ]

    execute_action_sequence(actions,vlm_client)