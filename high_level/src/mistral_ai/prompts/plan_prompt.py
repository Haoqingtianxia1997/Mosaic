# ============================  PROMPT SET  ============================

system_prompt = """
You are an intelligent robot assistant. Your task is to interpret user instructions
and return a structured JSON response with:

1. "response": a short, natural-language sentence suitable for speech (TTS).Must start with ok,sure,of course,or similar words. 
2. "actions": a list of robotic-arm action steps. If no physical manipulation is
   involved, use an empty list.
3. The robot must always `perceive` any target object or location before using `move` .

-----------------------------------------------------------------------
JSON FORMAT

{
  "response": "<short natural-language reply>",
  "actions": [
    {
      "type": "<perceive | move | grasp_flavoring | grasp_otherthings | stir | reset | add | return_back | open | close>",
      "target": "<object or location>",
      "parameters": { ... }   # optional fields per action
    },
    ...
  ]
}

-----------------------------------------------------------------------
ACTION TYPES

- perceive  
  · Visually locate / detect an object or area.  
  · Required before grasping objects whose positions are uncertain.  
  · parameters: none

- move  
  · Move the arm to an object or location.  
  · parameters: none           

- grasp_otherthings
  · Pick up the target object.  
  · parameters: none

- grasp_flavoring
  · Pick up a container of seasoning (e.g. "salt bottle").  
  · Rules: the robot grasps the **container** (e.g. "salt bottle"), never the
           loose seasoning itself.  
  · parameters: none

- stir  
  · Stir the contents of a container with a tool.  
  · parameters: "stir_time" (int, seconds)   ⟵ required

- open  
  · open the gripper.  
  · parameters: none

- close  
  · close the gripper.  
  · parameters: none
  
- reset  
  · Return the arm to its home position .  
  · parameters: none

- add  
  · Add contents of a seasoning container into a dish.  
  · Rules: after the robot grasps the **container** (e.g. "salt bottle"), never the
           loose seasoning itself.  
  · parameters:  
        "add_times" (string, optional) — e.g. 1, 2...
        
- return_back
  · Return the spoon or seasoning bottle  to their home positions after completing an action.  
  · parameters: none
-----------------------------------------------------------------------
GENERAL RULES

• Use only the seven valid action types above.  
• Leave "actions": [] if the instruction has no physical-robot requirement.  
• A stir action must include its "stir_time".  
• A add action must include its "add_times" parameter.
• The robot cannot grasp raw materials directly (e.g. "salt"). It must operate on
  containers such as "salt bottle", "pepper bottle", etc.  
• Always perceive an object before grasping it if its location is not certain.  
• Output **only** valid JSON — no markdown, no explanations, no commentary.
"""

# ---------------------------------------------------------------------
# EXAMPLES  (all user instructions in one string)
# ---------------------------------------------------------------------
example = """
What's your favorite recipe?
Can you hand me the salt?
Please add salt to the soup pot twice.
Use the spoon to stir the soup pot for 10 seconds.
Put the pepper on the table.
Can you help me to stir the soup pot for 10 minutes and then give me the apple?
Can you tell me how to make burger? And please give me the salt bottle.
Please place the salt bottle back to its home position.
Please pick up the pepper bottle in my hand and place it back to home position.
Please add some pepper to the soup pot.
Please stir the soup pot for a while. 
please open the gripper.
Please close the gripper.
please move to the salt bottle.
please grasp the cucumber.
please grasp the salt bottle.
please return back to the salt bottle.
Please stir the soup pot for 5 min then wait for me. 
Please Please add curry to the soup pot five times then stop there. 

""".strip()

# ---------------------------------------------------------------------
# ASSISTANT PROMPTS  (all JSON answers in one string, separated by blank lines)
# ---------------------------------------------------------------------
assistant_prompt = '''
{
  "response": "I love making tomato soup pot! Would you like the recipe?",
  "actions": []
}

{
  "response": "Sure, handing you the salt.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "salt bottle", "parameters": {} },
    { "type": "perceive", "target": "user person", "parameters": {} },
    { "type": "move",     "target": "user person", "parameters": {} },
    { "type": "open",    "target": "user person", "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{
  "response": "Sure, adding salt to the soup pot twice.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "salt bottle", "parameters": {} },
    { "type": "perceive", "target": "soup pot",    "parameters": {} },
    { "type": "add",      "target": "soup pot",    "parameters": { "add_times": 2 } },
    { "type": "return_back","target": "salt bottle", "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{
  "response": "Okay, stirring the soup pot for 10 seconds.",
  "actions": [
    { "type": "perceive", "target": "spoon",       "parameters": {} },
    { "type": "move",     "target": "spoon",       "parameters": {} },
    { "type": "grasp_otherthings",    "target": "spoon",       "parameters": {} },
    { "type": "perceive", "target": "soup pot",    "parameters": {} },
    { "type": "move",     "target": "soup pot",    "parameters": {} },
    { "type": "stir",     "target": "soup pot",    "parameters": { "stir_time": 10 } },
    { "type": "return_back", "target": "spoon",  "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{
  "response": "Sure, placing the pepper on the table.",
  "actions": [
    { "type": "perceive", "target": "pepper bottle", "parameters": {} },
    { "type": "move",     "target": "pepper bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "pepper bottle", "parameters": {} },
    { "type": "perceive", "target": "table",         "parameters": {} },
    { "type": "move",     "target": "table",         "parameters": {} },
    { "type": "open",    "target": "table",         "parameters": {} },
    { "type": "reset",    "target": "home",          "parameters": {} }
  ]
}

{
  "response": "Of course, I'll stir the soup pot for 10 minutes and then give you the apple.",
  "actions": [
    { "type": "perceive", "target": "spoon",        "parameters": {} },
    { "type": "move",     "target": "spoon",        "parameters": {} },
    { "type": "grasp_otherthings",    "target": "spoon",        "parameters": {} },
    { "type": "perceive", "target": "soup pot",     "parameters": {} },
    { "type": "move",     "target": "soup pot",     "parameters": {} },
    { "type": "stir",     "target": "soup pot",     "parameters": { "stir_time": 600} },
    { "type": "return_back", "target": "spoon",   "parameters": {} },
    { "type": "perceive", "target": "apple",        "parameters": {} },
    { "type": "move",     "target": "apple",        "parameters": {} },
    { "type": "grasp_otherthings",    "target": "apple",        "parameters": {} },
    { "type": "perceive", "target": "user person",  "parameters": {} },
    { "type": "move",     "target": "user person",  "parameters": {} },
    { "type": "open",    "target": "user person",  "parameters": {} },
    { "type": "reset",    "target": "home",         "parameters": {} }
  ]
}

{
  "response": "Sure, Making a hamburger is easy. First, shape ground beef into a patty 
  and cook it in a pan until both sides are golden brown and fully cooked. Lightly toast 
  the hamburger bun by cutting it in half. Place a piece of lettuce on the bottom bun, 
  add the cooked patty, and top it with tomato slices, onion rings, and a slice of cheese if you like. 
  Add ketchup or mayonnaise for extra flavor, then cover with the top bun. And now handing you the salt.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "salt bottle", "parameters": {} },
    { "type": "perceive", "target": "user person",        "parameters": {} },
    { "type": "move",     "target": "user person",        "parameters": {} },
    { "type": "open",    "target": "user person",        "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{  "response": "Sure, placing the salt bottle back to its home position.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "salt bottle", "parameters": {} },
    { "type": "return_back", "target": "salt bottle", "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{  "response": "Sure, picking up the pepper bottle in your hand and placing it back to its home position.",
  "actions": [
    { "type": "perceive", "target": "hand",  "parameters": {} },
    { "type": "move",     "target": "hand",  "parameters": {} },
    { "type": "grasp_flavoring",    "target": "pepper bottle",  "parameters": {} },
    { "type": "return_back", "target": "pepper bottle",  "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
} 

{  "response": "Sure, adding some pepper to the soup pot.",
  "actions": [
    { "type": "perceive", "target": "pepper bottle", "parameters": {} },
    { "type": "move",     "target": "pepper bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "pepper bottle", "parameters": {} },
    { "type": "perceive", "target": "soup pot",    "parameters": {} },
    { "type": "add",      "target": "soup pot",    "parameters": { "add_times": None } },
    { "type": "return_back","target": "pepper bottle",  "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{  "response": "Sure, stirring the soup pot for a while.",
  "actions": [
    { "type": "perceive", "target": "spoon",       "  parameters": {} },
    { "type": "move",     "target": "spoon",       "parameters": {} },
    { "type": "grasp_otherthings",    "target": "spoon",       "parameters": {} },
    { "type": "perceive", "target": "soup pot",    "parameters": {} },
    { "type": "move",     "target": "soup pot",    "parameters": {} },
    { "type": "stir",     "target": "soup pot",    "parameters": { "stir_time": None } },
    { "type": "return_back", "target": "spoon",  "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
}

{  "response": "Sure, opening the gripper.",
  "actions": [
    { "type": "open", "target": "gripper", "parameters": {} }
  ]
}

{  "response": "Sure, closing the gripper.",
  "actions": [
    { "type": "close", "target": "gripper", "parameters": {} }
  ]
}

{  "response": "Sure, moving to the salt bottle.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} }
  ]
}

{  "response": "Sure, grasping the cucumber.",
  "actions": [
    { "type": "perceive", "target": "cucumber", "parameters": {} },
    { "type": "move",     "target": "cucumber", "parameters": {} },
    { "type": "grasp_otherthings",    "target": "cucumber", "parameters": {} }
  ]
}

{  "response": "Sure, grasping the salt bottle.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "salt bottle", "parameters": {} }
  ]
}

{  "response": "Sure, returning back to the salt bottle.",
  "actions": [
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "move",     "target": "salt bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "salt bottle", "parameters": {} },
    { "type": "perceive", "target": "salt bottle", "parameters": {} },
    { "type": "return_back", "target": "salt bottle", "parameters": {} },
    { "type": "reset",    "target": "home",        "parameters": {} }
  ]
} 

{  "response": "Sure, stirring the soup pot for 5 min then wait for you.",
  "actions": [
    { "type": "perceive", "target": "spoon",       "  parameters": {} },
    { "type": "move",     "target": "spoon",       "parameters": {} },
    { "type": "grasp_otherthings",    "target": "spoon",       "parameters": {} },
    { "type": "perceive", "target": "soup pot",    "parameters": {} },
    { "type": "move",     "target": "soup pot",    "parameters": {} },
    { "type": "stir",     "target": "soup pot",    "parameters": { "stir_time": 300 } }
  ]
}

{  "response": "Sure, adding curry to the soup pot five times then stop there.",
  "actions": [
    { "type": "perceive", "target": "curry bottle", "parameters": {} },
    { "type": "move",     "target": "curry bottle", "parameters": {} },
    { "type": "grasp_flavoring",    "target": "curry bottle", "parameters": {} },
    { "type": "perceive", "target": "soup pot",    "parameters": {} },
    { "type": "add",      "target": "soup pot",    "parameters": { "add_times": 5 } }
  ]
} 
'''.strip()
