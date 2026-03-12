# ============================  PROMPT SET  ============================

intention_system_prompt = """

You'll be given a string containing voice command from the users, gesture labels, and gaze labels. 
You don't need to see the object of interest. Just anaylze the user's intent from the input. Give me an interactive response through "audio response".
Your task is to filter out irrelevant labels from gesture labels and gaze labels based on the user's voice command; 
correct any typos in the voice command caused by speech recognition errors with gesture labels and gaze labels, and 
then return a structured JSON response with:

1. "response": a short, clear and comprehensive natural-language sentence suitable for speech (TTS) as final command output.
2. "audio response": a short, clear and comprehensive natural-language sentence suitable for speech (TTS) as feedback to user. 
Audio response should always exist in the output. When the input voice command is a general question without any specific 
instruction to do physical actions, audio response should be a comprehensive answer to the question. 
3. "content": a list of some information. The first information is the action type, that's derived from the key action in command output;
And the second is the target object or location, that's derived from the key target in command output.


-----------------------------------------------------------------------
JSON FORMAT

{
  "response": "<short, clear and comprehensive natural-language reply>",
  "audio response": "<short, clear and comprehensive natural-language reply suitable for TTS>",
  "content": [
    {
      "action_type": "<action type>",
      "target": "<object or location>"
    }
  ]
}

-----------------------------------------------------------------------
GENERAL RULES
* response, action_type and target should ALWAYS be strings, for example: "" or "apple". 
* When you don't understand the user's voice command, response MUST be "".
* response is NOT an answer to the user's question, but a more comprehensive command output based on the user's voice command, gesture labels, and gaze labels. 
* generated target must follow the requirements of the first target from user's voice command. For example, if the voice command asks for something to eat, 
the target should be a food item.
• When multiple items meet the requirement, choose the one that you think is the most possible.
* If certain words in the voice command seem to be typos, and in the meantime similar label(s) exist in gesture label or gaze label, 
then correct the voice command to match the gesture label or gaze label.
* When gaze label and gesture label don't contain any labels that resemble the first target in the voice command or alphabetically similar to the first 
target in the voice command, do NOT change the output of voice command!!!!
* The above rules must be followed strictly and simultaneously.PLEASE REFER TO THE EXAMPLES BELOW SERIOUSLY!
* You need to understand my true intention based on the given voice command, gesture label, and gaze label, and generate an accurate response.
* When my voice command is clear, don't change my voice command structure and wording, just correct the typos if necessary. 
* When my voice command is abstract or unclear, you can generate a more specific response based on the gesture label and gaze label.
* If no labels are available in gaze label or gesture label, response should repeat the voice command and extract the first target from the voice 
command and action_type(if available)

• Output **only** valid JSON — no markdown, no explanations, no commentary.

"""

# ---------------------------------------------------------------------
# EXAMPLES  (all user instructions in one string)
# ---------------------------------------------------------------------
intention_example = """
I have a speech command: 'What's your favorite recipe?', gesture label: '' and gaze label: ''.
I have a speech command: 'What's your favorite recipe?', gesture label: 'orange, salt' and gaze label: ''.
I have a speech command: 'What's your favorite recipe?', gesture label: 'remote' and gaze label: 'apple, juice'.

I have a speech command: 'Can you hand me the cop?', gesture label: '' and gaze label: ''.
I have a speech command: 'Can you hand me the cop?', gesture label: 'cup, pepper' and gaze label: ''.
I have a speech command: 'Can you hand me the salt?', gesture label: 'pepper' and gaze label: 'ketchup'.
I have a speech command: 'Can you hand me the salt?', gesture label: 'salt' and gaze label: 'pepper'.
I have a speech command: 'Can you hand me the cop?', gesture label: 'cup, ketchup' and gaze label: 'cup'.

I have a speech command: 'Please give me the peper bottle on the table.', gesture label: '' and gaze label: ''.
I have a speech command: 'Please give me the peper bottle on the table.', gesture label: '' and gaze label: 'pepper'.
I have a speech command: 'Please give me the peper on the table.', gesture label: 'salt, kechtup' and gaze label: 'ketchup'.
I have a speech command: 'Please give me the peper bottle on the table.', gesture label: 'salt' and gaze label: 'pepper'.
I have a speech command: 'Please give me the peper on the table.', gesture label: 'pepper' and gaze label: 'pepper'.

I have a speech command: 'Please add salt to the soup pot twice.', gesture label: '' and gaze label: ''.
I have a speech command: 'Please add salt to the soup pot twice.', gesture label: 'salt, pot' and gaze label: ''.
I have a speech command: 'Please add salt to the soup pot twice.', gesture label: 'pepper' and gaze label: 'ketchup'.
I have a speech command: 'Please add salt to the soup pot twice.', gesture label: 'ketchup' and gaze label: 'salt, pepper'.
I have a speech command: 'Please add salt to the soup pot twice.', gesture label: 'salt' and gaze label: 'salt, ketchup'.

I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture label: '' and gaze label: ''.
I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture label: '' and gaze label: 'spoon'.
I have a speech command: 'Use the spoon to stir the soup pot for 10 seconds.', gesture label: 'knife' and gaze label: 'fork, onion'.
I have a speech command: 'Use the spoon to stir the soup pot for 10 seconds.', gesture label: 'spoon' and gaze label: 'fork'.
I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture label: 'spoon, knife' and gaze label: 'spoon'.

I have a speech command: 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?', gesture label: '' and gaze label: ''.
I have a speech command: 'Can you help me use spool to stir the soup pot for 10 minutes and then give me the apple?', gesture label: 'spoon, fork' and gaze label: ''.
I have a speech command: 'Can you help me use spool to stir the soup pot for 10 minutes and then give me the apple?', gesture label: 'fork' and gaze label: 'knife, tomato'.
I have a speech command: 'Can you help me use spook to stir the soup pot for 10 minutes and then give me the apple?', gesture label: 'orange, fork' and gaze label: 'spoon'.
I have a speech command: 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?', gesture label: 'spoon' and gaze label: 'spoon, fork'.

I have a speech command: 'Can you tell me how to make burger? And please give me the salt bottle.', gesture label: '' and gaze label: ''.
I have a speech command: 'Can you tell me how to make burger? And please give me the sault bottle.', gesture label: '' and gaze label: 'salt, pepper'.
I have a speech command: 'Can you tell me how to make burger? And please give me the salt bottle.', gesture label: 'salt, orange' and gaze label: 'orange, apple, fork'.
I have a speech command: 'Can you tell me how to make burger? And please give me the paper bottle.', gesture label: 'pepper, ketchup' and gaze label: 'ketchup, curry, knife'.
I have a speech command: 'Can you tell me how to make burger? And please give me the carry bottle.', gesture label: 'curry, ketchup, mustard' and gaze label: 'curry'.

I have a speech command: 'I'm hungry. Give me something to eat.', gesture label: '' and gaze label: ''.
I have a speech command: 'I'm hungry. Give me something to eat.', gesture label: 'apple, cup' and gaze label: 'cake, cucumber, tomato'.
I have a speech command: 'I'm hungry. Give me some meat.', gesture label: 'cup, pork, knife, table' and gaze label: 'pork'.
I have a speech command: 'I'm hungry. Give me some meat.', gesture label: 'cup, knife, table' and gaze label: ''.
I have a speech command: 'I want to cut something. Please give me a tool', gesture label: 'knife, fork' and gaze label: 'apple'.
I have a speech command: 'I'm hungry. Give me something to eat.', gesture label: 'apple' and gaze label: 'apple'.

I have a speech command: 'I want to cut something. Please give me a tool', gesture label: 'knife, fork, scissors' and gaze label: 'apple, grinder'.

I have a speech command: 'Please tell me a story.', gesture label: '' and gaze label: ''.
""".strip()

# ---------------------------------------------------------------------
# ASSISTANT PROMPTS  (all JSON answers in one string, separated by blank lines)
# ---------------------------------------------------------------------
intention_assistant_prompt = '''
{
  "response": "What's your favorite recipe?",
  "audio response": "",
  "content": [
    {"action_type": None,
      "target": None}
  ]
}

{
  "response": "What's your favorite recipe?",
  "audio response": "",
  "content": [
    {"action_type": None,
      "target": None}
  ]
}

{
  "response": "What's your favorite recipe?",
  "audio response": "",
  "content": [
    {"action_type": None,
      "target": None}
  ]
}






{
  "response": "Can you hand me the cop?",
  "audio response": "Sorry I can't understand. Do you mean 'Can you hand me the cup?'",
  "content": [
    { "action_type": "hand", 
    "target": "cop"}
  ]
}

{
  "response": "Can you hand me the cup?",
  "audio response": "Sure! Here is the cup.",
  "content": [
    { "action_type": "hand", 
    "target": "cup"}
  ]
}

{
  "response": "Can you hand me the salt?",
  "audio response": "Sure! Giving you the salt.",
  "content": [
    { "action_type": "hand", 
    "target": "salt"}
  ]
}

{
  "response": "Can you hand me the salt?",
  "audio response": "Sure! Here is the salt.",
  "content": [
    { "action_type": "hand", 
    "target": "salt"}
  ]
}

{
  "response": "Can you hand me the cup?",
  "audio response": "Sure! Giving you the cup.",
  "content": [
    { "action_type": "hand", 
    "target": "cup"}
  ]
}






{
  "response": "Please give me the peper bottle on the table.",
  "audio response": "Sorry I can't understand. Do you mean 'Please give me the pepper bottle on the table?'",
  "content": [
    { "action_type": "give",
        "target": "pepper bottle"}
  ]
}

{
  "response": "Please give me the pepper bottle on the table.",
  "audio response": "Sure! Here is the pepper bottle on the table.",
  "content": [
    { "action_type": "give",
        "target": "pepper bottle"}
  ]
}

{
  "response": "Please give me the pepper bottle on the table.",
  "audio response": "Sure! Giving you the pepper bottle on the table.",
  "content": [
    { "action_type": "give",
        "target": "pepper bottle"}
  ]
}

{
  "response": "Please give me the pepper bottle on the table.",
  "audio response": "Sure! Here is the pepper bottle on the table.",
  "content": [
    { "action_type": "give",
        "target": "pepper bottle"}
  ]
}

{
  "response": "Please give me the pepper bottle on the table.",
  "audio response": "Sure! Here is the pepper bottle on the table.",
  "content": [
    { "action_type": "give",
        "target": "pepper bottle"}
  ]
}











{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt"}
  ]
}

{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt"}
  ]
}

{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt"}
  ]
}

{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt"}
  ]
}

{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt"}
  ]
}







{
  "response": "Use the spool to stir the soup pot for 10 seconds.",
  "audio response": "Sorry I can't understand. Do you mean 'Use the spoon to stir the soup pot for 10 seconds?'",
  "content": [
    { "action_type": "use",
      "target": "spool"}
  ]
}

{
  "response": "Use the spoon to stir the soup pot for 10 seconds.",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 seconds.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Use the spoon to stir the soup pot for 10 seconds.",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 seconds.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Use the spoon to stir the soup pot for 10 seconds.",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 seconds.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Use the spoon to stir the soup pot for 10 seconds.",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 seconds.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}










{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the apple.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the apple.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Can you help me use spool to stir the soup pot for 10 minutes and then give me the apple?",
  "audio response": "Sorry I can't understand. Do you mean 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?'",
  "content": [
    { "action_type": "use",
      "target": "spool"}
  ]
}

{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the apple.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the apple?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the apple.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}









{
  "response": "Can you tell me how to make burger? And please give me the salt bottle.",
  "audio response": "Sure! First, you'll need to gather the ingredients for a burger, which typically include ground beef, a bun, lettuce, tomato, and condiments. Then, you can cook the ground beef in a pan or on a grill until it's fully cooked. After that, you can assemble the burger by placing the cooked beef patty on the bun and adding your desired toppings. Finally, I'll give you the salt bottle to season your burger to your liking. Now giving you the salt bottle.",
  "content": [
    { "action_type": "give",
      "target": "salt bottle"}
  ]
}

{
  "response": "Can you tell me how to make burger? And please give me the salt bottle.",
  "audio response": "Sure! First, you'll need to gather the ingredients for a burger, which typically include ground beef, a bun, lettuce, tomato, and condiments. Then, you can cook the ground beef in a pan or on a grill until it's fully cooked. After that, you can assemble the burger by placing the cooked beef patty on the bun and adding your desired toppings. Finally, I'll give you the salt bottle to season your burger to your liking. Now giving you the salt bottle.",
  "content": [
    { "action_type": "give",
      "target": "salt bottle"}
  ]
}

{
  "response": "Can you tell me how to make burger? And please give me the salt bottle.",
  "audio response": "Sure! First, you'll need to gather the ingredients for a burger, which typically include ground beef, a bun, lettuce, tomato, and condiments. Then, you can cook the ground beef in a pan or on a grill until it's fully cooked. After that, you can assemble the burger by placing the cooked beef patty on the bun and adding your desired toppings. Finally, I'll give you the salt bottle to season your burger to your liking. Now giving you the salt bottle.",
  "content": [
    { "action_type": "give",
      "target": "salt bottle"}
  ]
}

{
  "response": "Can you tell me how to make burger? And please give me the pepper bottle.",
  "audio response": "Sure! First, you'll need to gather the ingredients for a burger, which typically include ground beef, a bun, lettuce, tomato, and condiments. Then, you can cook the ground beef in a pan or on a grill until it's fully cooked. After that, you can assemble the burger by placing the cooked beef patty on the bun and adding your desired toppings. Finally, I'll give you the pepper bottle to season your burger to your liking. Now giving you the pepper bottle.",
  "content": [
    { "action_type": "give",
      "target": "pepper bottle"}
  ]
}

{
  "response": "Can you tell me how to make burger? And please give me the curry bottle.",
  "audio response": "Sure! First, you'll need to gather the ingredients for a burger, which typically include ground beef, a bun, lettuce, tomato, and condiments. Then, you can cook the ground beef in a pan or on a grill until it's fully cooked. After that, you can assemble the burger by placing the cooked beef patty on the bun and adding your desired toppings. Finally, I'll give you the curry bottle to season your burger to your liking. Now giving you the curry bottle.",
  "content": [
    { "action_type": "give",
      "target": "curry bottle"}
  ]
}







{
  "response": "I'm hungry. Give me something to eat.",
  "audio response": "Sure! Finding something to eat.",
  "content": [
    { "action_type": "give",
      "target": "something"}
  ]
}

{
  "response": "I'm hungry. Give me the apple.",
  "audio response": "Sure! Here is the apple.",
  "content": [
    { "action_type": "give",
      "target": "apple"}
  ]
}

{
  "response": "I'm hungry. Give me the pork.",
  "audio response": "Sure! Here is the pork.",
  "content": [
    { "action_type": "give",
      "target": "pork"}
  ]
}

{
  "response": "I'm hungry. Give me some meat.",
  "audio response": "Sure! Giving you some meat.",
  "content": [
    { "action_type": "give",
      "target": "meat"}
  ]
}

{
  "response": "I want to cut something. Please give me the knife.",
  "audio response": "Sure! Here is the knife.",
  "content": [
    { "action_type": "give",
      "target": "knife"}
  ]
}

{
  "response": "I'm hungry. Give me the apple.",
  "audio response": "Sure! Here is the apple.",
  "content": [
    { "action_type": "give",
      "target": "apple"}
  ]
}

{  "response": "I want to cut something. Please give me the knife.",
  "audio response": "Sure! Here is the knife.",
  "content": [
    { "action_type": "give",
      "target": "knife"}
  ]
}

{
  "response": "Please tell me a story.",
  "audio response": "Sure! Once upon a time, there was a little girl named Lily who lived in a small village. She loved to explore the nearby forest and often went on adventures with her friends. One day, they discovered a hidden cave filled with sparkling gems and magical creatures. They spent hours playing and exploring the cave, making unforgettable memories. From that day on, Lily and her friends continued to have many more exciting adventures together. The end.",
  "content": [
    { "action_type": "",
      "target": ""}
  ]
}
'''.strip()
