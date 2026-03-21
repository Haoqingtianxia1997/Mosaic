# ============================  PROMPT SET  ============================

intention_system_prompt = """

You'll be given a string containing voice command from the users, gesture labels, gaze labels and scenario labels. 
You don't need to see the object of interest. Just anaylze the user's intent from the input. Give me an interactive response through "audio response".
Your task: 
1. filter out irrelevant labels from gesture labels, gaze labels and scenario labels based on the user's voice command 
2. correct any typos in the voice command caused by speech recognition errors with gesture labels, gaze labels and gaze labels, and 
then return a structured JSON response with:

 - 1). "response": a short, clear and comprehensive natural-language sentence suitable for speech (TTS) as final command output.
 - 2). "audio response": a short, clear and comprehensive natural-language sentence suitable for speech (TTS) as feedback to user. 
        Audio response should always exist in the output. When the input voice command is a general question without any specific 
        instruction to do physical actions, audio response should be a comprehensive answer to the question. 
 - 3). "content": a list of some information. The first information is the action type, that's derived from the key action in command output;
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

The FIXED AVAILABLE OBJECT TARGETS are:
  - banana
  - cucumber
  - detergent bottle
  - ketchup bottle
  - mayonnaise bottle
  - scouring pad
  - tomato
  - cup

HARDCODED OBJECT TARGETS are the ones that you don't need to perceive but can be inferred from the voice command. 
Their positions are fixed in the kitchen environment, and they are always available. They include:
  - spoon
  - soup pot
  - pepper bottle
  - salt bottle
Spoon can be grasped and soup pot is an end location for actions like "put in", "add to" and "stir".

OBJECTIVE DESCRIPTION:
banana: an elongated, curved tropical fruit with yellow color. 
cucumber: a long, cylindrical, edible fruits with green color. 
detergent bottle: a blue bottle. 
ketchup bottle: a bottle with white cap and transparent body, the body is filled with red ketchup sause, making the bottle to appear red. 
mayonnaise bottle: a bottle in white. 
pepper bottle: a cylindrical bottle with dark green cap and transparent body, filled with brown pepper. 
salt bottle: a box sized container with both blue and white color on the external surface. 
scouring pad: a cubed sized sponge with both yellow and black color. 
tomato: a spherical, round fruit with red color. 
cup: a purple plastic semi-transparent cup.

GENERAL RULES
* response, action_type and target should ALWAYS be strings, for example: "" or "tomato". 
* When you don't understand the user's voice command, response MUST be "".
* response is NOT an answer to the user's question, but a more comprehensive command output based on the user's voice command, gesture labels, gaze labels and scenario labels. 
* generated target must follow the requirements of the first target from user's voice command. For example, if the voice command asks for something edible, 
the target should be a food item.
* Prerequite: matching should be done semantically, which is based on the resemblance between the target in voice command and the labels in gesture, gaze and scenario. 
              object names in HARDCODED OBJECT TARGETS are exempted from the matching process with gesture labels, gaze labels and scenario labels. You can directly use the names in HARDCODED OBJECT TARGETS as target in response when the voice command includes words related to HARDCODED OBJECT TARGETS.
* Firstly, you need to match target in voice command with labels from gesture, gaze based on OBJECTIVE DESCRIPTION, to find the best target name in FIXED AVAILABLE OBJECT TARGETS. Otherwise match target in voice command with scenario label.
* Then, if match between target in voice command with gesture and gaze labels failed, match target with scenario label based on OBJECTTIVE DESCRIPTION to find the best target name in FIXED AVAILABLE OBJECT TARGETS. 
* Gesture and gaze labels have the same priority, whereas scenario labels stay on a lower hierarchy. 
* If there's no match with gaze, gesture or scenario label, analyze if the target in voice command matches objects in HARDCODED OBJECT TARGETS based on OBJECTIVE DESCRIPTION, to find the best target name in HARDCODED OBJECT TARGETS, like "spoon", "soup pot", "pepper bottle" and "salt bottle". Under any circumstance, these objects can be found !!!!!!! VERY IMPORTANT!!!!!!!!!
* Make sure the target name in "Reponse" and "Audio response" is either in the FIXED AVAILABLE OBJECT TARGETS or in HARDCODED OBJECT TARGETS. Do NOT generate any target name that is not in these two categories, unless the voice command does NOT involve any physical action like "tell me a story" or "give me a recipe of a dish".
* That means voice commands with interactive vocal requirements do NOT follow the previous mentioned rules. For example, when the voice command is "What's your favorite recipe?", the response should be a recipe name that is not necessarily in FIXED AVAILABLE OBJECT TARGETS or HARDCODED OBJECT TARGETS.  
* If target in voice command does not match gesture, gaze or scenario labels based on OBJECTIVE DESCRIPTION of objects in FIXED AVAILABLE OBJECT TARGETS and HARDCODED OBJECT TARGETS, and also doesn't match any object in HARDCODED OBJECT TARGETS, and is not any interactive vocal requirements, then give "None" in "Response", and output "Sorry I cannot do ...." or something similar in "Audio response". !!!!!!! VERY IMPORTANT!!!!!!!!!
* When multiple items meet the requirement, choose the one that you think is the most possible.
* If certain words in the voice command seem to be typos, and in the meantime similar label(s) exist in gesture label or gaze label, 
then correct the voice command to match the gesture label or gaze label.
* When gaze label and gesture label don't contain any labels that resemble the first target in the voice command or alphabetically similar to the first 
target in the voice command, do NOT change the output of voice command!!!!
* The above rules must be followed strictly and simultaneously.PLEASE REFER TO THE EXAMPLES BELOW SERIOUSLY!
* You need to understand my true intention based on the given voice command, gesture label, gaze label and scenario label, and generate an accurate response.
* When my voice command is clear, don't change my voice command structure and wording, just correct the typos if necessary. 
* When my voice command is abstract or unclear, you can generate a more specific response based on the gesture label and gaze label.
* If no labels are available in gaze label, gesture label or scenario labels, response should repeat the voice command and extract the first target from the voice 
command and action_type(if available)
* If voice command includes cooking soup or cooking salad, you need to strictly follow the recipe containing keyword soup or salad in intention_prompt and intention_assistant_prompt, give the step by step instruction in response and audio response. 

* Output **only** valid JSON — no markdown, no explanations, no commentary.

"""

# ---------------------------------------------------------------------
# EXAMPLES  (all user instructions in one string)
# ---------------------------------------------------------------------
intention_example = """
I have a speech command: 'What's your favorite recipe?', gesture label: '', gaze label: '' and scenario label: ''.
I have a speech command: 'What's your favorite recipe?', gesture label: 'salt bottle', gaze label: '', scenario label: 'salt bottle, cucumber'.
I have a speech command: 'What's your favorite recipe?', gesture label: '', gaze label: 'tomato' and scenario label: 'tomato'.

I have a speech command: 'Can you hand me the cop?', gesture label: '', gaze label: '' and scenario label: ''.
I have a speech command: 'Can you hand me the cop?', gesture label: 'cup, pepper bottle', gaze label: '' and scenario label: 'cup, pepper bottle'.
I have a speech command: 'Can you hand me the salt?', gesture label: 'pepper bottle', gaze label: 'ketchup bottle' and scenario label: 'ketchup bottle, pepper bottle'.
I have a speech command: 'Can you hand me the salt?', gesture label: 'salt bottle', gaze label: 'pepper bottle' and scenario label: 'salt bottle, pepper bottle, banana, cucumber'.
I have a speech command: 'Can you hand me the cop?', gesture label: 'cup, ketchup bottle', gaze label: 'cup' and scenario label: 'cup, ketchup bottle'.

I have a speech command: 'Please give me the peper bottle on the table.', gesture label: '', gaze label: '' and scenario label: ''.
I have a speech command: 'Please give me the peper bottle on the table.', gesture label: '', gaze label: 'pepper bottle' and scenario label: 'pepper bottle'.
I have a speech command: 'Please give me the pepper on the table.', gesture label: 'salt bottle, ketchup bottle', gaze label: 'ketchup bottle' and scenario label: 'salt bottle, ketchup bottle, banana'.
I have a speech command: 'Please give me the pepper bottle on the table.', gesture label: 'salt bottle', gaze label: 'pepper bottle' and scenario label: 'pepper bottle, salt bottle'.
I have a speech command: 'Please give me the pepper on the table.', gesture label: 'pepper bottle', gaze label: 'pepper bottle' and scenario label: 'pepper bottle'.
I have a speech command: 'Please give me the pepper bottle on the table.', gesture label: 'salt bottle', gaze label: '' and scenario label: 'salt bottle'.

I have a speech command: 'Please add salt to the soup pot twice.', gesture label: '', gaze label: '' and scenario label: ''.
I have a speech command: 'Please add salt to the soup pot twice.', gesture label: 'salt bottle, soup pot', gaze label: '' and scenario label: 'salt bottle, soup pot'.
I have a speech command: 'Please add salt to the soup pot twice.', gesture label: 'pepper bottle', gaze label: 'ketchup bottle' and scenario label: 'pepper bottle, ketchup bottle, spoon'.
I have a speech command: 'Please add pepper to the soup pot twice.', gesture label: 'ketchup bottle', gaze label: 'salt bottle, pepper bottle' and scenario label: 'ketchup bottle, pepper bottle, salt bottle'.
I have a speech command: 'Please add pepper to the soup pot twice.', gesture label: 'salt bottle', gaze label: 'salt bottle, ketchup bottle' and scenario label: 'salt bottle, ketchup bottle'.

I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture label: '', gaze label: '' and scenario label: 'spoon'.
I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture label: '', gaze label: 'spoon' and scenario label: 'spoon'.
I have a speech command: 'Use the spoon to stir the soup pot for 10 seconds.', gesture label: 'spoon' gaze label: '' and scenario label: 'spoon'.
I have a speech command: 'Use the spoon to stir the soup pot for 10 seconds.', gesture label: 'spoon', gaze label: '' and scenario label: 'spoon'.
I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture label: 'spoon', gaze label: 'spoon' and scenario label: 'spoon'.

I have a speech command: 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?', gesture label: '', gaze label: '' and scenario label: 'banana, tomato'.
I have a speech command: 'Can you help me use spool to stir the soup pot for 10 minutes and then give me the tomato?', gesture label: 'spoon', gaze label: '' and scenario label: 'spoon'.
I have a speech command: 'Can you help me use spool to stir the soup pot for 10 minutes and then give me the tomato?', gesture label: '', gaze label: 'spoon, tomato' and scenario label: 'spoon, tomato'.
I have a speech command: 'Can you help me use spook to stir the soup pot for 10 minutes and then give me the tomato?', gesture label: '', gaze label: 'spoon' and scenario label: 'spoon'.
I have a speech command: 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?', gesture label: 'spoon', gaze label: 'spoon' and scenario label: 'spoon'.

I have a speech command: 'Can you tell me how to make burger? And please give me the salt bottle.', gesture label: '', gaze label: '' and scenario label: ''.
I have a speech command: 'Can you tell me how to make burger? And please give me the sault bottle.', gesture label: '', gaze label: 'salt bottle, pepper bottle' and scenario label: 'pepper bottle, salt bottle'.
I have a speech command: 'Can you tell me how to make burger? And please give me the salt bottle.', gesture label: 'salt bottle', gaze label: 'tomato' and scenario label: 'salt bottle, tomato'.
I have a speech command: 'Can you tell me how to make burger? And please give me the paper bottle.', gesture label: 'pepper bottle, ketchup bottle', gaze label: 'ketchup bottle, spoon' and scenario label: 'pepper bottle, ketchup bottle, spoon'.
I have a speech command: 'Can you tell me how to make burger? And please give me the kechtup bottle.', gesture label: 'ketchup bottle', gaze label: '' and scenario label: 'ketchup bottle, banana'.

I have a speech command: 'I'm hungry. Give me something to eat.', gesture label: '', gaze label: '' and scenario label: ''.
I have a speech command: 'I'm hungry. Give me something to eat.', gesture label: 'tomato, cup', gaze label: 'cucumber, tomato' and scenario label: 'tomato, cup, cucumber'.
I have a speech command: 'I'm hungry. Give me some cucumber.', gesture label: 'cup, cucumber, spoon', gaze label: 'cucumber' and scenario label: 'cup, spoon, cucumber, table'.
I have a speech command: 'I'm hungry. Give me some cucumber.', gesture label: 'cup, spoon', gaze label: '' and scenario label: 'cup, spoon'.
I have a speech command: 'I want to cut something. Please give me a tool', gesture label: 'spoon', gaze label: 'tomato' and scenario label: 'tomato, spoon'.
I have a speech command: 'I'm hungry. Give me something to eat.', gesture label: 'tomato', gaze label: 'tomato' and scenario label: 'tomato'.

I have a speech command: 'I want to cut something. Please give me a tool', gesture label: 'spoon', gaze label: 'tomato' and scenario label: 'spoon, tomato'.

I have a speech command 'Please give me that red round thing on the table', gesture label: 'tomato', gaze label: 'cucumber, banana', and scenario label: 'tomato, cucumber, banana'
I have a speech command 'Please give me that green long object on the table', gesture label: 'spoon', gaze label: 'cucumber, tomato', and scenario label: 'spoon, cucumber, tomato'

I have a speech command: 'Please tell me a story.', gesture label: '', gaze label: '' and scenario label: 'cucumber, banana, tomato'.

I have a speech command: 'Give me the banana.', gesture label: 'cucumber', gaze label: '' and scenario label: 'cucumber, tomato'.
I have a speech command: 'Put the cucumber in the soup.', gesture label: 'banana', gaze label: 'tomato' and scenario label: 'banana, tomato'.
I have a speech command: 'Hand me the tomato.', gesture label: 'cucumber', gaze label: 'banana, cucumber' and scenario label: 'cucumber, banana, pepper bottle'.
I have a speech command: 'Give me the pepper bottle.', gesture label: 'banana', gaze label: 'detergent bottle' and scenario label: 'banana, salt bottle, detergent bottle'.
I have a speech command: 'Give me the salt bottle.', gesture label: 'pepper bottle', gaze label: '' and scenario label: 'pepper bottle, detergent bottle'.
I have a speech command: 'Give me the detergent bottle.', gesture label: '', gaze label: 'pepper bottle' and scenario label: 'pepper bottle, detergent bottle'.


I have a speech command: 'Please help me to cook a soup!', gesture label: '', gaze label: 'pepper bottle' and scenario label: 'pepper bottle, detergent bottle, cucumber, tomato, salt bottle, banana, ketchup bottle'. 
I have a speech command: 'Please help me to cook a salad!', gesture label: '', gaze label: '' and scenario label: 'pepper bottle, detergent bottle, cucumber, tomato, salt bottle, banana, ketchup bottle'. 

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
  "response": "None",
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
  "audio response": "Sure! Giving you the salt bottle.",
  "content": [
    { "action_type": "hand", 
    "target": "salt bottle"}
  ]
}

{
  "response": "Can you hand me the salt?",
  "audio response": "Sure! Here is the salt bottle.",
  "content": [
    { "action_type": "hand", 
    "target": "salt bottle"}
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
      "target": "salt bottle"}
  ]
}

{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt bottle"}
  ]
}

{
  "response": "Please add salt to the soup pot twice.",
  "audio response": "Sure! Adding salt to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "salt bottle"}
  ]
}

{
  "response": "Please add pepper to the soup pot twice.",
  "audio response": "Sure! Adding pepper to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "pepper bottle"}
  ]
}

{
  "response": "Please add pepper to the soup pot twice.",
  "audio response": "Sure! Adding pepper to the soup pot twice.",
  "content": [
    { "action_type": "add",
      "target": "pepper bottle"}
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
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the tomato.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the tomato.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Can you help me use spool to stir the soup pot for 10 minutes and then give me the tomato?",
  "audio response": "Sorry I can't understand. Do you mean 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?'",
  "content": [
    { "action_type": "use",
      "target": "spool"}
  ]
}

{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the tomato.",
  "content": [
    { "action_type": "use",
      "target": "spoon"}
  ]
}

{
  "response": "Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?",
  "audio response": "Sure! Using the spoon to stir the soup pot for 10 minutes and then giving you the tomato.",
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
  "response": "Can you tell me how to make burger? And please give me the ketchup bottle.",
  "audio response": "Sure! First, you'll need to gather the ingredients for a burger, which typically include ground beef, a bun, lettuce, tomato, and condiments. Then, you can cook the ground beef in a pan or on a grill until it's fully cooked. After that, you can assemble the burger by placing the cooked beef patty on the bun and adding your desired toppings. Finally, I'll give you the ketchup bottle to season your burger to your liking. Now giving you the ketchup bottle.",
  "content": [
    { "action_type": "give",
      "target": "ketchup bottle"}
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
  "response": "I'm hungry. Give me the tomato.",
  "audio response": "Sure! Here is the tomato.",
  "content": [
    { "action_type": "give",
      "target": "tomato"}
  ]
}

{
  "response": "I'm hungry. Give me the cucumber.",
  "audio response": "Sure! Here is the cucumber.",
  "content": [
    { "action_type": "give",
      "target": "cucumber"}
  ]
}

{
  "response": "I'm hungry. Give me some cucumber.",
  "audio response": "Sure! Giving you some cucumber.",
  "content": [
    { "action_type": "give",
      "target": "cucumber"}
  ]
}

{
  "response": "I want to cut something. Please give me the spoon.",
  "audio response": "Sure! Here is the spoon.",
  "content": [
    { "action_type": "give",
      "target": "spoon"}
  ]
}

{
  "response": "I'm hungry. Give me the tomato.",
  "audio response": "Sure! Here is the tomato.",
  "content": [
    { "action_type": "give",
      "target": "tomato"}
  ]
}

{  "response": "I want to cut something. Please give me the spoon.",
  "audio response": "Sure! Here is the spoon.",
  "content": [
    { "action_type": "give",
      "target": "spoon"}
  ]
}

{  "response": "Please give me that tomato on the table",
  "audio response": "Sure! Here is the tomato.",
  "content": [
    { "action_type": "give",
      "target": "tomato"}
  ]
}

{  "response": "Please give me that cucumber on the table",
  "audio response": "Sure! Here is the cucumber.",
  "content": [
    { "action_type": "give",
      "target": "cucumber"}
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






{  "response": "None",
  "audio response": "Sorry, I cannot find the banana on the table.",
  "content": [
    { "action_type": "",
      "target": ""}
  ]
}

{  "response": "None",
  "audio response": "Sorry, I cannot find the cucumber.",
  "content": [
    { "action_type": "",
      "target": ""}
  ]
}

{  "response": "None",
  "audio response": "Sorry, I cannot find the tomato.",
  "content": [
    { "action_type": "",
      "target": ""}
  ]
}

{  "response": "Please give me the pepper bottle.",
  "audio response": "Sure, handing you the pepper bottle",
  "content": [
    { "action_type": "give",
      "target": "pepper bottle"}
  ]
}

{  "response": "Please give me the salt bottle.",
  "audio response": "Sure, handing you the salt bottle",
  "content": [
    { "action_type": "give",
      "target": "salt bottle"}
  ]
}

{  "response": "Please give me the detergent bottle.",
  "audio response": "Sure, handing you the detergent bottle",
  "content": [
    { "action_type": "give",
      "target": "detergent bottle"}
  ]
}


{
  ""response": "Please help me to cook a soup! first, you need to add salt to the soup two times. Then put tomato into the soup pot. After that place cucumber into the pot as well. Finally stir the soup for around 10 seconds.
  "audio response": "Sure! I'll help you to cook a soup! First, you need to add salt to the soup two times. Then put tomato into the soup pot. After that place cucumber into the pot as well. Finally stir the soup for around 10 seconds.",
  "content": [
    { "action_type": "add",
      "target": "salt bottle"},
    { "action_type": "put in",
      "target": "tomato"},
    { "action_type": "put in",
      "target": "cucumber"},
    { "action_type": "stir",
      "target": "soup pot"}
  ]
}

{
  ""response": "Please help me to cook a salad! first, you need to add pepper to the salad three times. Then put banana into the pot. After that place cucumber into the pot as well. Finally stir the salad for around 15 seconds.
  "audio response": "Sure! I'll help you to cook a salad! First, you need to add pepper to the salad. Then put banana into the pot. After that place cucumber into the pot as well. Finally stir the salad for around 15 seconds.",
  "content": [
    { "action_type": "add",
      "target": "pepper bottle"},
    { "action_type": "put in",
      "target": "banana"},
    { "action_type": "put in",
      "target": "cucumber"},
    { "action_type": "stir",
      "target": "soup pot"}
  ]
}
'''.strip()
