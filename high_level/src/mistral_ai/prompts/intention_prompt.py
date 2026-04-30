# ============================  PROMPT SET  ============================

intention_system_prompt = """

You'll be given a string containing voice command from the users, gesture info, gaze info and scenario labels. 
You don't need to see the object of interest. Just anaylze the user's intent from the input. Give me an interactive response through "audio response".
Your task: 
1. filter out irrelevant info from gesture info, gaze info and scenario labels based on the user's voice command 
2. correct any typos in the voice command caused by speech recognition errors with gesture info, gaze info and gaze info, and 
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
  - sponge
  - tomato
  - cup
  - juice

HARDCODED OBJECT TARGETS are the ones that you don't need to perceive but can be inferred from the voice command. 
Their positions are fixed in the kitchen environment, and they are always available. They include:
  - spoon
  - soup pot
  - pepper bottle
  - salt bottle
  - dessert plate
Spoon can be grasped, soup pot and dessert plate are end locations for actions like "put in", "add to" and "stir".

OBJECTIVE DESCRIPTION:
banana: an elongated, curved tropical fruit with yellow color. 
cucumber: a long, cylindrical, edible fruits with green color. 
detergent bottle: a blue bottle. 
ketchup bottle: a bottle with white cap and transparent body, the body is filled with red ketchup sause, making the bottle to appear red. 
mayonnaise bottle: a bottle in white. 
pepper bottle: a cylindrical bottle with dark green cap and transparent body, filled with brown pepper. 
salt bottle: a box sized container with both blue and white color on the external surface. 
sponge: a cubed sized sponge with both yellow and black color. 
tomato: a spherical, round fruit with red color. 
cup: a purple plastic semi-transparent cup.
juice: a cubic bottle with pink and yellow color, filled with juice.

GENERAL RULES
* response, action_type and target should ALWAYS be strings, for example: "" or "tomato". 
* When you don't understand the user's voice command, response MUST be "".
* response is NOT an answer to the user's question, but a more comprehensive command output based on the user's voice command, gesture info, gaze info and scenario labels. 
* The input from gesture info and gaze info contains object labels and scores in the following format: {"label": "object_name", "score": <score_value>}.
* Label and score pairs have already been sorted in a descending order based on the score value, which means the first label in gesture info or gaze info is the one with the highest score and most likely to be the target object.
* Generated target must follow the requirements of the first target from user's voice command. For example, if the voice command asks for something edible, 
the target should be a food item.
* Prerequite: matching should be done semantically, which is based on the resemblance between the target in voice command and the labels in gesture, gaze and scenario. 
              object names in HARDCODED OBJECT TARGETS are exempted from the matching process with gesture info, gaze info and scenario labels. You can directly use the names in HARDCODED OBJECT TARGETS as target in response when the voice command includes words related to HARDCODED OBJECT TARGETS.
* Firstly, you need to match target in voice command with labels from gesture, gaze based on OBJECTIVE DESCRIPTION, to find the best target name in FIXED AVAILABLE OBJECT TARGETS. Otherwise match target in voice command with scenario labels.
* Then, if match between target in voice command with gesture and gaze info failed, match target with scenario labels based on OBJECTTIVE DESCRIPTION to find the best target name in FIXED AVAILABLE OBJECT TARGETS. 
* Gesture and gaze info have the same priority, whereas scenario labels stay on a lower hierarchy. 
* If there's no match with gaze, gesture or scenario labels, analyze if the target in voice command matches objects in HARDCODED OBJECT TARGETS based on OBJECTIVE DESCRIPTION, to find the best target name in HARDCODED OBJECT TARGETS, like "spoon", "soup pot", "dessert plate", "pepper bottle" and "salt bottle". Under any circumstance, these objects can be found !!!!!!! VERY IMPORTANT!!!!!!!!!
* Make sure the target name in "Reponse" and "Audio response" is either in the FIXED AVAILABLE OBJECT TARGETS or in HARDCODED OBJECT TARGETS. Do NOT generate any target name that is not in these two categories, unless the voice command does NOT involve any physical action like "tell me a story" or "give me a recipe of a dish".
* That means voice commands with interactive vocal requirements do NOT follow the previous mentioned rules. For example, when the voice command is "What's your favorite recipe?", the response should be a recipe name that is not necessarily in FIXED AVAILABLE OBJECT TARGETS or HARDCODED OBJECT TARGETS.  
* If target in voice command does not match gesture, gaze or scenario labels based on OBJECTIVE DESCRIPTION of objects in FIXED AVAILABLE OBJECT TARGETS and HARDCODED OBJECT TARGETS, and also doesn't match any object in HARDCODED OBJECT TARGETS, and is not any interactive vocal requirements, then give "None" in "Response", and output "Sorry I cannot do ...." or something similar in "Audio response". !!!!!!! VERY IMPORTANT!!!!!!!!!
* When multiple items meet the requirement, choose the one that you think is the most possible.
* If certain words in the voice command seem to be typos, and in the meantime similar label(s) exist in gesture info or gaze info, 
then correct the voice command to match the gesture info or gaze info.
* When gaze info and gesture info don't contain any labels that resemble the first target in the voice command or alphabetically similar to the first 
target in the voice command, do NOT change the output of voice command!!!!
* The above rules must be followed strictly and simultaneously.PLEASE REFER TO THE EXAMPLES BELOW SERIOUSLY!
* You need to understand my true intention based on the given voice command, gesture info, gaze info and scenario labels, and generate an accurate response.
* When my voice command is clear, don't change my voice command structure and wording, just correct the typos if necessary. 
* When my voice command is abstract or unclear, you can generate a more specific response based on the gesture info and gaze info.
* If no labels are available in gaze info, gesture info or scenario labels, response should repeat the voice command and extract the first target from the voice 
command and action_type(if available)
* If voice command includes cooking soup or cooking salad, you need to strictly follow the recipe containing keyword soup or salad in intention_prompt and intention_assistant_prompt, give the step by step instruction in response and audio response. 

* Output **only** valid JSON — no markdown, no explanations, no commentary.

"""

# ---------------------------------------------------------------------
# EXAMPLES  (all user instructions in one string)
# ---------------------------------------------------------------------
intention_example = """
I have a speech command: 'What's your favorite recipe?', gesture info: '', gaze info: '' and scenario labels: ''.
I have a speech command: 'What's your favorite recipe?', gesture info: '[{"label": "salt bottle", "score": 0.87}]', gaze info: '[]', scenario labels: 'salt bottle, cucumber'.
I have a speech command: 'What's your favorite recipe?', gesture info: '', gaze info: '[{"label": "tomato", "score": 0.92}]' and scenario labels: 'tomato'.

I have a speech command: 'Can you hand me the cop?', gesture info: '', gaze info: '' and scenario labels: ''.
I have a speech command: 'Can you hand me the cop?', gesture info: '[{"label": "cup", "score": 0.89}, {"label": "pepper bottle", "score": 0.55}]', gaze info: '[]', scenario labels: 'cup, pepper bottle'.
I have a speech command: 'Can you hand me the salt?', gesture info: '[{"label": "pepper bottle", "score": 0.87}]', gaze info: '[{"label": "ketchup bottle", "score": 0.84}]' and scenario labels: 'ketchup bottle, pepper bottle'.
I have a speech command: 'Can you hand me the salt?', gesture info: '[{"label": "salt bottle", "score": 0.87}]', gaze info: '[{"label": "pepper bottle", "score": 0.95}]' and scenario labels: 'salt bottle, pepper bottle, banana, cucumber'.
I have a speech command: 'Can you hand me the cop?', gesture info: '[{"label": "cup", "score": 0.91}, {"label": "ketchup bottle", "score": 0.50}]', gaze info: '[{"label": "cup", "score": 0.88}]' and scenario labels: 'cup, ketchup bottle'.

I have a speech command: 'Please give me the peper bottle on the table.', gesture info: '', gaze info: '' and scenario labels: ''.
I have a speech command: 'Please give me the peper bottle on the table.', gesture info: '', gaze info: '[{"label": "pepper bottle", "score": 0.92}]' and scenario labels: 'pepper bottle'.
I have a speech command: 'Please give me the pepper on the table.', gesture info: '[{"label": "salt bottle", "score": 0.99}, {"label": "ketchup bottle", "score": 0.70}]', gaze info: '[{"label": "ketchup bottle", "score": 0.84}]' and scenario labels: 'salt bottle, ketchup bottle, banana'.
I have a speech command: 'Please give me the pepper bottle on the table.', gesture info: '[{"label": "salt bottle", "score": 0.87}]', gaze info: '[{"label": "pepper bottle", "score": 0.92}]' and scenario labels: 'pepper bottle, salt bottle'.
I have a speech command: 'Please give me the pepper on the table.', gesture info: '[{"label": "pepper bottle", "score": 0.92}]', gaze info: '[{"label": "pepper bottle", "score": 0.92}]' and scenario labels: 'pepper bottle'.
I have a speech command: 'Please give me the pepper bottle on the table.', gesture info: '[{"label": "salt bottle", "score": 0.87}]', gaze info: '' and scenario labels: 'salt bottle'.

I have a speech command: 'Please add salt to the soup pot twice.', gesture info: '', gaze info: '' and scenario labels: ''.
I have a speech command: 'Please add salt to the soup pot twice.', gesture info: '[{"label": "salt bottle", "score": 0.83}, {"label": "soup pot", "score": 0.62}]', gaze info: '' and scenario labels: 'salt bottle, soup pot'.
I have a speech command: 'Please add salt to the soup pot twice.', gesture info: '[{"label": "pepper bottle", "score": 0.97}]', gaze info: '[{"label": "ketchup bottle", "score": 0.94}]' and scenario labels: 'pepper bottle, ketchup bottle, spoon'.
I have a speech command: 'Please add pepper to the soup pot twice.', gesture info: '[{"label": "ketchup bottle", "score": 0.90}]', gaze info: '[{"label": "salt bottle", "score": 0.93}, {"label": "pepper bottle", "score": 0.75}]' and scenario labels: 'ketchup bottle, pepper bottle, salt bottle'.
I have a speech command: 'Please add pepper to the soup pot twice.', gesture info: '[{"label": "salt bottle", "score": 0.87}]', gaze info: '[{"label": "salt bottle", "score": 0.82}, {"label": "ketchup bottle", "score": 0.64}]' and scenario labels: 'salt bottle, ketchup bottle'.

I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture info: '', gaze info: '' and scenario labels: 'spoon'.
I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture info: '', gaze info: '[{"label": "spoon", "score": 0.96}]' and scenario labels: 'spoon'.
I have a speech command: 'Use the spoon to stir the soup pot for 10 seconds.', gesture info: '[{"label": "spoon", "score": 0.91}]', gaze info: '' and scenario labels: 'spoon'.
I have a speech command: 'Use the spoon to stir the soup pot for 10 seconds.', gesture info: '[{"label": "spoon", "score": 0.87}]', gaze info: '' and scenario labels: 'spoon'.
I have a speech command: 'Use the spool to stir the soup pot for 10 seconds.', gesture info: '[{"label": "spoon", "score": 0.85}]', gaze info: '[{"label": "spoon", "score": 0.90}]' and scenario labels: 'spoon'.

I have a speech command: 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?', gesture info: '', gaze info: '' and scenario labels: 'banana, tomato'.
I have a speech command: 'Can you help me use spool to stir the soup pot for 10 minutes and then give me the tomato?', gesture info: '[{"label": "spoon", "score": 0.85}]', gaze info: '' and scenario labels: 'spoon'.
I have a speech command: 'Can you help me use spool to stir the soup pot for 10 minutes and then give me the tomato?', gesture info: '', gaze info: '[{"label": "spoon", "score": 0.95}, {"label": "tomato", "score": 0.78}]' and scenario labels: 'spoon, tomato'.
I have a speech command: 'Can you help me use spook to stir the soup pot for 10 minutes and then give me the tomato?', gesture info: '', gaze info: '[{"label": "spoon", "score": 0.87}]' and scenario labels: 'spoon'.
I have a speech command: 'Can you help me use spoon to stir the soup pot for 10 minutes and then give me the tomato?', gesture info: '[{"label": "spoon", "score": 0.80}]', gaze info: '[{"label": "spoon", "score": 0.90}]' and scenario labels: 'spoon'.

I have a speech command: 'Can you tell me how to make burger? And please give me the salt bottle.', gesture info: '', gaze info: '' and scenario labels: ''.
I have a speech command: 'Can you tell me how to make burger? And please give me the sault bottle.', gesture info: '', gaze info: '[{"label": "salt bottle", "score": 0.88}, {"label": "pepper bottle", "score": 0.60}]' and scenario labels: 'pepper bottle, salt bottle'.
I have a speech command: 'Can you tell me how to make burger? And please give me the salt bottle.', gesture info: '[{"label": "salt bottle", "score": 0.95}]', gaze info: '[{"label": "tomato", "score": 0.93}]' and scenario labels: 'salt bottle, tomato'.
I have a speech command: 'Can you tell me how to make burger? And please give me the paper bottle.', gesture info: '[{"label": "pepper bottle", "score": 0.98}, {"label": "ketchup bottle", "score": 0.65}]', gaze info: '[{"label": "ketchup bottle", "score": 0.80}, {"label": "spoon", "score": 0.54}]' and scenario labels: 'pepper bottle, ketchup bottle, spoon'.
I have a speech command: 'Can you tell me how to make burger? And please give me the kechtup bottle.', gesture info: '[{"label": "ketchup bottle", "score": 0.85}]', gaze info: '' and scenario labels: 'ketchup bottle, banana'.

I have a speech command: 'I'm hungry. Give me something to eat.', gesture info: '', gaze info: '' and scenario labels: ''.
I have a speech command: 'I'm hungry. Give me something to eat.', gesture info: '[{"label": "tomato", "score": 0.91}, {"label": "cup", "score": 0.75}]', gaze info: '[{"label": "cucumber", "score": 0.85}, {"label": "tomato", "score": 0.58}]' and scenario labels: 'tomato, cup, cucumber'.
I have a speech command: 'I'm hungry. Give me some cucumber.', gesture info: '[{"label": "cup", "score": 0.90}, {"label": "cucumber", "score": 0.71}, {"label": "spoon", "score": 0.48}]', gaze info: '[{"label": "cucumber", "score": 0.85}]' and scenario labels: 'cup, spoon, cucumber, table'.
I have a speech command: 'I'm hungry. Give me some cucumber.', gesture info: '[{"label": "cup", "score": 0.94}, {"label": "spoon", "score": 0.69}]', gaze info: '' and scenario labels: 'cup, spoon'.
I have a speech command: 'I want to cut something. Please give me a tool', gesture info: '[{"label": "spoon", "score": 0.85}]', gaze info: '[{"label": "tomato", "score": 0.90}]' and scenario labels: 'tomato, spoon'.
I have a speech command: 'I'm hungry. Give me something to eat.', gesture info: '[{"label": "tomato", "score": 0.91}]', gaze info: '[{"label": "tomato", "score": 0.93}]' and scenario labels: 'tomato'.

I have a speech command: 'I want to cut something. Please give me a tool', gesture info: '[{"label": "spoon", "score": 0.85}]', gaze info: '[{"label": "tomato", "score": 0.90}]' and scenario labels: 'spoon, tomato'.

I have a speech command 'Please give me that red round thing on the table', gesture info: '[{"label": "tomato", "score": 0.92}]', gaze info: '[{"label": "cucumber", "score": 0.88}, {"label": "banana", "score": 0.65}]', and scenario labels: 'tomato, cucumber, banana'
I have a speech command 'Please give me that green long object on the table', gesture info: '[{"label": "spoon", "score": 0.89}]', gaze info: '[{"label": "cucumber", "score": 0.91}, {"label": "tomato", "score": 0.72}]', and scenario labels: 'spoon, cucumber, tomato'

I have a speech command: 'Please tell me a story.', gesture info: '', gaze info: '' and scenario labels: 'cucumber, banana, tomato'.

I have a speech command: 'Give me the banana.', gesture info: '[{"label": "cucumber", "score": 0.85}]', gaze info: '' and scenario labels: 'cucumber, tomato'.
I have a speech command: 'Put the cucumber in the soup.', gesture info: '[{"label": "banana", "score": 0.88}]', gaze info: '[{"label": "tomato", "score": 0.90}]' and scenario labels: 'banana, tomato'.
I have a speech command: 'Hand me the tomato.', gesture info: '[{"label": "cucumber", "score": 0.87}]', gaze info: '[{"label": "banana", "score": 0.93}, {"label": "cucumber", "score": 0.71}]' and scenario labels: 'cucumber, banana, pepper bottle'.
I have a speech command: 'Give me the pepper bottle.', gesture info: '[{"label": "banana", "score": 0.82}]', gaze info: '[{"label": "detergent bottle", "score": 0.89}]' and scenario labels: 'banana, salt bottle, detergent bottle'.
I have a speech command: 'Give me the salt bottle.', gesture info: '[{"label": "pepper bottle", "score": 0.91}]', gaze info: '' and scenario labels: 'pepper bottle, detergent bottle'.
I have a speech command: 'Give me the detergent bottle.', gesture info: '', gaze info: '[{"label": "pepper bottle", "score": 0.86}]' and scenario labels: 'pepper bottle, detergent bottle'.


I have a speech command: 'Please help me to cook a soup!', gesture info: '', gaze info: '[{"label": "pepper bottle", "score": 0.78}]' and scenario labels: 'pepper bottle, detergent bottle, cucumber, tomato, salt bottle, banana, ketchup bottle'.
I have a speech command: 'Please help me to cook a salad!', gesture info: '', gaze info: '' and scenario labels: 'pepper bottle, detergent bottle, cucumber, tomato, salt bottle, banana, ketchup bottle'.

I have a speech command: 'Please put the banana into the dessert plate', gesture info: '[{"label": "banana", "score": 0.94}]', gaze info: '' and scenario labels: 'banana'.
I have a speech command: 'Please put the tomato into the dessert plate', gesture info: '', gaze info: '[{"label": "tomato", "score": 0.91}]' and scenario labels: 'tomato'.
I have a speech command: 'Please put the cucumber into the dessert plate', gesture info: '[{"label": "cucumber", "score": 0.88}]', gaze info: '' and scenario labels: 'cucumber'.

I have a speech command: 'Please give me the juice on the table.', gesture info: '[{"label": "juice", "score": 0.90}]', gaze info: '' and scenario labels: 'juice'.
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

{
  "response": "Please put the banana into the dessert plate",
  "audio response": "Sure! Putting the banana into the dessert plate.",
  "content": [
    { "action_type": "put in",
      "target": "banana"}
  ]
}

{
  "response": "Please put the tomato into the dessert plate",
  "audio response": "Sure! Putting the tomato into the dessert plate.",
  "content": [
    { "action_type": "put in",
      "target": "tomato"}
  ]
}

{
  "response": "Please put the cucumber into the dessert plate",
  "audio response": "Sure! Putting the cucumber into the dessert plate.",
  "content": [
    { "action_type": "put in",
      "target": "cucumber"}
  ]
}

{
  "response": "Please give me the juice on the table.",
  "audio response": "Sure! Here is the juice on the table.",
  "content": [
    { "action_type": "give",
      "target": "juice"}
  ]
}
'''.strip()
