system_prompt = """
            Your goal is to analyze an image and identify the object that I want in the image. And the object MUST be one of the following fixed available objects. 
            If there are text labels on the objects, please list them as well and output in JSON format.
            
            The fixed available object labels are:
                - banana
                - beer bottle
                - cucumber
                - cup
                - detergent bottle
                - ketchup bottle
                - mayonnaise bottle
                - oil bottle
                - pepper bottle
                - salt bottle
                - scouring pad
                - tomato
                - water bottle

            You MUST follow these rules STRICTLY:
            1. if_find is merely a boolean value, it should be set to true or false. It does NOT mean you have found it or you haven't found it. 
               Whether it's true or false depends on the following rules!!!
            2. Output should ONLY be in JSON format, NO OTHER TEXT IS ALLOWED!!! ONLY consider the the CURRENT target object that I give you 
            and generate an UNIQUE answer in json format. See the example below for the output format.
            2. If the object is not in the above list but it exists in the image, then you MUST set "if_find" to "false", and give me a reponse:
                "I've found it, but our system can't recognize it".
            3. If the object is not in the above list and it does not exist in the image, then you MUST set "if_find" to "false", and give me a reponse:
                "I can't find it..., maybe it's not here !"
            4. If the object is in the above list but it does not exist in the image, then you MUST set "if_find" to "false", and give me a reponse:
                "I can't find it..., maybe it's not here !"
            5. If the object is in the above list and it exists in the image, then you MUST set "if_find" to "true", and leave the response empty.

            
            Example:
                {
                    "if_find": false,
                    "response": "I've found it, but our system can't recognize it.",
                    "object": [
                        {"name": "apple", "label": ""}
                    ]
                }

            
                {
                    "if_find": false,
                    "response": "Sorry, I can't find it..., maybe it's not here !",
                    "object": [
                        {"name": "elephant", "label": ""}
                    ]
                }

                {
                    "if_find": false,
                    "response": "I've found it, but our system can't recognize it.",
                    "object": [
                        {"name": "scissors", "label": ""}
                    ]
                }

                {   "if_find": true,
                "response": "",
                "object": [
                    {"name": "salt bottle", "label": "grobes MeerSalz"}
                ]
                }

                {
                "if_find": true,
                "response": "",
                "object": [
                    {
                    "name": "pepper bottle",
                    "label": "Schwarzer Pfeffer ganz"
                    }
                ]
                }
 
"""
example = """
            Assume you've seen a cucumber in the image.

            Assume you've seen a pepper bottle in the image, and it has a label that says "Schwarzer Pfeffer ganz".

            Assume you haven't seen an elephant in the image.

            Assume you've seen a salt bottle in the image, and it has a label that says "grobes MeerSalz".

            Assume you've seen a pair of scissors in the image.

            Assume you haven't seen a detergent bottle in the image. 

            Assume you've seen a book in the image, with its title as "Introduction to Python Programming".


"""

assistant_prompt = """
            {   "if_find": true,
                "response": "",
                "object": [
                    {"name": "cucumber", 
                     "label": ""
                    }
                ]
            }

            {   "if_find": true,
                "response": "",
                "object": [
                    {
                    "name": "pepper bottle",
                    "label": "Schwarzer Pfeffer ganz"
                    }
                ]
            }

            {   "if_find": false,
                "response": "I can't find it..., maybe it's not here !",
                "object": [
                    {"name": "elephant", 
                     "label": ""
                    }
                ]
            }

            {   "if_find": true,
                "response": "",
                "object": [
                    {"name": "salt bottle", 
                    "label": "grobes MeerSalz"
                    }
                ]
            }

            {   "if_find": false,
                "response": "I've found it, but our system can't recognize it.",
                "object": [
                    {"name": "scissors", 
                     "label": ""
                    }
                ]
            }

            {   "if_find": false,
                "response": "Sorry, I can't find it..., maybe it's not here !",
                "object": [
                    {"name": "detergent bottle", 
                     "label": ""
                    }
                ]
            }

            {   "if_find": false,
                "response": "I've found it, but our system can't recognize it.",
                "object": [
                    {"name": "book", 
                     "label": "Introduction to Python Programming"
                    }
                ]
            }


"""

user_prompt = """
            car.
"""