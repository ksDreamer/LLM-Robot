# not complete, need to add returnValue to ros.

from litellm import completion

response = completion(
    model="ollama/phi3:latest", # or any local llm you have.
    prompt = '''Consider the following ontology:
                    {"action": "go_to_goal", "params": {"location": {"type": "str", "value": location}}}
                    {"action": "move", "params": {"linear_speed": linear_speed, "distance": distance, "is_forward": is_forward}}
                    {"action": "rotate", "params": {"angular_velocity": angular_velocity, "angle": angle, "is_clockwise": is_clockwise}}

                    You will be given human language prompts, and you need to return a JSON conformant to the ontology. Any action not in the ontology must be ignored. Here are some examples.

                    prompt: "Move forward for 1 meter at a speed of 0.5 meters per second."
                    returns: {"action": "move", "params": {"linear_speed": 0.5, "distance": 1, "is_forward": true, "unit": "meter"}}

                    prompt: "Rotate 60 degree in clockwise direction at 10 degrees per second and make pizza."
                    returns: {"action": "rotate", "params": {"angular_velocity": 10, "angle": 60, "is_clockwise": true, "unit": "degrees"}}
                    
                    prompt: "go to the bedroom, rotate 60 degrees and move 1 meter then stop"
                    returns: {"action": "sequence", "params": [{"action": "go_to_goal", "params": {"location": {"type": "str", "value": "bedroom"}}}, {"action": "rotate", "params": {"angular_velocity": 30, "angle": 60, "is_clockwise": false, "unit": "degrees"}}, {"action": "move", "params": {"linear_speed": 1, "distance": 1, "is_forward": true, "unit": "meter"}}, {"action": "stop"}]}
                    '''   
    messages=[{ "content": prompt,"role": "user"}],
    api_base="http://localhost:11434",
)

print(response)