Kevin Stark 高梦扬

# Introduction

As robot technology becomes widely applied, simplifying human-robot interaction has become a hot topic for research. Traditional robot operation interfaces (such as CNC machines in factories and large robotic arms on assembly lines) usually require users to have professional programming knowledge and skills (such as writing Gcode and PLC programming), which limits the use by non-professionals. To address this issue, we propose a human-robot interaction system that controls robot movement using a large language model (LLM) within the Robot Operating System (ROS). This system achieves intuitive control of the robot, reducing the barrier to entry, allowing ordinary users to interact with robots more easily and naturally.
We all know that controlling robots is a very important matter. In many courses on the theory of robotics, we have learned many core knowledge of robot motion control, including many concepts and methods in kinematics and dynamics, such as forward kinematics solution, inverse kinematics solution, Cartesian coordinate transformation, and so on. These knowledge are crucial for robot operation, but they are often too complex for non-professionals to master.
The process for a robot to complete a task is roughly as follows:
Connect the planning group (group) required for control, set the target pose (joint space or Cartesian space), set motion constraints and path constraints, and use motion solvers and planners (such as the Moveit framework) to plan a feasible motion path, and then execute.
There are already many mature kinematics solvers. However, how to define a task and how to decompose a task into instructions that a program can execute or target coordinates and poses, so that the next solver/planner can calculate and plan, is still a challenging task.
This project takes the classic small turtle in ROS, turtlesim, as an example, and implements the use of a large language model to understand user input and convert it into turtlesim node operation instructions in ROS. Under the Robot Operating System (ROS), this project combines the rapidly developing technology related to large language models (LLM) with robot body control, enabling the robot to better understand human instructions and requirements, reducing the barrier to human-robot interaction, and expanding the types of tasks that robots can perform.
This project is a small attempt, and we believe that through this approach, users can interact with robots more easily, thus expanding the application scope of robot technology.

![figure0_running](./figure0_running.jpg)

The core logic of the system is divided into the following steps:
1. Users send control commands to the system through natural language (voice or text).
2. The system parses the command using a LLM with customized knowledge and converts it into ROS control code.
3. In ROS, each node executes the code to control the robot to perform corresponding actions.
4. (Possible future development direction) Add a feedback system, where the robot will feed back the execution results to the system, which will then convert the results into natural language and provide feedback to the user.

# Tools

System: Ubuntu 22.04

ROS: ROS 2 Humble

LLM: 

* A: Commercial LLM, like ChatGPT
* B: Open-Source LLM, like Llama3 (optional)
* Use Ollama to bulid a local llm environment, use LiteLLM to provide local llm's API.

# Getting Start

1. Clone the repo to local computer

   ```sh
   	git clone https://github.com/ksdreamer/llm-ros.git
   ```

2. Environment setup

* Add OpenAI API (or local large language model API, please refer to the “LLM” section for details)

```
echo 'export OPENAI_API_KEY=<put_your_api_key_here>' >> ~/.bashrc
```

* For ROS2 Humble version (Ubuntu 22.04), you need to downgrade setuptools to the specified version (58.0.2)

```
pip3 install --upgrade setuptools==58.0.2
```

* Enter the workspace

```
cd llm-ros
```

* Install Python dependencies

```
	pip3 install -r requirements.txt
```

* ROS uses colcon to build packages

```
colcon build --packages-select rosgpt
```

* ROS source workspace

```
source install/setup.bash
```

3. Use it

   We will use two functional packages: rosgpt and turtlesim.

   Four nodes:

   rosgpt has three: rosgpt, rosgptparser_turtlesim, rosgpt_client_node.

   turtlesim has one: turtlesim_node.

   Therefore, you need to open four terminals, which can be quickly created by pressing “ctrl+alt+t”.

   Before running each node of the rosgpt package (steps 1, 3, 4), you must enter the work directory and source install/setup.bash.

* Run the ROSGPT Flask server

```sh
ros run rosgpt rosgpt
```

* Run the turtlesim node

``` sh
ros2 run turtlesim turtlesim_node
```

* Run rosgptparser_turtlesim.py

```sh
 ros2 run rosgpt rosgptparser_turtlesim
```

4. Run rosgpt_client_node.py

```sh
ros2 run rosgpt rosgpt_client_node
```



In the terminal of rosgpt_client_node, issue English commands to the robot (can be personalized and colloquial), for example, “ı want that you move 1 meter speed 0.8”.

# Result and Disscusion

we send a message in terminal, and the turtle simulator will run as we want. 

![figure1_move_before](./figure1_move_before.png)

![figure2_move_after](./figure2_move_after.png)

## LLM

The most important idea of this project is to use large language models such as ChatGPT-3.5 or Llama3. From the chatgpt_ros2_node node, according to certain requirements (Prompt, returning JSON format), convert user input, and then input it to the turtlesim_controller node.

### Prompt

                    ‘’‘Consider the following ontology:
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

Because the length of the prompt is variable, it needs to be split and then recombined. When the prompt is called with `prompt +=`, it will append the string on the right to the end of the prompt.
If the user input `text_command` is:
"Hey robot, please move 5 meters to the right and pick up the package on the table".
Then the returned JSON command will be:
{"command": "MOVE", "params": {"direction": "RIGHT", "distance": 5, "unit": "METERS", "pickup": true}}.
In fact, if there is no Persona or Embedding Vector, the node will return the `prompt + text_command` together to the LLM.

### Local LLM(Optional)

If you want to use the local llm, the models you can refer to [llama.cpp](https://github.com/ggerganov/llama.cpp) ,[ollama](https://ollama.com/) and [LMStudio](https://lmstudio.ai. The API of local LLM you can generate from LiteLLM](https://www.litellm.ai)。



We use Ollama to handle local LLM

```sh
curl -fsSL https://ollama.com/install.sh
```

Find the LLM that match your requirement on https://ollama.com/library , and type this: (we use llama3 as the open-source LLM)

```sh
ollama run llama3
```

enter in terminal to install LiteLLM.

```sh
pip install litellm
```

## ROS Package and Node

RQT is shown below:

![figure3_rqt_all](./figure3_rqt_all.jpg)

![figure4_rqt_nodes_only](./figure4_rqt_nodes_only.jpg)

### rosgpt_client

in file: rosgpt_client_node.py

```python
class ROSGPTClient(Node):
    def __init__(self):
        super().__init__('rosgpt_client')
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value

        self.get_logger().info('ROSGPT client node started')

        self.send_text_command()

    def send_text_command(self): 
        while rclpy.ok():
            print('Enter a move command or a rotate command. The current ROSGPTParser of rosgpt_turtlesim does not multiple command. Will be extended later')
            text_command = input("Enter a text command: ")
            data = {'text_command': text_command}

            response = requests.post(self.server_url, data=data)

            if response.status_code == 200:
                try:
                    response_str = response.content.decode('utf-8')
                    response_dict = json.loads(response_str)

                    self.get_logger().info('Response: {}'.format(response_dict['text']))
                    self.get_logger().info('JSON: {}'.format(json.loads(response_dict['json'])))
                except Exception as e:
                    print('[Exception] An unexpected error occurred:', str(e)) 
            else:
                self.get_logger().error('Error: {}'.format(response.status_code))
```

Define a node rosgpt_client，it receives 'server_url'，and has a function 'send_text_command'

function'send_text_command' ask user to enter a command, then read it and send to ROSGPT service.

### chatgpt_ros2_node

in file: rosgpt.py

```python
class ROSGPTNode(Node):
    def __init__(self):
        """
        Initialize the ROSGPTNode class which is derived from the rclpy Node class.
        """
        # Call the superclass constructor and pass the name of the node
        super().__init__('chatgpt_ros2_node')\
        # Create a publisher for the 'voice_cmd' topic with a message queue size of 10
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)

    def publish_message(self, message):
        """
        Publish the given message to the 'voice_cmd' topic.
        Args:
            message (str): The message to be published.
        """
        msg = String() # Create a new String message 
        msg.data = message # Convert the message to a JSON string and set the data field of the message
        self.publisher.publish(msg) # Publish the message using the publisher 
        #print('message Published: ', message) # Log the published message
        #print('msg.data Published: ', msg.data) # Log the published message
```

Define chatgpt_ros2_node，create a publisher，publish commands by topic 'voice_cmd' in JSON format.



# Reference

In our project, we implemented a human-robot interaction system using the ROSGPT package, developed by Anis Koubaa (2023). ROSGPT integrates the ChatGPT language model with the Robot Operating System (ROS) to enable natural language understanding and generation for robotic tasks. The project is available on github, [ROSGPT: ChatGPT Interface for ROS2 for Human-Robot Interaction](https://github.com/aniskoubaa/rosgptKoubaa), 

Koubaa, A. (2023). ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS. Preprints.org, 2023, 2023040827. https://www.preprints.org/manuscript/202304.0827/v2

˙

We also use these tools:

[ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/index.html)

[Ollama]( https://ollama.com)

[Litellm](https://docs.litellm.ai/docs)

## Other Reference

[鱼香 ROS](https://fishros.com/d2lros2/)

[古月居·ROS2入门21讲](https://www.bilibili.com/video/BV16B4y1Q7jQ/)

[Python import openai library ImportError](https://stackoverflow.com/questions/71873182/no-module-named-openai)

https://github.com/NoneJou072/robochain

https://github.com/Auromix/ROS-LLM

https://github.com/GT-RIPL/Awesome-LLM-Robotics