# Inference of Human Knowledge by Combining Large Language Models

Significant advances have been made in Large Language Models (LLM), enabling robots to extract crucial knowledge through spoken interaction with humans or even just by listening to humans interacting with each other. Here, we present a modular implementation which explores how to ex-
tract human knowledge from human-human speech interaction by combining different LLMs (like ChatGPT and GPT-3) with state-of-the-art embedding and ASR models, a SQL database, and Ontologies. We have combined them using a hierarchical task planning structure to achieve our final goal of human
knowledge extraction, dividing this task into several tasks/sub- tasks/actions to focus the problem and obtain the best results in each step. These tasks/sub-tasks/actions can be combined in series or in parallel and include, for example, the recognition of the agent (human, robot, etc) roles to differentiate them and the detection of the topic of conversation.

## Requisites

- Docker Engine CE on Ubuntu (https://docs.docker.com/engine/install/ubuntu/)

## Installation steps

1. Download the dockerfile of this repository
2. Inside the terminal, go to the directory where the dockerfile is stored and execute "docker build --no-cache -t ai4hri:1.0 ."
3. Inside the terminal, execute "docker run --privileged -it --rm --env OPENAI_ORG_ID=&lt;insert your own OpenAI org ID&gt; --env OPENAI_API_KEY=&lt;insert your own OpenAI key&gt; --env LANGUAGE_WHISPER='en' --device /dev/snd:/dev/snd ai4hri:1.0"
4. Wait for the message "Node whisper initialized. Listening..." and start talking to the microphone.

Remember to execute "export ROS_MASTER_URI=http://&lt;insert IP address where roscore is running&gt;:11311" to listen to the ROS topics outside the docker container.
