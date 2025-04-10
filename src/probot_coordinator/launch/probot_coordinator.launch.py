import launch
from launch_ros.actions import Node

LLM_URL = 'http://localhost:11434/v1'
LLM_KEY  = '<YOUR-LLM-API-KEY>'

def generate_launch_description():
    return launch.LaunchDescription([
        # Coordinator Node
        Node(
            package='probot_coordinator',
            executable='coordinator',
            name='chat_gpt_coordinator',
            parameters = [{
                'llm_url' : LLM_URL,
                'llm_key' : LLM_KEY,
                'system_prompt': 'Rispondi in modo breve. Usa un tono accomodante e cortese, non usare punti elenco, non usare periodi lunghi. Si conciso nelle risposte e nel saluto.'
            }]
        ),
    ])
