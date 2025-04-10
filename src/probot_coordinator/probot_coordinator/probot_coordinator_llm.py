import time
from threading import Thread, Event
from enum import Enum
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_Result
from trajectory_msgs.msg import JointTrajectoryPoint

from probot_touch_interfaces.msg import TouchEvent
from probot_ui_interfaces.msg import FaceUi
from probot_voice_interfaces.srv import DoTTS, SetSTT

from openai import OpenAI

TAG = 'ProbotLLMCordinator'

TOUCH_TOPIC = '/touch_event'
STT_TOPIC = '/voice_stt'
UI_TOPIC = '/face_ui'
SERVO_CONTROL_TOPIC = '/joint_trajectory_controller/follow_joint_trajectory'
TTS_SERVICE = 'do_tts'
STT_SERVICE = 'set_stt'

class ProbotState(Enum):
    pass

class ProbotLLMCoordinator(Node):
    _logger : RcutilsLogger = None
    
    _engaged : bool = False

    _chat_gpt_thread : Thread = None
    # Variables for handling the STT 
    _current_sentence : str = ''
    _sentence_event : Event = None
    # ChatGPT client
    _open_ai_client : OpenAI = None
    _system_prompt : str = ''
    # ROS2 items
    _ui_publisher : Publisher = None
    _engage_sub : Subscription = None

    def __init__(self):
        super().__init__(TAG)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('llm_url', ''),
                ('llm_key', ''),
                ('system_prompt', '')
            ]
        )

        self._logger = self.get_logger()
        
        llm_key = self.get_parameter('llm_key').get_parameter_value().string_value
        if not llm_key or llm_key == '':
            raise RuntimeError("OpenAI key is not a valid string")

        system_prompt = self.get_parameter('system_prompt').get_parameter_value().string_value
        if system_prompt and not system_prompt == '':
            self._logger.info(f'system prompt is : {system_prompt}')
            self._system_prompt = system_prompt

        #Create OpenAI client
        llm_url = self.get_parameter('llm_url').get_parameter_value().string_value
        if llm_url and not llm_url == '':
            self._open_ai_client = OpenAI(base_url=llm_url, api_key=llm_key)
        else:
            self._open_ai_client = OpenAI(api_key=llm_key)

        # Subscribe to engage topic
        self._engage_sub = self.create_subscription(
            TouchEvent,
            TOUCH_TOPIC,
            self._touch_handler,
            1)
        
        # Subscribe to stt topic
        self._stt_sub = self.create_subscription(
            String,
            STT_TOPIC,
            self._stt_handler,
            1)
        
        # Create a publisher for changing faces
        self._ui_publisher = self.create_publisher(
            FaceUi,
            UI_TOPIC,
            1
        )

        # Ros2 control action publishing
        self._joint_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            SERVO_CONTROL_TOPIC
        )
        
        self._sentence_event = Event()
        self._chat_gpt_thread = Thread(target=self._chatgpt_thread, daemon=False)
        self._chat_gpt_thread.start()

        self._set_face('sleep')
        self._do_head_up_animation()

        self._logger.info('Probot LLM Coordinator Ready!')
        

    def _touch_handler(self, msg):
        event : TouchEvent = msg.event
        if event == TouchEvent.HEAD_TOUCH:
            self._logger.info(f'Head touch recived')
            self._engaged = True if not self._engaged else False
            self._logger.info(f'Engaged : {self._engaged}')
            self._set_stt(self._engaged)
              
            if self._engaged:
                self._set_face('default')
            else:
                self._set_face('sleep')

        elif event == TouchEvent.BOTH_SIDES_TOUCH:
            self._logger.info(f'Both side touch recived')
        elif event == TouchEvent.LEFT_SIDE_TOUCH:
            self._logger.info(f'Left side touch recived')
        elif event == TouchEvent.RIGHT_SIDE_TOUCH:
            self._logger.info(f'Right side touch recived')

    def _stt_handler(self, msg):
        if self._engaged:
            self._logger.info(f'Sentence recived {msg}')
            self._current_sentence = msg.data
            self._sentence_event.set()
    
    def _chatgpt_thread(self):
        self._logger.info(f'ChatGPT thread running, client: {not self._open_ai_client.is_closed()}')
        while True:
            sentence_ready = self._sentence_event.wait(1)
            if sentence_ready and not self._current_sentence == '':
                self._set_stt(False)
                self._logger.info(f'Passing sentence to ChatGPT: {self._current_sentence}')
                
                data = []
                if self._system_prompt != '':
                    data.append({
                        "role": "system",
                        "content": self._system_prompt,
                    })
                
                data.append({
                    'role': 'user',
                    'content': self._current_sentence,
                })

                result = self._open_ai_client.chat.completions.create(
                    messages=data,
                    model='gpt-4o-mini',
                )
                self._logger.info(f'result: {result.choices[0].message.content}')
                # Setting face ui
                self._set_face('happy')
                # Calling tts
                self._do_tts(result.choices[0].message.content)
                # Resetting face ui
                self._set_face('default')
                self._set_stt(self._engaged)
                self._sentence_event.clear()
    
    def _set_face(self, face : str):
        ui_msg = FaceUi()
        ui_msg.face_ui_item = face
        self._ui_publisher.publish(ui_msg)

    def _do_tts(self, text : str, lang : str = 'en'):
        client = self.create_client(DoTTS, TTS_SERVICE)
        while not client.wait_for_service(1.0):
            self._logger.info('Waiting for the TTS service')
        request = DoTTS.Request()
        request.text = text
        tts_response = client.call(request)
        self._logger.info(f'TTS done: {tts_response}')
    
    def _set_stt(self, enable : bool):
        client = self.create_client(SetSTT, STT_SERVICE)
        while not client.wait_for_service(1.0):
            self._logger.info('Waiting for the STT service')
        request = SetSTT.Request()
        request.enable = enable
        client.call_async(request)

    def _do_head_up_animation(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']

        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.2, 0.0]
        point.time_from_start = Duration(sec=2)

        goal_msg.trajectory.points = [point]

        self._do_animation(goal_msg)

    def _do_animation(self, animation : FollowJointTrajectory.Goal):
        try:
            self._joint_client.wait_for_server()
            self._animation_result_future = self._joint_client.send_goal_async(animation)
            self._animation_result_future.add_done_callback(self._animation_result_callback)
        except Exception as e:
            self._logger.error(f'Exception in _do_animation: {e}')

    def _animation_result_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Animation goal not accepted')
            else:
                self.get_logger().info('Animation goal accepted, awaiting completion')
                try:
                    self._animation_completed_future = goal_handle.get_result_async()
                    self._animation_completed_future.add_done_callback(self._animation_completed_callback)
                except Exception as e:
                    if e:
                        self._logger.error(str(e))
                    else:
                        self._logger.error('Exception in animation callback!')

    def _animation_completed_callback(self, future):
        try:
            result : FollowJointTrajectory_Result = future.result().result
            if result.error_code == 0:
                self._logger.info('Animation completed')
        except Exception as e:
            self._logger.error(e)
