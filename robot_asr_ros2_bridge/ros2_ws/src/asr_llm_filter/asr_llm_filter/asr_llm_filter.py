import textwrap
import time

import rclpy
from llm_action_interfaces.action import LLM
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_pt import asr_filter_pt
from robot_state_interfaces.srv import State
from std_msgs.msg import String


class AsrLlmFilter(Node):
    '''
    '''

    def __init__(self):
        super().__init__('asr_llm_filter')

        # Declare parameters with default values
        self.declare_parameter('asr_topic', 'asr')
        self.declare_parameter('asr_filter_topic', 'asr_filter')
        self.declare_parameter('msg_prefix', '<User said>')
        self.declare_parameter('num_state_chunks', 10)

        # Get parameter values
        asr_topic = self.get_parameter('asr_topic').value
        asr_filter_topic = self.get_parameter('asr_filter_topic').value
        self.msg_prefix = self.get_parameter('msg_prefix').value
        self.num_state_chunks = self.get_parameter('num_state_chunks').value

        self.get_logger().info(f'asr_topic: {asr_topic}')
        self.get_logger().info(f'asr_filter_topic: {asr_filter_topic}')
        self.get_logger().info(f'msg_prefix: {self.msg_prefix}')
        self.get_logger().info(f'num_state_chunks: {self.num_state_chunks}')

        self.subscription = self.create_subscription(
            String, asr_topic, self.asr_callback, 10)

        self.publisher_ = self.create_publisher(String, asr_filter_topic, 10)

        #######################
        #  LLM Action Client
        #######################
        self.declare_parameter('llm_action_server',
                               'llm_action_server_gen_8b_action')
        self.declare_parameter('max_tokens', 1000)
        self.declare_parameter('llm_temp', 0.6)
        self.declare_parameter('llm_seed', 14)
        llm_action_server_name = self.get_parameter('llm_action_server').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.llm_temp = self.get_parameter('llm_temp').value
        self.llm_seed = self.get_parameter('llm_seed').value

        self.get_logger().info(f'asr_topic: {asr_topic}')
        self.get_logger().info(f'asr_topic: {asr_topic}')

        self.action_client_ = ActionClient(self, LLM, llm_action_server_name)

        #################################
        #  Service clients
        #################################
        self.state_client = self.create_client(State, 'get_state')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                '\'state_client\' service not available, waiting again...')

        self.state_req = State.Request()

    def asr_callback(self, msg):
        '''
        '''
        # ASR message to be filtered
        self.asr = msg.data

        # Remove message prefix
        self.asr = self.asr.replace(self.msg_prefix + ' ', '')

        state_future = self.state_client.call_async(self.state_req)
        state_future.add_done_callback(self.state_callback)

    def state_callback(self, state_future):
        '''
        '''
        # Unpack state response
        state = state_future.result().out_state

        # State of current action cycle
        visual_info = self.extract_state_part(state, 'visual_information')
        state_chunks = self.extract_state_part(
            state, 'state_chunks', exclude_tag=True)

        # Reduce length of chunks to "immediate context"
        state_chunks = self.extract_state_chunks(
            state_chunks, self.num_state_chunks)

        state = visual_info + '\n\n' + state_chunks

        self.user_msg = asr_filter_pt(self.asr, state)

        llm_goal = LLM.Goal()
        llm_goal.system_prompt = ''
        llm_goal.prompt = self.user_msg
        llm_goal.max_tokens = self.max_tokens
        llm_goal.temp = self.llm_temp
        llm_goal.seed = self.llm_seed

        self.t0 = time.time()

        self.action_client_.wait_for_server()
        self.send_llm_goal_future = self.action_client_.send_goal_async(
            llm_goal)
        self.send_llm_goal_future.add_done_callback(
            self.llm_goal_response_callback)

    def llm_goal_response_callback(self, llm_future):
        '''
        '''
        goal_handle = llm_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_llm_result_future = goal_handle.get_result_async()
        self.get_llm_result_future.add_done_callback(
            self.get_llm_result_callback)

    def get_llm_result_callback(self, llm_future):
        '''
        '''
        llm_result_msg = llm_future.result().result
        asr_filter = llm_result_msg.response

        self.t1 = time.time()
        dt = self.t1 - self.t0

        self.get_logger().info(
            f'ASR: {self.asr} | Filtered: {asr_filter} ({dt:.2f} s)')

        # Log prompt and result to file
        ts = self.str_time()
        with open(f'/tmp/{ts}_prompt_asr.txt', 'w') as f:
            f.write(self.log_str('', self.user_msg, asr_filter))

        # Filtered noise
        if '$none$' in asr_filter:
            return

        # Add back message prefix
        asr_filter_prefix = self.msg_prefix + ' ' + asr_filter

        # Publish filtered ASR
        asr_filter_msg = String(data=asr_filter_prefix)
        self.publisher_.publish(asr_filter_msg)

    @staticmethod
    def extract_state_part(
            state: str,
            tag: str,
            exclude_tag: bool = False,
            ) -> str:
        '''
        Returns a substring representing part of state inside a tag like
            <state_chunks>
            ...
            </state_chunks>.

        Args:
            state: Full state.
            tag: Extract part of state within a tag (ex: state_chunks)
        '''
        # Split the text into lines
        lines = state.split('\n')

        # Find the indices of the last pair of <tag> and </tag> tags
        start_index = -1
        end_index = -1

        for i, line in enumerate(lines):
            if line.strip() == f'<{tag}>':
                start_index = i
            elif line.strip() == f'</{tag}>':
                end_index = i

        if exclude_tag:
            start_index += 1
            end_index -= 1

        # If we found both tags
        if start_index != -1 and end_index != -1 and start_index < end_index:
            # Extract the content between the tags, including the tags
            chunk = '\n'.join(lines[start_index:end_index+1])
            return chunk.strip()
        else:
            return f"No valid {tag} found"

    @staticmethod
    def extract_state_chunks(state_chunks: str, num: int) -> str:
        '''
        Returns the N bottom / most recent state chunks.

        Args:
            state_chunks: Sequence of state information chunks formatted as:
                topic: /asr
                ts: 2024-08-31 17:45:23
                data: <Robot heard voice> User says: Hey robot, I'm heading ...
                ---
                topic: /action_response
                ts: 2024-08-31 17:45:25
                data: <Robot completed reply action> Robot says: Certainly! ...
                ---
            num: Number of chunks to extract counting from the bottom.
        '''
        # Split the input string into chunks
        chunks = state_chunks.strip().split('---')

        # Remove any empty chunks
        chunks = [chunk.strip() for chunk in chunks if chunk.strip()]

        # Get the N bottom chunks
        bottom_chunks = chunks[-num:]

        # Join the chunks back together with the separator
        result = '\n---\n'.join(bottom_chunks) + '\n---'

        return result

    @staticmethod
    def str_time() -> str:
        '''
        Returns a timestamp in the format '1729514281_1626208'.
        '''
        return str(time.time()).replace('.', '_')

    @staticmethod
    def log_str(sys_msg: str, usr_msg: str, res: str) -> str:
        '''
        Returns a log string representing an LLM prompt and response.
        '''
        log_str = textwrap.dedent('''
            <sys_msg>
            {}
            </sys_msg>
            <usr_msg>
            {}
            </usr_msg>
            <res>
            {}
            </res>
            ''')
        log_str = log_str.format(sys_msg, usr_msg, res)

        # Remove leading/trailing line breaks
        log_str = log_str.strip()

        return log_str


def main(args=None):
    rclpy.init(args=args)

    node = AsrLlmFilter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
