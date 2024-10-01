import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AsrRivaBridge(Node):
    '''
    '''

    def __init__(self):
        super().__init__('asr_riva_bridge')

        # Declare parameters with default values
        self.declare_parameter('asr_topic', 'asr')
        self.declare_parameter('pub_freq', '0.1')  # [s]
        self.declare_parameter('asr_output_file', '/tmp/out.txt')

        # Get parameter values
        asr_topic = self.get_parameter('asr_topic').value
        self.pub_freq = float(self.get_parameter('pub_freq').value)
        self.asr_output_file = self.get_parameter('asr_output_file').value

        self.publisher_ = self.create_publisher(String, asr_topic, 10)

        self.timer = self.create_timer(self.pub_freq, self.timer_callback)

    def timer_callback(self):
        '''
        Callback function reading ASR output from file and publishing it to
        a ROS2 topic.

        NOTE Robot spoken output from the TTS topic is filtered out using the
        'tts_is_speaking' topic.
        '''
        # Remove ASR output file after reading
        while True:
            try:
                # Read ASR output from file
                with open(self.asr_output_file, 'r') as f:
                    asr_output = f.read()
                # Empty ASR output file
                with open(self.asr_output_file, 'w') as f:
                    f.write('')
                break
            except OSError as e:
                self.get_logger().info(f"Failed to read and write file: {e}")
        if len(asr_output) == 0:
            return

        asr_output = self.preprocess_asr_output(asr_output)
        msg = String()
        msg.data = '<User said> ' + asr_output
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {asr_output}")

    def preprocess_asr_output(self, asr_output):
        asr_output = asr_output.replace('## ', '')
        asr_output = asr_output.replace(' \n', '')
        return asr_output


def main(args=None):
    rclpy.init(args=args)

    asr_riva_bridge = AsrRivaBridge()

    rclpy.spin(asr_riva_bridge)

    asr_riva_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
