
# ros2
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy as History

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class SoundPlay(smach.State):
    def __init__(self, node, name, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node
        self.file = '/root/scripts/sound/sounds/voice/' + name + '.mp3'
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=History.KEEP_LAST,
            depth=1
        )
        self.pub_ = self.node.create_publisher(String,
                                               '/sound_file',
                                               qos_profile)

    def execute(self, userdata):
        start_time = time.time()
        while rclpy.ok():
            msg = String()
            msg.data = self.file
            self.pub_.publish(msg)
            debug(f'Sent file: {msg.data}')
            break

        outcome = 'succeeded' # because timeout not mandatory
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break
            time.sleep(0.1)
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    outcome = 'succeeded'
                    break
            else:
                break
        return outcome