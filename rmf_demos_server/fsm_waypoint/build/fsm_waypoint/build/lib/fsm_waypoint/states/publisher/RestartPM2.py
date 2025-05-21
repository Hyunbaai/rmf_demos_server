
# ros2
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical
import json


class RestartPM2(smach.State):
    def __init__(self, node, target, function, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node
        self.target = target
        self.function = function
        self.pub_ = self.node.create_publisher(String,
                                               '/action_client_aggr',
                                               QoSProfile(depth=10))

    def execute(self, userdata):
        start_time = time.time()
        message = {
            "command": 'docker',
            "target": self.target,
            "function": self.function
        }
        while rclpy.ok():
            msg = String()
            msg.data = json.dumps(message)
            self.pub_.publish(msg)
            debug(f'Sent command: {msg.data}')
            break

        outcome = 'aborted' # because timeout mandatory
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
        return outcome