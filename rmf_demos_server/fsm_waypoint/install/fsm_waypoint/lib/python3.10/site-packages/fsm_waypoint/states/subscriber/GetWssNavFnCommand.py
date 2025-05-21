# ros2
import rclpy
from std_msgs.msg import String

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical
import json


class GetWssNavFnCommand(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'navfn'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None
        self.obstacle_timeout = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(String,
                                                  'wss_command',
                                                  self.wss_command_callback,
                                                  10)
        self.result = None  # initialize result
        self.obstacle_timeout = userdata.blackboard.obstacle_timeout
        start_time = time.time()
        outcome = 'aborted'
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
            if self.result:
                info(f'Next State: {self.result}')
                outcome = self.result
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def wss_command_callback(self, msg: String):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if msg.data:
            info(f'[{current_time}] Received data: {msg.data}')
            try:
                data = json.loads(msg.data)
                if 'Navfn' in data and self.obstacle_timeout:
                    self.result = 'navfn'

            except json.JSONDecodeError:
                info(f'[{current_time}] Failed to decode JSON data')
        else:
            info(f'[{current_time}] Received empty data array. No Event available.')