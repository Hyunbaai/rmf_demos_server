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


class GetWssCancelCommand(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'cancel'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(String,
                                                  'wss_command',
                                                  self.wss_command_callback,
                                                  10)
        self.result = None  # initialize result
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
                data = json.loads(msg.data)  # JSON 형식으로 데이터 파싱
                waypoint = ['robot_id', 'path_id', 'waypoint_id', 'type']
                waypoint_found = next((key for key in waypoint if key in data), None)

                if 'Cancel' in data:
                    self.result = 'cancel'
                elif waypoint_found:
                    if data['type'] < 0:  # Cancel
                        self.result = 'cancel'
            except json.JSONDecodeError:
                info(f'[{current_time}] Failed to decode JSON data')
        else:
            info(f'[{current_time}] Received empty data array. No Event available.')