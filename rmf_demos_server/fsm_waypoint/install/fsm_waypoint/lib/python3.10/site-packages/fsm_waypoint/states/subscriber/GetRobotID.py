# ros2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical
import json


class GetRobotID(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.robot_id = None
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(
            DiagnosticArray,
            'diagnostics',
            self.diagnostics_callback,
            10
        )
        self.result = None  # 결과 초기화
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
                userdata.blackboard.robot_id = self.robot_id
                outcome = 'succeeded'
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def diagnostics_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        for status in msg.status:
            if status.name == "SystemInfo":
                for value in status.values:
                    if value.key == "System Info":
                        try:
                            # JSON parsing
                            system_info = json.loads(value.value)
                            mac_address = system_info.get('mac_address', '')
                            # remove last 6 characters
                            if len(mac_address) >= 6:
                                mac_address_trimmed = mac_address.replace(':', '')[-6:]
                                self.robot_id = mac_address_trimmed
                                self.result = True  # 결과 설정
                                info(f'[{current_time}] robot_id set to {mac_address_trimmed}')
                        except json.JSONDecodeError as e:
                            error(f'[{current_time}] Failed to parse System Info JSON: {e}')