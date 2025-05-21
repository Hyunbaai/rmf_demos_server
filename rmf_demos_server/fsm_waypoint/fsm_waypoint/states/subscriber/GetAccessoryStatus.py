# ros2
import rclpy
from std_msgs.msg import String

# fsm
import smach

# utils
import time
import json
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class GetAccessoryStatus(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.bottom_led = None
        self.top_led = None
        self.lighting = None
        self.fan = None
        self.door = None
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(
            String,
            '/diagnostic_aggr',
            self.diagnostics_aggr_callback,
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
                userdata.blackboard.door = self.door
                userdata.blackboard.fan = self.fan
                userdata.blackboard.lighting = self.lighting
                userdata.blackboard.top_led = self.top_led
                userdata.blackboard.bottom_led = self.bottom_led
                info(
                    f'Door: {self.door}, Fan: {self.fan}, Lighting: {self.lighting}, Top LED: {self.top_led}, Bottom LED: {self.bottom_led}')

                outcome = 'succeeded'
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def diagnostics_aggr_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            diagnostics_aggr = json.loads(msg.data)
            if 'accessory' in diagnostics_aggr:
                accessory_data = diagnostics_aggr['accessory']["value"]
                self.door = accessory_data.get('door', 0)
                self.fan = accessory_data.get('fan', 0)
                self.lighting = accessory_data.get('lighting', 0)
                self.top_led = accessory_data.get('top_led', 0)
                self.bottom_led = accessory_data.get('bottom_led', 0)
                self.result = True

        except json.JSONDecodeError:
            print(f"Error decoding JSON at {current_time}")
        except Exception as e:
            print(f"An error occurred at {current_time}: {e}")