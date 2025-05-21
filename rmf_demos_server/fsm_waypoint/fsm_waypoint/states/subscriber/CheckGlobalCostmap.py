# ros2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class CheckGlobalCostmap(smach.State):
    def __init__(self, node, count, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None
        self.frequency_count = 0
        self.target_count = count

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(DiagnosticArray,
                                                  'diagnostics',
                                                  self.diagnostics_callback,
                                                  10)
        self.result = None
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
                outcome = 'succeeded'
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def diagnostics_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        for status in msg.status:
            if status.name == "web_diagnostic_node: costmap topic status":
                for value in status.values:
                    if value.key == "Actual frequency (Hz)":
                        frequency = float(value.value)
                        if frequency >= 1.4:  # simulation mode 0.4, real robot 1.4
                            debug(f'[{current_time}] Frequency: {frequency} Hz')
                            self.frequency_count += 1
                            if self.frequency_count >= self.target_count:
                                self.result = True
                                self.frequency_count = 0
                                debug(f'[{current_time}] Frequency condition met for 5 consecutive seconds.')
                        else:
                            self.frequency_count = 0
                        return