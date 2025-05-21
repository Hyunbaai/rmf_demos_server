# ros2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class CheckAccuracy(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(DiagnosticArray,
                                                  'diagnostics',
                                                  self.diagnostics_callback,
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
                outcome = 'succeeded'
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def diagnostics_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        for status in msg.status:
            if status.name == "ublox_gps_node: fix":
                horizontal_acc = None
                vertical_acc = None

                for value in status.values:
                    if value.key == "Horizontal Accuracy [m]":
                        horizontal_acc = float(value.value)
                    elif value.key == "Vertical Accuracy [m]":
                        vertical_acc = float(value.value)

                if horizontal_acc is not None and vertical_acc is not None:
                    if horizontal_acc <= 0.02 and vertical_acc <= 0.02:
                        self.result = True
                        debug(f'[{current_time}] GPS accuracy is within the threshold 0.02m')