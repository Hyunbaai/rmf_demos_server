# ros2
import rclpy
from std_msgs.msg import String

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class CheckObjectDetect(smach.State):
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
        self.sub_ = self.node.create_subscription(String,
                                                  'obstacle_topic',
                                                  self.obstacle_callback,
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

    def obstacle_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if msg.data == "Undetected":
            self.result = True
            debug(f'[{current_time}] Undetected an obstacle')