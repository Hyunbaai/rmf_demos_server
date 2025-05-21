# ros2
import rclpy
# fsm
import smach
# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


# supabase
from supabase import create_client, Client

class PreemptedTimeout(smach.State):
    def __init__(self, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.timeout = timeout

    def execute(self, ud):
        start_time = time.time()
        outcome = 'aborted'
        while True:
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break
            time.sleep(0.1)
            if time.time() - start_time > self.timeout:
                outcome = 'succeeded'
                break
            ud.blackboard.timeout = time.time() - start_time
        return outcome