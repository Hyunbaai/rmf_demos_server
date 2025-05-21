# ros2
import rclpy

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class RunSpinOnce(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node

    def execute(self, userdata):
        start_time = time.time()
        outcome = 'aborted'
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break
            if userdata.blackboard.wait_spin:
                time.sleep(0.1)
            else:
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except IndexError as e:
                    error(f"IndexError: {e}")
                    time.sleep(0.1)
                except rclpy._rclpy_pybind11.InvalidHandle:
                    error('Node handle is invalid.')
                    time.sleep(0.1)
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    outcome = 'succeeded'
                    break
        return outcome