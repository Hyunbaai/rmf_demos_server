# ros2
import rclpy
from sensor_msgs.msg import NavSatFix

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class GetGpsFiltered(smach.State):
    def __init__(self, node, type, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None
        self.gps_value = None
        self.type = type

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(NavSatFix,
                                                  '/gps/filtered',
                                                  self.gps_callback,
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
                if self.type == 'init':
                    userdata.blackboard.initial_gps = self.gps_value
                    debug(f'Initial GPS data: {userdata.blackboard.initial_gps}')
                elif self.type == 'final':
                    userdata.blackboard.final_gps = self.gps_value
                    debug(f'Final GPS data: {userdata.blackboard.final_gps}')
                outcome = 'succeeded'
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def gps_callback(self, msg):
        self.gps_value = (msg.latitude, msg.longitude)
        self.result = True