# ros2
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

# fsm
import smach

# utils
import time
import math
from fsm_waypoint.utils import debug, info, warning, error, critical
from fsm_waypoint.utils import latLonYaw2Geopose
import uuid


class GoToPose(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        unique_name = f"fsm_go_to_pose_navigator_{uuid.uuid4().hex}"
        self.navigator = BasicNavigator(unique_name)
        self.timeout = timeout
        self.node = node

    def execute(self, userdata):
        userdata.blackboard.wait_spin = True
        time.sleep(0.1)
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        lat, lon = userdata.blackboard.final_gps  # rotate north in place
        # debugging
        # lat = 37.6078214
        # lon = 126.891570
        heading_radian = math.pi / 2  # North fixed
        wp = [latLonYaw2Geopose(lat, lon, heading_radian)]
        debug(f'waypoints: {lat}, {lon}, {heading_radian}')
        self.navigator.followGpsWaypoints(wp)
        start_time = time.time()
        outcome = 'aborted'
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                self.navigator.cancelTask()
                error('preempted: navigator.cancelTask()')
                outcome = 'preempted'
                break
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
                    self.navigator.cancelTask()
                    error('timeout: navigator.cancelTask()')
                    outcome = 'succeeded'
                    break
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                debug(f'Navigator result: {result}')
                outcome = 'succeeded'
                break
        userdata.blackboard.wait_spin = False
        return outcome