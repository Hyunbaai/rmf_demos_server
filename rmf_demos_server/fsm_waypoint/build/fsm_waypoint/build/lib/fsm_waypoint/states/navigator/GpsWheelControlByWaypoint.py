# ros2
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical
from geographiclib.geodesic import Geodesic
import uuid


class GpsWheelControlByWaypoint(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        unique_name = f"fsm_gps_wheel_control_navigator_{uuid.uuid4().hex}"
        self.navigator = BasicNavigator(unique_name)
        self.geod = Geodesic.WGS84
        self.timeout = timeout
        self.node = node

    # our robot's base heading is East (0 degrees)
    def execute(self, userdata):
        userdata.blackboard.wait_spin = True
        time.sleep(0.1)
        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        if not userdata.blackboard.current_waypoints:
            error("The current_waypoint array is empty.")
        self.navigator.followGpsWaypoints(userdata.blackboard.current_waypoints)
        start_time = time.time()
        outcome = 'aborted'
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                self.navigator.cancelTask()
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
                    outcome = 'succeeded'
                    break
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                debug(f'Navigator result: {result}')
                outcome = 'succeeded'
                break
        userdata.blackboard.wait_spin = False
        return outcome