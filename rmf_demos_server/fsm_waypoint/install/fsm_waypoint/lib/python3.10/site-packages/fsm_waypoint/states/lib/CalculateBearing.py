# ros2
import rclpy
# fsm
import smach
# utils
import time
import math
from fsm_waypoint.utils import debug, info, warning, error, critical


def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    d_lon = lon2_rad - lon1_rad
    x = math.sin(d_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - (math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon))
    bearing = math.atan2(x, y)
    bearing_deg = math.degrees(bearing)
    if bearing_deg < 0:
        bearing_deg += 360
    return bearing_deg


class CalculateBearing(smach.State):
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
            time.sleep(0.1)
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    outcome = 'succeeded'
                    break
            if userdata.blackboard.initial_gps and userdata.blackboard.final_gps:
                initial_lat, initial_lon = userdata.blackboard.initial_gps
                final_lat, final_lon = userdata.blackboard.final_gps
                bearing = calculate_bearing(initial_lat, initial_lon, final_lat, final_lon)
                userdata.blackboard.bearing = bearing
                debug(f"Bearing from initial to final position: {userdata.blackboard.bearing} degrees")
                outcome = 'succeeded'
                break
        return outcome