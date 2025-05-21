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
from geographiclib.geodesic import Geodesic
import uuid


class GpsWheelControlByCalculation(smach.State):
    def __init__(self, node, _direction='', _distance=0, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        unique_name = f"fsm_gps_wheel_control_navigator_{uuid.uuid4().hex}"
        self.navigator = BasicNavigator(unique_name)
        self.geod = Geodesic.WGS84
        self.timeout = timeout
        self.node = node
        self.direction = _direction
        self.distance = _distance

    # our robot's base heading is East (0 degrees)
    def execute(self, userdata):
        userdata.blackboard.wait_spin = True
        time.sleep(0.1)
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        initial_lat, initial_lon = userdata.blackboard.initial_gps

        # 현제 로봇의 방향을 구하고 그 방향을 기준으로 90왼쪽으로 1M의 위치를 계산해서 new 위경도를 구하고
        heading_radian = userdata.blackboard.heading  # ex: 0 radian
        debug(f'Current robot Heading : {math.degrees(heading_radian)} degrees')
        # new_bearing_radian = heading_radian + (math.pi / 2) # ex: 1.5708 radian  left 90 degree

        direction = self.direction if self.direction != '' else userdata.blackboard.direction
        distance = self.distance if self.direction != '' else userdata.blackboard.distance

        if int(distance) == 0:
            if direction == 'north':
                new_bearing_degree = 90
            elif direction == 'east':
                new_bearing_degree = 0
            elif direction == 'south':
                new_bearing_degree = 270
            elif direction == 'west':
                new_bearing_degree = 180
            else:
                error(f"Invalid direction: {direction}")
                return 'aborted'
            heading_radian = math.radians(new_bearing_degree)
            wp = [latLonYaw2Geopose(initial_lat, initial_lon, heading_radian)]  # 회전한 방향에 맞게만 변경
            debug(f'waypoints: {initial_lat}, {initial_lon}, {new_bearing_degree}')
        else:
            if direction == 'left90':
                new_bearing_radian = heading_radian + (math.pi / 2)  # 왼쪽으로 90도 회전
            elif direction == 'right90':
                new_bearing_radian = heading_radian - (math.pi / 2)  # 오른쪽으로 90도 회전
            elif direction == 'left45':
                new_bearing_radian = heading_radian + (math.pi / 4)  # 왼쪽으로 45도 회전
            elif direction == 'right45':
                new_bearing_radian = heading_radian - (math.pi / 4)  # 오른쪽으로 45도 회전
            elif direction == 'forward':
                new_bearing_radian = heading_radian  # 앞으로는 현 방향 그대로
            elif direction == 'backward':
                new_bearing_radian = heading_radian + math.pi  # 뒤로는 180도 회전
            else:
                error(f"Invalid direction: {direction}")
                return 'aborted'

            bearing_degrees = math.degrees(new_bearing_radian)

            # original bearing to robot bearing => robot 기준으로 위치 설정
            wp_heading_degrees = (90 - bearing_degrees) % 360
            result = self.geod.Direct(initial_lat, initial_lon, wp_heading_degrees, distance)
            new_lat, new_lon = result['lat2'], result['lon2']

            # robot bearing to original bearing => 진행 방향 그대로
            new_wp_heading_degrees = (90 - wp_heading_degrees) % 360
            heading_radian = math.radians(new_wp_heading_degrees)
            wp = [latLonYaw2Geopose(new_lat, new_lon, heading_radian)]
            debug(f'waypoints: {new_lat}, {new_lon}, {new_wp_heading_degrees}')
        if self.direction == '':
            userdata.blackboard.current_waypoints = wp
        self.navigator.followGpsWaypoints(wp)
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