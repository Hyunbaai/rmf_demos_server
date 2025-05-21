# ros2
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import String
from rclpy.qos import QoSProfile

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical
import uuid


class FollowGpsWaypointsBySnapshot(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        unique_name = f"fsm_follow_gps_waypoint_navigator_{uuid.uuid4().hex}"
        self.navigator = BasicNavigator(unique_name)
        self.timeout = timeout
        self.node = node
        self.previous_waypoint_index = None  # 이전 웨이포인트를 추적하기 위한 변수
        self.pub_ = self.node.create_publisher(String,
                                               '/sound_file',
                                               QoSProfile(depth=10))

    # our robot's base heading is East (0 degrees)
    def execute(self, userdata):
        userdata.blackboard.wait_spin = True
        time.sleep(0.1)
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        if not userdata.blackboard.current_waypoints:
            error("The current_waypoint array is empty.")
        start_index = userdata.blackboard.current_waypoint_index
        waypoints_to_follow = userdata.blackboard.current_waypoints[start_index:]
        self.navigator.followGpsWaypoints(waypoints_to_follow)
        current_waypoint_index = 0
        start_time = time.time()
        outcome = 'aborted'

        self.previous_waypoint_index = None  # reset

        # 전체 웨이포인트 정보 가져오기 (waypoints_by_path)
        waypoints_by_path = userdata.blackboard.waypoints_by_path
        path_id = userdata.blackboard.path_id
        waypoints = waypoints_by_path.get(path_id, [])
        if not waypoints:
            error("The waypoints array is empty.")
            return 'aborted'

        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                self.navigator.cancelTask()
                userdata.blackboard.current_waypoint_index = userdata.blackboard.current_waypoint_index + current_waypoint_index
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
            else:
                feedback = self.navigator.getFeedback()
                if feedback:
                    current_waypoint_index = feedback.current_waypoint
                    start_index = userdata.blackboard.current_waypoint_index
                    wp_type = waypoints[start_index + current_waypoint_index]["type"]

                    if current_waypoint_index != self.previous_waypoint_index:
                        debug(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(waypoints_to_follow)}')
                        info(f'type: {wp_type}')
                        info(f'current_waypoint_index: {current_waypoint_index}')
                        info(f'waypoint: {waypoints[start_index + current_waypoint_index]}')
                        self.previous_waypoint_index = current_waypoint_index  # 이전 웨이포인트 업데이트

                        if current_waypoint_index:
                            file = '/root/scripts/sound/sounds/voice/next_waypoint.mp3'
                            if wp_type == 1:
                                file = '/root/scripts/sound/sounds/voice/crowded.mp3'
                            elif wp_type == 2:
                                file = '/root/scripts/sound/sounds/voice/crosswalk.mp3'
                            elif wp_type == 3:
                                file = '/root/scripts/sound/sounds/voice/caution.mp3'

                            msg = String()
                            msg.data = file
                            self.pub_.publish(msg)
                            debug(f'Sent file: {msg.data}')

        userdata.blackboard.wait_spin = False
        return outcome