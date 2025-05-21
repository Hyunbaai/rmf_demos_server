# ros2
import rclpy
from std_msgs.msg import String

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical
import json


class GetWssCommand(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self,
                             outcomes=['bearing', 'waypoint', 'path', 'create', 'update', 'succeeded', 'preempted',
                                       'aborted', 'cancel'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None
        self.direction = ''
        self.distance = 0.0
        self.robot_id = 0
        self.path_id = 0
        self.waypoint_id = 0
        self.path_type = 0

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(String,
                                                  'wss_command',
                                                  self.wss_command_callback,
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
                info(f'Next State: {self.result}')
                outcome = self.result
                userdata.blackboard.direction = self.direction
                userdata.blackboard.distance = self.distance
                userdata.blackboard.robot_id = self.robot_id
                userdata.blackboard.path_id = self.path_id
                userdata.blackboard.waypoint_id = self.waypoint_id
                userdata.blackboard.path_type = self.path_type
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def wss_command_callback(self, msg: String):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        if msg.data:
            info(f'[{current_time}] Received data: {msg.data}')
            try:
                data = json.loads(msg.data)  # JSON 형식으로 데이터 파싱
                # Forward, Backward, Left, Right 키가 있는지 확인
                directions = ['Forward', 'Backward', 'Left90', 'Right90', 'Left45', 'Right45', 'North', 'South', 'East',
                              'West']
                direction_found = next((key for key in directions if key in data), None)

                waypoint = ['robot_id', 'path_id', 'waypoint_id', 'type']
                waypoint_found = next((key for key in waypoint if key in data), None)

                if direction_found:
                    self.direction = direction_found.lower()  # 방향 저장
                    self.distance = data[direction_found]  # 해당 방향의 거리 저장
                    self.result = 'waypoint'
                elif 'initialize_bearing' in data:
                    self.result = 'bearing'
                elif 'Cancel' in data:
                    self.result = 'cancel'
                elif waypoint_found:
                    self.robot_id = data['robot_id']
                    self.path_id = data['path_id']
                    self.waypoint_id = data['waypoint_id']
                    self.path_type = data['type']
                    if self.path_type < 0:  # Cancel
                        self.result = 'cancel'
                    elif self.path_type == 1300:  # Add New Waypoint
                        self.result = 'create'
                    elif self.path_type == 2300:  # Update Waypoint
                        self.result = 'update'
                    else:
                        self.result = 'path'  # waypoint_id < 0 Path Following else Waypoint Following
            except json.JSONDecodeError:
                info(f'[{current_time}] Failed to decode JSON data')
        else:
            info(f'[{current_time}] Received empty data array. No Event available.')