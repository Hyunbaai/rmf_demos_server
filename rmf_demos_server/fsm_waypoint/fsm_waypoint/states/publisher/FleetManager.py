#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import math
import yaml
import json
import time
import copy
import argparse
import requests
import datetime
import os

# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

# rmf
from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary, RobotMode
from rmf_task_msgs.msg import ApiRequest, ApiResponse

# fsm
import smach

# utils
from fsm_waypoint.utils import debug, info, warning, error, critical
from pyproj import Transformer
import threading
import uuid

# websocket
import websocket
import ssl

# YAML 파일 로드
def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Error: 파일 '{file_path}'이(가) 존재하지 않습니다.")

    with open(file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}  # 파일이 비어 있을 경우 빈 딕셔너리 반환


# YAML 데이터 파싱
def parse_vertices(file_path):
    try:
        parsed_data = load_yaml(file_path)
        vertices = parsed_data.get("levels", {}).get("L1", {}).get("vertices", [])

        if not vertices:
            raise ValueError("Error: 'vertices' 데이터가 존재하지 않습니다.")

        chargers = []  # 로봇 이름 (is_charger가 있는 경우)
        targets = []  # 로봇이 가야 할 위치 (is_charger가 없는 경우)

        for vertex in vertices:
            if not isinstance(vertex, list) or len(vertex) < 3:
                raise ValueError("Error: 'vertices' 데이터 형식이 잘못되었습니다.")

            x, y, attributes = vertex
            name = attributes.get("name", "")
            spawn_robot_name = attributes.get("spawn_robot_name", "")

            if "is_charger" in attributes and attributes["is_charger"]:
                chargers.append({"name": name, "spawn_robot_name": spawn_robot_name})  # 충전소 정보 저장
            elif name:
                targets.append(name)  # 로봇이 가야 할 목표 지점

        return chargers, targets

    except FileNotFoundError as e:
        print(e)
        return {}, []
    except ValueError as e:
        print(e)
        return {}, []
    except Exception as e:
        print(f"Unexpected error: {e}")
        return {}, []


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        self.mode_teleop = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = \
            self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True


# config_file:=/home/bcc/Works1/rmf_demos_robot/rmf_demos/config/turtlebot3_world/turtlebot3_world_config.yaml
class FleetManager(smach.State):
    def __init__(self, node, config, timeout=None):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.debug = False
        self.config = config
        self.node = node
        self.timeout = timeout
        self.fleet_name = self.config['rmf_fleet']['name']
        mgr_config = self.config['fleet_manager']
        self.yaml_file = self.config['rmf_fleet'].get('zero_path', 'default_path.yaml')
        self.task_ids = []  # 태스크 ID 리스트
        self.last_check_time = {}  # 각 태스크의 마지막 체크 시간

        self.url = 'wss://hp8idwb108.execute-api.ap-northeast-2.amazonaws.com/dev'
        self.auth = '66fc47ba2013b66a223b5c8a6fac0926'
        # self.connection_id = ''
        self.ws = None
        self.ws_thread = None
        self.connection_open = False  # WebSocket 연결 상태
        self.connection_closed = False  # 상태 전이를 위한 플래그
        self.sub_list = []  # 구독된 토픽들 목록
        self.robots = {}  # Map robot name to state
        self.headers = {}

        for robot_name, _ in self.config['rmf_fleet']['robots'].items():
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.node.create_publisher(ApiRequest, 'task_api_requests', transient_qos)
        self.sub = None
        self.pending_requests = {}

    def execute(self, userdata):
        self.sub_list = [] # reset
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.node.create_subscription(
            ApiResponse,
            'task_api_responses',
            self.receive_response,
            qos_profile=transient_qos)

        self.sub_list.append(self.sub)

        start_time = time.time()
        info("FleetManager state executing...")
        self.connection_open = False  # reset
        self.connection_closed = False  # reset
        self.task_ids = []  # 태스크 ID 리스트 초기화
        self.last_check_time = {}  # 마지막 체크 시간 딕셔너리 초기화

        if self.robots:
            robot_name = next(iter(self.robots))
            print(f"First robot_name: {robot_name}")
        else:
            raise ValueError("self.robots is empty, no robot_name available")

        chargers, targets = parse_vertices(self.yaml_file)
        print("로봇 이름 (충전소 있는 곳):", chargers)
        print("로봇이 가야 할 위치:", targets)

        # targets에 chargers의 name 정보 추가
        for charger in chargers:
            if charger.get("name") and charger["name"] not in targets:
                targets.append(charger["name"])

        self.headers = {
            "Auth": self.auth,
            "fleet_name": self.fleet_name,
            "chargers": json.dumps(chargers),
            "targets": json.dumps(targets),
        }
        # info(f"headers: {self.headers}")
        self.connect(self.headers)

        outcome = 'succeeded'

        try:
            while rclpy.ok():
                if self.preempt_requested():
                    self.service_preempt()
                    self.cleanup()
                    outcome = 'preempted'
                    break

                if self.connection_closed:
                    self.cleanup_only_wss()
                    self.connection_open = False
                    self.connection_closed = False
                    self.connect(self.headers)
                    time.sleep(1)

                # 태스크 상태 체크
                current_time = time.time()
                if self.connection_open:
                    # 5초마다 로봇 상태 체크
                    if current_time - self.last_check_time.get("robot_status", 0) >= 5:
                        self.check_robot_status(self.task_ids)
                        self.last_check_time["robot_status"] = current_time

                    # 태스크 상태 체크
                    if self.task_ids:
                        for task_id in self.task_ids[:]:  # 복사본으로 순회
                            if current_time - self.last_check_time.get(task_id, 0) >= 5:  # 5초마다 체크
                                if self.check_task_status(task_id):
                                    self.task_ids.remove(task_id)  # 완료된 태스크 제거
                                    self.last_check_time.pop(task_id, None)  # 체크 시간 기록 제거
                                else:
                                    self.last_check_time[task_id] = current_time  # 체크 시간 업데이트

                time.sleep(0.1)

                if self.timeout:
                    if time.time() - start_time > self.timeout:
                        outcome = 'succeeded'
                        break

        except KeyboardInterrupt:
            error("Interrupted by user, shutting down...")
            self.cleanup()
            return 'preempted'
        except Exception as e:
            error(f"An error occurred while creating waypoint: {str(e)}")
            self.cleanup()
            return 'aborted'

        self.cleanup()
        return outcome

    def connect(self, headers):
        self.ws = websocket.WebSocketApp(self.url,
                                         on_open=self.on_open,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close,
                                         header=headers)
        self.ws_thread = threading.Thread(target=self.ws.run_forever,
                                          kwargs={"sslopt": {"cert_reqs": ssl.CERT_NONE}})
        self.ws_thread.start()

    def on_open(self, ws):
        self.connection_open = True
        info("Connection opened")

    def on_message(self, ws, message):
        info('Received message: ' + message)
        try:
            package = json.loads(message)
            if package.get('type') == 'connected':
                args = package.get('args', {})
                connection_id = args.get('connection_id')
                # info(f'connection_id: {connection_id}')
                return  # 여기서 종료
            elif package.get('type') == 'request':
                request = package.get('args', {})

                if request.get("type") in ["robot_task_request", "cancel_task_request"]:
                    self.handle_api_request(request)
                elif request.get("type") == "iot_task_request":
                    self.handle_iot_request(request)
                
            else:
                error(f"Unknown message type: {package}")

        except json.JSONDecodeError:
            error("The received message is not in JSON format.")

    def on_error(self, ws, error):
        self.connection_open = False
        error(f"WebSocket error: {error}")
        self.connection_closed = True  # 상태 전이를 위한 플래그 설정

    def on_close(self, ws, close_status_code, close_msg):
        self.connection_open = False
        info(f"WebSocket connection closed, code: {close_status_code}, msg: {close_msg}")
        self.connection_closed = True  # 상태 전이를 위한 플래그 설정

    def cleanup(self):
        """구독 해제 및 WebSocket 종료"""
        info("Cleaning up sub_list and closing WebSocket...")
        for sub in self.sub_list:
            self.node.destroy_subscription(sub)  # 모든 구독 해제

        if self.ws:
            self.ws.close()
            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join()

    def cleanup_only_wss(self):
        if self.ws:
            self.ws.close()
            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join()

    def handle_iot_request(self, request):
        # request에서 필요한 정보 추출
        robot_name = request.get("robot_name", "unknown")
        fleet_name = request.get("fleet_name", "unknown")
        category = request.get("category", "unknown")
        value = request.get("value", "unknown")  # 목적지 place
        info(f"category: {category}, value: {value}")

        # robot_name에서 '/' 구분자가 있을 경우 마지막 부분만 사용
        if '/' in robot_name:
            robot_name = robot_name.split('/')[-1]

        # 로봇의 현재 상태 확인
        robot_statuses = self.check_robot_status()
        if not robot_statuses:
            error("로봇 상태 확인 실패")
            return

        # 요청된 로봇의 상태 확인
        if robot_name in robot_statuses:
            robot_status = robot_statuses[robot_name]
            current_place = robot_status.get("place", "unknown")
            
            # 현재 위치와 요청된 위치 비교
            is_at_target = False
            
            # category가 "door"인 경우 value에 여러 목적지가 있을 수 있음
            if category == "door":
                # value를 쉼표로 분리하여 목적지 목록 생성
                target_places = [place.strip() for place in value.split(',')]
                # 현재 위치가 목적지 목록 중 하나라도 일치하면 true
                if current_place in target_places:
                    is_at_target = True
                    info(f"로봇 {robot_name}이(가) 요청된 위치 중 하나인 {current_place}에 있습니다.")
            else:
                # 기존 로직: 단일 목적지와 비교
                if current_place == value:
                    is_at_target = True
                    info(f"로봇 {robot_name}이(가) 요청된 위치 {value}에 있습니다.")
            
            if is_at_target:
                # 충전소로 이동하는 태스크 생성
                request_id = f"patrol_{uuid.uuid4()}"  # 고유한 request_id 생성
                self.pending_requests[request_id] = request  # 요청 저장

                # 0.yaml에서 충전소 위치 찾기
                chargers, _ = parse_vertices(self.yaml_file)
                charger_place = None
                for charger in chargers:
                    if charger.get("spawn_robot_name") == robot_name:
                        charger_place = charger.get("name")
                        break

                # config에서 로봇 설정 찾기
                places = [charger_place]  # 기본값은 충전소 위치
                if robot_name in self.config['rmf_fleet']['robots']:
                    robot_config = self.config['rmf_fleet']['robots'][robot_name]
                    if 'recursive_paths' in robot_config:
                        places = robot_config['recursive_paths'][0]
                        info(f"로봇 {robot_name}의 recursive_paths를 사용합니다: {places}")

                if charger_place:
                    msg = ApiRequest()
                    msg.request_id = request_id
                    payload = {
                        "type": "robot_task_request",
                        "robot": robot_name,
                        "fleet": fleet_name,
                        "requester": robot_name,  # requester 필드 추가
                        "request": {
                            "category": "patrol",
                            "description": {
                                "places": places,  # recursive_path 또는 charger_place 사용
                                "rounds": 1
                            }
                        }
                    }
                    msg.json_msg = json.dumps(payload)
                    self.pub.publish(msg)
                    info(f"Published charge request: {msg.json_msg}")
                else:
                    error(f"로봇 {robot_name}의 충전소 위치를 찾을 수 없습니다.")
            else:
                if category == "door":
                    warning(f"로봇 {robot_name}의 현재 위치({current_place})가 요청된 위치({value}) 중 하나와 일치해야 합니다.")
                else:
                    warning(f"로봇 {robot_name}의 현재 위치({current_place})와 요청된 위치({value})가 같아야합니다.")
        else:
            error(f"로봇 {robot_name}의 상태 정보를 찾을 수 없습니다.")

    def handle_api_request(self, request):
        task_type = request.get("type", "default_robot_task_request")
        if task_type == "robot_task_request":
            request_id = f"patrol_{uuid.uuid4()}"  # 고유한 request_id 생성
            self.pending_requests[request_id] = request  # 요청 저장

            fleet_name = request.get("fleet_name", "default_turtlebot3")
            robot_name = request.get("robot_name", "default_tinybot1")
            # robot_name에서 '/' 구분자가 있을 경우 마지막 부분만 사용
            if '/' in robot_name:
                robot_name = robot_name.split('/')[-1]

            task_type = request.get("type", "default_robot_task_request")
            category = request.get("category", "default_patrol")
            # provider app에서 value protocol로 받은 목적지 정보 d1-2, d1-3, d1-4
            patrol_location = request.get("value", "default_patrol_A1")
            # patrol_location을 리스트로 변환
            if isinstance(patrol_location, str):
                patrol_location = patrol_location.split(',')

            # ['s1'] 그대로 사용 or ['s1', 'd1-1'] 그대로사용 or ['d1-1'] destination_paths 사용
            # patrol_location이 리스트이고 길이가 1보다 크면 그대로 사용
            if isinstance(patrol_location, list) and len(patrol_location) > 1:
                info(f"여러 목적지가 지정되었습니다: {patrol_location}")
            # patrol_location이 리스트가 아니거나 길이가 1이고, 'd'로 시작하는 경우
            elif isinstance(patrol_location, list) and len(patrol_location) == 1:
                # 로봇 설정에서 destination_paths 찾기
                if robot_name in self.config['rmf_fleet']['robots']:
                    robot_config = self.config['rmf_fleet']['robots'][robot_name]
                    if 'destination_paths' in robot_config:
                        # 각 경로의 마지막 위치와 place가 일치하는 인덱스 찾기
                        for i, path in enumerate(robot_config['destination_paths']):
                            if path and path[-1] == patrol_location[0]:  # 마지막 위치와 일치하는지 확인
                                patrol_location = path
                                info(f"로봇 {robot_name}의 destination_paths[{i}]를 사용합니다: {patrol_location}")
                                break

            msg = ApiRequest()
            msg.request_id = request_id
            payload = {
                "type": task_type,
                "robot": robot_name,
                "fleet": fleet_name,
                "requester": robot_name,
                "request": {
                    "category": category,
                    "description": {
                        "places": patrol_location,  # 이제 배열 형태
                        "rounds": 1  # 한 번 순찰
                    }
                }
            }
            msg.json_msg = json.dumps(payload)

            self.pub.publish(msg)
            info(f"Published patrol request: {msg.json_msg}")

        elif task_type == "cancel_task_request":
            msg = ApiRequest()
            msg.request_id = "cancel_task_" + str(uuid.uuid4())  # 고유한 취소 요청 ID 생성
            self.pending_requests[msg.request_id] = request  # 요청 저장
            payload = {
                "type": "cancel_task_request",
                "task_id": request.get("task_id", "default_task_id")  # task_id가 필요함
            }

            msg.json_msg = json.dumps(payload)
            # print(f"Published cancel task request: \n{json.dumps(payload, indent=2)}")
            self.pub.publish(msg)

        else:
            print(f"Unknown request type: {task_type}")

    def receive_response(self, response_msg: ApiResponse):
        request_id = response_msg.request_id
        response_data = json.loads(response_msg.json_msg)
        
        # requester가 admin인 경우 dashboard 호출로 처리
        if response_data.get("state", {}).get("booking", {}).get("requester") == "admin":
            warning(f"Dashboard 호출: {response_msg.json_msg}")
            return

        if request_id not in self.pending_requests:
            return

        original_request = self.pending_requests.pop(request_id)

        response_message = {
            "response_all": {
                "fleet_name": original_request.get("fleet_name", "default_fleet_001"),
                "robot_name": original_request.get("robot_name", "default_tinybot1"),
                "task_id": request_id,
                "type": original_request.get("type", "default_robot_task_request"),
                "value": original_request.get("value", "default_t1"),
                "category": original_request.get("category", "default_patrol"),
                "result": response_data
            }
        }

        if self.connection_open:
            self.ws.send(json.dumps(response_message))
            info(f"Sent response_amr")
        else:
            error("WebSocket is not connected. Unable to send response.")

        # robot_task_request라면 task_ids에 추가
        if original_request.get("type") == "robot_task_request":
            self.task_ids.append(request_id)
            self.last_check_time[request_id] = 0  # 초기 체크 시간 설정

    def check_task_status(self, task_id):
        BASE_URL = "http://localhost:8000"
        GET_URL = f"{BASE_URL}/tasks"
        Auth = "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJzdHViIiwicHJlZmVycmVkX3VzZXJuYW1lIjoiYWRtaW4iLCJpYXQiOjE1MTYyMzkwMjIsImF1ZCI6InJtZl9hcGlfc2VydmVyIiwiaXNzIjoic3R1YiIsImV4cCI6MjA1MTIyMjQwMH0.zzX3zXp467ldkzmLVIadQ_AHr8M5uWVV43n4wEB0OhE"
        HEADERS = {
            "Content-Type": "application/json",
            "Authorization": Auth
        }
        try:
            get_response = requests.get(GET_URL, headers=HEADERS)

            if get_response.status_code != 200:
                error(f"GET 요청 실패: {get_response.status_code}")
                return False

            tasks = get_response.json()
            for task in tasks:
                if task["booking"]["id"] == task_id:
                    info(f"task: {task}")
                    status = task["status"]
                    start_time = task.get("unix_millis_start_time", None)
                    finish_time = task.get("unix_millis_finish_time", None)
                    phases = task.get("phases", {})

                    # 추가 정보 추출
                    original_estimate = task.get("original_estimate_millis", 0) / 1000  # 밀리초를 초로 변환
                    current_estimate = task.get("estimate_millis", 0) / 1000  # 밀리초를 초로 변환
                    assigned_to = task.get("assigned_to", {})
                    fleet_name = assigned_to.get("group", "unknown")
                    robot_name = assigned_to.get("name", "unknown")
                    
                    # place 정보 추출 (현재 진행 중인 페이즈의 category에서 추출)
                    current_place = "unknown"
                    if status == "completed":
                        # completed 상태일 때는 마지막으로 완료된 페이즈의 place를 사용
                        completed_phases = task.get("completed", [])
                        if completed_phases:
                            last_phase_id = str(completed_phases[-1])
                            last_phase = phases.get(last_phase_id, {})
                            category = last_phase.get("category", "")
                            if "Go to [place:" in category:
                                current_place = category.split("[place:")[1].split("]")[0]
                    else:
                        # 진행 중인 페이즈의 place를 찾음
                        for phase_id, phase in phases.items():
                            if phase.get("unix_millis_start_time") and not phase.get("unix_millis_finish_time"):
                                category = phase.get("category", "")
                                if "Go to [place:" in category:
                                    current_place = category.split("[place:")[1].split("]")[0]
                                    break

                    # 전체 경과 시간 계산
                    if start_time and finish_time:
                        elapsed_time = (finish_time - start_time) / 1000  # 밀리초를 초로 변환
                        info(f"전체 경과 시간: {elapsed_time:.2f}초")
                    else:
                        current_time = int(time.time() * 1000)  # 현재 시간을 밀리초로 변환
                        elapsed_time = (current_time - start_time) / 1000
                        info(f"현재까지 경과 시간: {elapsed_time:.2f}초")

                    # 각 페이즈별 경과 시간 계산
                    for phase_id, phase in phases.items():
                        phase_start = phase.get("unix_millis_start_time")
                        phase_finish = phase.get("unix_millis_finish_time")
                        
                        # events 객체가 None이 아닌 경우에만 상태 확인
                        events = phase.get("events", {})
                        if events:
                            phase_status = events.get("0", {}).get("status", "unknown")
                        else:
                            phase_status = "unknown"
                        
                        if phase_start:
                            if phase_finish:
                                phase_elapsed = (phase_finish - phase_start) / 1000
                                info(f"페이즈 {phase_id} 경과 시간: {phase_elapsed:.2f}초 (상태: {phase_status})")
                            else:
                                current_time = int(time.time() * 1000)
                                phase_elapsed = (current_time - phase_start) / 1000
                                info(f"페이즈 {phase_id} 현재까지 경과 시간: {phase_elapsed:.2f}초 (상태: {phase_status})")

                    start_time_fmt = datetime.datetime.fromtimestamp(start_time / 1000).strftime(
                        '%Y-%m-%d %H:%M:%S') if start_time else "N/A"
                    finish_time_fmt = datetime.datetime.fromtimestamp(finish_time / 1000).strftime(
                        '%Y-%m-%d %H:%M:%S') if finish_time else "N/A"

                    info(f"현재 상태: {status}")
                    if status == "underway":
                        info(f"로봇이 이동 중... 시작 시간: {start_time_fmt}")
                    elif status == "completed":
                        info(f"로봇이 도착 완료! 도착 시간: {finish_time_fmt}")

                    # WebSocket을 통해 상태 전송
                    response_message = {
                        "response_all": {
                            "task_id": task_id,
                            "status": status,
                            "start_time": start_time_fmt,
                            "finish_time": finish_time_fmt,
                            "elapsed_time": f"{elapsed_time:.2f}초" if 'elapsed_time' in locals() else "N/A",
                            "fleet_name": fleet_name,
                            "robot_name": robot_name,
                            "original_estimate": f"{original_estimate:.2f}초",
                            "current_estimate": f"{current_estimate:.2f}초",
                            "place": current_place  # 현재 진행 중인 place 정보
                        }
                    }

                    if self.connection_open:
                        self.ws.send(json.dumps(response_message))
                        info(f"Sent response: {response_message}")
                    else:
                        error("WebSocket is not connected. Unable to send response.")

                    # 태스크 완료 또는 취소 시 True 반환
                    if status in ["completed", "canceled"]:
                        return True

            return False

        except Exception as e:
            error(f"오류 발생: {e}")
            return False
    def check_robot_status(self, task_ids=None):
        BASE_URL = "http://localhost:8000"
        GET_URL = f"{BASE_URL}/tasks"
        Auth = "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJzdHViIiwicHJlZmVycmVkX3VzZXJuYW1lIjoiYWRtaW4iLCJpYXQiOjE1MTYyMzkwMjIsImF1ZCI6InJtZl9hcGlfc2VydmVyIiwiaXNzIjoic3R1YiIsImV4cCI6MjA1MTIyMjQwMH0.zzX3zXp467ldkzmLVIadQ_AHr8M5uWVV43n4wEB0OhE"
        HEADERS = {
            "Content-Type": "application/json",
            "Authorization": Auth
        }
        try:
            get_response = requests.get(GET_URL, headers=HEADERS)

            if get_response.status_code != 200:
                error(f"GET 요청 실패: {get_response.status_code}")
                return None

            tasks = get_response.json()
            robot_tasks = {}  # 로봇별 모든 태스크를 임시로 저장

            # 모든 태스크를 로봇별로 분류
            for task in tasks:
                assigned_to = task.get("assigned_to", {})
                if not assigned_to:  # assigned_to 정보가 없는 경우 건너뛰기
                    continue
                    
                robot_name = assigned_to.get("name", "unknown")
                if robot_name == "unknown":  # robot_name이 unknown인 경우 건너뛰기
                    continue
                
                # robot_name에서 '/' 구분자가 있을 경우 마지막 부분만 사용
                if '/' in robot_name:
                    robot_name = robot_name.split('/')[-1]

                if robot_name not in robot_tasks:
                    robot_tasks[robot_name] = []
                robot_tasks[robot_name].append(task)

            robot_statuses = {}
            # 각 로봇별로 가장 최근 태스크 선택
            for robot_name, tasks in robot_tasks.items():
                if not tasks:  # 태스크가 없는 경우 건너뛰기
                    continue
                    
                # 시작 시간을 기준으로 정렬 (최신순)
                sorted_tasks = sorted(tasks, 
                                   key=lambda x: x.get("unix_millis_start_time", 0), 
                                   reverse=True)
                
                latest_task = sorted_tasks[0]
                status = latest_task.get("status", "unknown")
                fleet_name = latest_task.get("assigned_to", {}).get("group", "unknown")
                
                # place 정보 추출
                current_place = "unknown"
                phases = latest_task.get("phases", {})
                if status == "completed":
                    completed_phases = latest_task.get("completed", [])
                    if completed_phases:
                        last_phase_id = str(completed_phases[-1])
                        last_phase = phases.get(last_phase_id, {})
                        category = last_phase.get("category", "")
                        if "Go to [place:" in category:
                            current_place = category.split("[place:")[1].split("]")[0]
                else:
                    for phase_id, phase in phases.items():
                        if phase.get("unix_millis_start_time") and not phase.get("unix_millis_finish_time"):
                            category = phase.get("category", "")
                            if "Go to [place:" in category:
                                current_place = category.split("[place:")[1].split("]")[0]
                                break

                # 로봇 상태 정보 저장
                start_time = latest_task.get("unix_millis_start_time", None)
                finish_time = latest_task.get("unix_millis_finish_time", None)
                current_time = int(time.time() * 1000)  # 현재 시간을 밀리초로 변환

                # 경과 시간 계산
                if start_time and finish_time:
                    elapsed_time = (finish_time - start_time) / 1000  # 밀리초를 초로 변환
                else:
                    elapsed_time = (current_time - start_time) / 1000 if start_time else 0

                robot_statuses[robot_name] = {
                    "status": status,
                    "fleet_name": fleet_name,
                    "robot_name": robot_name,
                    "place": current_place,
                    "task_id": latest_task.get("booking", {}).get("id", "unknown"),
                    "start_time": datetime.datetime.fromtimestamp(start_time / 1000).strftime('%Y-%m-%d %H:%M:%S') if start_time else "N/A",
                    "finish_time": datetime.datetime.fromtimestamp(finish_time / 1000).strftime('%Y-%m-%d %H:%M:%S') if finish_time else "N/A",
                    "elapsed_time": f"{elapsed_time:.2f}초" if elapsed_time > 0 else "N/A"
                }

            # task_ids가 비어있을 때만 WebSocket 메시지 전송
            if not self.task_ids and self.connection_open:
                for robot_name, status in robot_statuses.items():
                    response_message = {
                        "response_all": status
                    }
                    self.ws.send(json.dumps(response_message))
                    debug(f"Sent robot status for {robot_name}: {response_message}")

            return robot_statuses

        except Exception as e:
            error(f"로봇 상태 확인 중 오류 발생: {e}")
            return None


# 수정후
# export CONFIG_FILE=/home/zeta/.../gl_deliveryRobot_config.yaml && colcon build --packages-select fsm_waypoint && export ROS_DOMAIN_ID=? && ros2 run fsm_waypoint fsm_waypoint_node