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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary, RobotMode

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry

import numpy as np
from pyproj import Transformer

import socketio

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading
import requests
from rmf_demos_fleet_adapter.utils.logger2 import info, warning, error, debug

MODE_MAP = {
    0: "IDLE",
    1: "CHARGING",
    2: "MOVING",
    3: "PAUSED",
    4: "WAITING",
    5: "EMERGENCY",
    6: "GOING_HOME",
    7: "DOCKING",
    8: "ADAPTER_ERROR",
    9: "CLEANING",
}
app = FastAPI()


class Request(BaseModel):
    map_name: Optional[str] = None
    activity: Optional[str] = None
    label: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None, url: str = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        self.mode_teleop = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]
        self.url = url  # 로봇 URL

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


class FleetManager(Node):
    def __init__(self, config, nav_path):
        self.debug = False
        self.config = config
        self.fleet_name = self.config['rmf_fleet']['name']
        mgr_config = self.config['fleet_manager']

        self.gps = False
        self.offset = [0, 0]
        reference_coordinates_yaml = mgr_config.get('reference_coordinates')
        if reference_coordinates_yaml is not None:
            offset_yaml = reference_coordinates_yaml.get('offset')
            if offset_yaml is not None and len(offset_yaml) > 1:
                self.gps = True
                self.offset = offset_yaml

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.action_paths = {}  # Map activities to paths

        for robot_name, robot_config in self.config['rmf_fleet']['robots'].items():
            self.robots[robot_name] = State(url=robot_config.get("url", None))
        assert(len(self.robots) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible =\
            self.config['rmf_fleet']['reversible']

        fleet_manager_config = self.config['fleet_manager']
        self.action_paths = fleet_manager_config.get('action_paths', {})
        self.sio = socketio.Client()

        @self.sio.on("/gps")
        def message(data):
            try:
                robot = json.loads(data)
                robot_name = robot['robot_id']
                self.robots[robot_name].gps_to_xy(robot)
            except KeyError as e:
                self.get_logger().info(f"Malformed GPS Message!: {e}")

        if self.gps:
            while True:
                try:
                    self.sio.connect('http://0.0.0.0:8080')
                    break
                except Exception:
                    warning(
                        f"Trying to connect to sio server at"
                        f"http://0.0.0.0:8080..")
                    time.sleep(1)

        @app.get('/open-rmf/rmf_demos_fm/status/',
                 response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response['data']['all_robots'].append(
                        self.get_robot_state(state, robot_name))
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response['data'] = self.get_robot_state(state, robot_name)
            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_demos_fm/navigate/',
                  response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(dest.destination) < 1):
                return response

            robot = self.robots[robot_name]

            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']
            target_map = dest.map_name
            target_speed_limit = dest.speed_limit

            target_x -= self.offset[0]
            target_y -= self.offset[1]

            t = self.get_clock().now().to_msg()

            path_request = PathRequest()
            robot = self.robots[robot_name]
            cur_x = robot.state.location.x
            cur_y = robot.state.location.y
            cur_yaw = robot.state.location.yaw
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
                int(abs(abs(cur_yaw) - abs(target_yaw)) /
                    self.vehicle_traits.rotational.nominal_velocity)
            t.sec = t.sec + duration
            target_loc = Location()
            target_loc.t = t
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = target_map
            target_loc.obey_approach_speed_limit = False
            if target_speed_limit is not None and target_speed_limit > 0.0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = target_speed_limit

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            try:
                payload = {
                    "fleet_name": path_request.fleet_name,
                    "robot_name": path_request.robot_name,
                    "task_id": path_request.task_id,
                    "path": [
                        {
                            "x": loc.x,
                            "y": loc.y,
                            "yaw": loc.yaw,
                            "level_name": loc.level_name
                        } for loc in path_request.path
                    ]
                }
                if not robot.url:
                    error(f"No URL specified for robot {robot_name}.")
                    return {"success": False, "msg": "No URL specified of fleet manager"}
                post_response = requests.post(robot.url, json=payload)

                # debugging
                # info(f'/navigate: {json.dumps(payload, indent=2)}')
                simplified_info = {
                    "name": payload["robot_name"],
                    "task_id": payload["task_id"],
                    "path": [
                        {
                            "x": round(payload["path"][0]["x"], 2),
                            "y": round(payload["path"][0]["y"], 2),
                        } if len(payload["path"]) > 0 else {},
                        {
                            "x": round(payload["path"][1]["x"], 2),
                            "y": round(payload["path"][1]["y"], 2),
                        } if len(payload["path"]) > 1 else {},
                    ],
                }

                # 간소화된 정보 출력
                log_message = (
                    f"name: {simplified_info['name']}, "
                    f"task_id: {simplified_info['task_id']}"
                )

                # path 정보 추가
                for i, path_point in enumerate(simplified_info["path"]):
                    if path_point:  # path가 비어있지 않은 경우만 추가
                        log_message += f", path[{i}]: x={path_point['x']}, y={path_point['y']}"

                info(f'[nav] core -> server: {log_message}')

                if post_response.status_code == 200:
                    # debug(f"Successfully called pub_robot_path_requests for {robot_name}")
                    pass
                else:
                    error(
                        f"Failed to call pub_robot_path_requests for {robot_name}: {post_response.text}"
                    )
            except Exception as e:
                error(f"Failed to call pub_robot_path_requests for {robot_name}: {e}")
                response['msg'] = f"Error: {e}"
                return response

            if self.debug:
                print(f'Sending navigate request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = target_loc
            info(
                f'[nav] core -> server: robot.destination: x={robot.destination.x:.2f}, y={robot.destination.y:.2f}, yaw={robot.destination.yaw:.2f}')

            response['success'] = True
            return response

        @app.get('/open-rmf/rmf_demos_fm/stop_robot/',
                 response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            # Appending the current location twice will effectively tell the
            # robot to stop
            path_request.path.append(robot.state.location)
            path_request.path.append(robot.state.location)

            path_request.task_id = str(cmd_id)
            try:
                payload = {
                    "fleet_name": path_request.fleet_name,
                    "robot_name": path_request.robot_name,
                    "task_id": path_request.task_id,
                    "path": [
                        {
                            "x": loc.x,
                            "y": loc.y,
                            "yaw": loc.yaw,
                            "level_name": loc.level_name
                        } for loc in path_request.path
                    ]
                }
                if not robot.url:
                    error(f"No URL specified for robot {robot_name}.")
                    return {"success": False, "msg": "No URL specified of fleet manager"}
                post_response = requests.post(robot.url, json=payload)
                info(f'stop_robot: {json.dumps(payload, indent=2)}')

                if post_response.status_code == 200:
                    # info(f"Successfully called pub_robot_path_requests for {robot_name}")
                    pass
                else:
                    error(
                        f"Failed to call pub_robot_path_requests for {robot_name}: {post_response.text}"
                    )
            except Exception as e:
                error(f"Failed to call pub_robot_path_requests for {robot_name}: {e}")
                response['msg'] = f"Error: {e}"
                return response

            if self.debug:
                info(f'Sending stop request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            info(f'[stop] core -> server: robot.destination: {robot.destination}')
            robot.destination = None

            response['success'] = True
            return response

        @app.get('/open-rmf/rmf_demos_fm/action_paths/',
                 response_model=Response)
        async def action_paths(activity: str, label: str):
            response = {'success': False, 'msg': ''}
            if activity not in self.action_paths:
                return response

            if label not in self.action_paths[activity][label]:
                return response

            response['data'] = self.action_paths[activity][label]
            response['success'] = True
            info(f'[action paths] core -> server for [{activity}] at [{label}]')
            return response

        @app.post('/open-rmf/rmf_demos_fm/start_activity/',
                  response_model=Response)
        async def start_activity(
            robot_name: str,
            cmd_id: int,
            request: Request
        ):
            response = {'success': False, 'msg': ''}
            if (
                robot_name not in self.robots
                or request.activity not in self.action_paths
                or request.label not in self.action_paths[request.activity]
            ):
                return response

            robot = self.robots[robot_name]

            path_request = PathRequest()
            cur_loc = robot.state.location
            target_loc = Location()
            path_request.path.append(cur_loc)

            activity_path = self.action_paths[request.activity][request.label]
            map_name = activity_path['map_name']
            for wp in activity_path['path']:
                target_loc = Location()
                target_loc.x = wp[0]
                target_loc.y = wp[1]
                target_loc.yaw = wp[2]
                target_loc.level_name = map_name
                path_request.path.append(target_loc)

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            try:
                payload = {
                    "fleet_name": path_request.fleet_name,
                    "robot_name": path_request.robot_name,
                    "task_id": path_request.task_id,
                    "path": [
                        {
                            "x": loc.x,
                            "y": loc.y,
                            "yaw": loc.yaw,
                            "level_name": loc.level_name
                        } for loc in path_request.path
                    ]
                }
                if not robot.url:
                    error(f"No URL specified for robot {robot_name}.")
                    return {"success": False, "msg": "No URL specified of fleet manager"}
                post_response = requests.post(robot.url, json=payload)
                info(f'[start_activity] core -> server: {json.dumps(payload, indent=2)}')

                if post_response.status_code == 200:
                    # info(f"Successfully called pub_robot_path_requests for {robot_name}")
                    pass
                else:
                    error(
                        f"Failed to call pub_robot_path_requests for {robot_name}: {post_response.text}"
                    )
            except Exception as e:
                error(f"Failed to call pub_robot_path_requests for {robot_name}: {e}")
                response['msg'] = f"Error: {e}"
                return response

            if self.debug:
                info(
                    f'Sending [{request.activity}] at [{request.label}] '
                    f'request for {robot_name}: {cmd_id}'
                )
            robot.last_path_request = path_request
            robot.destination = target_loc
            info(f'start_activity: robot.destination: {robot.destination}')

            response['success'] = True
            response['data'] = {}
            response['data']['path'] = activity_path
            return response

        @app.post('/open-rmf/rmf_demos_fm/toggle_teleop/',
                  response_model=Response)
        async def toggle_teleop(robot_name: str, mode: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots):
                return response
            # Toggle action mode
            self.robots[robot_name].mode_teleop = mode.toggle
            response['success'] = True
            info(f'[toggle_teleop] core -> server for {robot_name}')
            return response

        @app.post('/open-rmf/rmf_demos_fm/sub_robot_state/', response_model=Response)
        async def sub_robot_state(data: dict):
            """
            Receives robot state data via REST API and processes it.
            """
            robot_name = data.get("name")
            if robot_name in self.robots:
                robot = self.robots[robot_name]
                # Process robot state data
                if not robot.is_expected_task_id(data["task_id"]) and not robot.mode_teleop:
                    if robot.last_path_request is not None:
                        if self.debug:
                            info(
                                f"Republishing task request for {robot_name}: "
                                f"{robot.last_path_request.task_id}, "
                                f"because it is currently following {data['task_id']}"
                            )
                        try:
                            payload = {
                                "fleet_name": self.fleet_name,
                                "robot_name": robot_name,
                                "task_id": robot.last_path_request.task_id,
                                "path": [
                                    {
                                        "x": loc.x,
                                        "y": loc.y,
                                        "yaw": loc.yaw,
                                        "level_name": loc.level_name
                                    } for loc in robot.last_path_request.path
                                ]
                            }
                            if not robot.url:
                                error(f"No URL specified for robot {robot_name}.")
                                return {"success": False, "msg": "No URL specified of fleet manager"}
                            response = requests.post(robot.url, json=payload)
                            # debugging
                            # info(f'/sub_robot_state: {json.dumps(payload, indent=2)}')\
                            simplified_info = {
                                "name": payload["robot_name"],
                                "task_id": payload["task_id"],
                                "path": [
                                    {
                                        "x": round(payload["path"][0]["x"], 2),
                                        "y": round(payload["path"][0]["y"], 2),
                                    } if len(payload["path"]) > 0 else {},
                                    {
                                        "x": round(payload["path"][1]["x"], 2),
                                        "y": round(payload["path"][1]["y"], 2),
                                    } if len(payload["path"]) > 1 else {},
                                ],
                            }
                            # 간소화된 정보 출력
                            log_message = (
                                f"name: {simplified_info['name']}, "
                                f"req_task_id: {simplified_info['task_id']}, "
                                f"current_task_id: {data['task_id']}"
                            )

                            # path 정보 추가
                            for i, path_point in enumerate(simplified_info["path"]):
                                if path_point:  # path가 비어있지 않은 경우만 추가
                                    log_message += f", path[{i}]: x={path_point['x']}, y={path_point['y']}"

                            info(f'[state] robot -> server: {log_message}')

                            if response.status_code == 200:
                                # info(f"Successfully called pub_robot_path_requests for {robot_name}")
                                pass
                            else:
                                error(
                                    f"Failed to call pub_robot_path_requests for {robot_name}: {response.text}"
                                )
                        except Exception as e:
                            error(f"Failed to call pub_robot_path_requests for {robot_name}: {e}")

                    return {"success": False, "msg": "Task ID mismatch"}
                # Convert received data to State object
                robot.state = RobotState(
                    location=Location(
                        x=data["location"]["x"],
                        y=data["location"]["y"],
                        yaw=data["location"]["yaw"],
                        level_name=data["location"]["level_name"]
                    ),
                    mode=RobotMode(mode=data["mode"]["mode"]),
                    battery_percent=data["battery_percent"]
                )
                # debugging
                # self.get_logger().info(f"Robot updated: {robot}")
                if robot.destination is None:
                    return {"success": True, "msg": "State updated without destination"}

                # Check if robot reached destination
                if (
                        (
                                data["mode"]["mode"] == RobotMode.MODE_IDLE
                                or data["mode"]["mode"] == RobotMode.MODE_CHARGING
                        )
                        and len(data["path"]) == 0
                ):
                    mode_text = MODE_MAP.get(data["mode"]["mode"], "UNKNOWN")

                    info(f"[state] robot -> server: from robot {robot_name} reached destination, mode: {mode_text}")
                    robot.destination = None
                    completed_request = int(data["task_id"])
                    if robot.last_completed_request != completed_request:
                        if self.debug:
                            print(
                                f"Detecting completed request for {robot_name}: {completed_request}"
                            )
                    robot.last_completed_request = completed_request
            return {"success": True, "msg": "Robot state processed"}

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        if self.gps:
            position = copy.deepcopy(robot.gps_pos)
        else:
            position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.yaw
        data['robot_name'] = robot_name
        data['map_name'] = robot.state.location.level_name
        data['position'] =\
            {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = robot.state.battery_percent
        if (robot.destination is not None
                and robot.last_path_request is not None):
            destination = robot.destination
            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target =\
                self.disp(position, [destination.x, destination.y])
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (dist_to_target /
                        self.vehicle_traits.linear.nominal_velocity +
                        ori_delta /
                        self.vehicle_traits.rotational.nominal_velocity)
            cmd_id = int(robot.last_path_request.task_id)
            data['destination_arrival'] = {
                'cmd_id': cmd_id,
                'duration': duration
            }
        else:
            data['destination_arrival'] = None

        data['last_completed_request'] = robot.last_completed_request
        if (
            robot.state.mode.mode == RobotMode.MODE_WAITING
            or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR
        ):
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data['replan'] = True
        else:
            data['replan'] = False

        return data

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    info(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config['fleet_manager']['ip'],
        port=config['fleet_manager']['port'],
        log_level='warning'
    )


if __name__ == '__main__':
    main(sys.argv)
