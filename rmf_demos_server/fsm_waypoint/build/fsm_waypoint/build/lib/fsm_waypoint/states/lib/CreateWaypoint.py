# ros2
import rclpy

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical
import os
import random

# supabase
from supabase import create_client, Client


class CreateWaypoint(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node
        url: str = os.environ.get("SUPABASE_URL")
        key: str = os.environ.get("SUPABASE_KEY")
        self.supabase: Client = create_client(url, key)

    def generate_random_color(self):
        return "#{:06x}".format(random.randint(0, 0xFFFFFF))

    def execute(self, userdata):
        start_time = time.time()
        outcome = 'aborted'

        try:
            path_id = userdata.blackboard.path_id
            latitude, longitude = userdata.blackboard.initial_gps
            yaw = userdata.blackboard.heading

            if not all([path_id, latitude, longitude, yaw]):
                error("Missing required fields in blackboard.")
                return 'aborted'

            # 1. Path 테이블에서 path_id가 있는지 확인
            path_response = self.supabase.table("Path").select("id").eq("id", path_id).execute()

            if not path_response.data:
                error(f"Path ID {path_id} does not exist in the database.")
                return 'aborted'

            # 2. 해당 pathId에 속한 Waypoint들의 이름(name) 필드를 가져오기
            waypoints_response = self.supabase.table("Waypoint").select("name").eq("pathId", path_id).execute()

            if not waypoints_response.data:
                info(f"No waypoints found for path ID {path_id}, starting with name 1.")
                next_name = 1
            else:
                # 3. 이미 존재하는 waypoints 중 가장 큰 name을 찾고, 그 다음 순번으로 새로운 name을 설정
                waypoint_names = [int(wp['name']) for wp in waypoints_response.data if wp['name'].isdigit()]
                if waypoint_names:
                    next_name = max(waypoint_names) + 1
                else:
                    next_name = 1  # 기존 웨이포인트가 없으면 1부터 시작

            random_color = self.generate_random_color()
            # 4. 새로운 Waypoint 생성
            new_waypoint = {
                'latitude': latitude,
                'longitude': longitude,
                'yaw': yaw,
                'pathId': path_id,
                'name': str(next_name),  # 새로 계산한 name 값
                'color': random_color,
            }

            waypoint_insert_response = self.supabase.table("Waypoint").insert(new_waypoint).execute()
            if not waypoint_insert_response:
                error(f"Failed to create new waypoint in path {path_id}")
                return 'aborted'

            info(f"New waypoint created successfully with name {new_waypoint['name']} in path {path_id}")

        except Exception as e:
            error(f"An error occurred while creating waypoint: {str(e)}")
            return 'aborted'

        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    outcome = 'succeeded'
                    break
            else:
                outcome = 'succeeded'
                break
        return outcome