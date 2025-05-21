# ros2
import rclpy

# fsm
import smach

# utils
import time
import math
from fsm_waypoint.utils import debug, info, warning, error, critical
import os

# supabase
from supabase import create_client, Client


class LoadingWaypoints(smach.State):
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

    def execute(self, userdata):
        start_time = time.time()
        outcome = 'aborted'

        try:
            robot_id = userdata.blackboard.robot_id
            path_id = userdata.blackboard.path_id
            waypoint_id = userdata.blackboard.waypoint_id

            if robot_id is None:
                error("Robot ID is missing in blackboard.")
                return 'aborted'
            if path_id is None:
                error("Path ID is missing in blackboard.")
                return 'aborted'

            # 1. Robot 테이블에서 robotId 필드와 일치하는 로봇의 id 값을 가져오기
            robot_response = self.supabase.table("Robot").select("id").eq("robotId", robot_id).execute()
            if not robot_response.data:
                error(f"Failed to fetch robot info for robotId {robot_id}")
                return 'aborted'
            # 해당하는 로봇의 id 값 (테이블의 기본 키)
            robot_table_id = robot_response.data[0]['id']

            # 2. 로봇의 id 값에 해당하는 Path 정보 가져오기
            path_response = self.supabase.table("Path").select("id").eq("robotId", robot_table_id).eq("id",
                                                                                                      path_id).execute()

            if not path_response.data:
                error(f"Failed to fetch path info for robot table ID {robot_table_id}")
                return 'aborted'
            # pathId 리스트에 저장
            path_ids = [path['id'] for path in path_response.data]

            # 각 pathId별로 waypoints를 저장할 딕셔너리
            all_waypoints_by_path = {}

            # 3. 각 pathId에 해당하는 Waypoints 가져오기
            for path_id in path_ids:
                waypoints_response = self.supabase.table("Waypoint").select("*").eq("pathId", path_id).execute()

                if not waypoints_response.data:
                    error(f"Failed to fetch waypoints for path ID {path_id}")
                    continue  # 실패 시 다음 경로로 넘어가기

                # 모든 waypoints 가져오기
                waypoints = waypoints_response.data

                # 4. waypoint_id가 -1이면 모든 웨이포인트 저장
                if waypoint_id == -1:
                    all_waypoints_by_path[path_id] = waypoints
                else:
                    # waypoint_id가 -1이 아닌 경우 해당 ID에 맞는 웨이포인트의 이름보다 큰 값만 필터링
                    start_name = None
                    for wp in waypoints:
                        if wp['id'] == waypoint_id:
                            start_name = int(wp['name'])
                            break

                    if start_name is not None:
                        # name 값이 start_name보다 큰 웨이포인트만 저장
                        filtered_waypoints = [wp for wp in waypoints if int(wp['name']) >= start_name]
                        all_waypoints_by_path[path_id] = filtered_waypoints
                    else:
                        error(f"No waypoint found with waypoint_id {waypoint_id} in path {path_id}.")
                        return 'aborted'

            # blackboard에 path별로 묶인 waypoints 저장
            userdata.blackboard.waypoints_by_path = all_waypoints_by_path
            info(f"Filtered waypoints by path: {all_waypoints_by_path}")

        except Exception as e:
            error(f"An error occurred: {str(e)}")
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