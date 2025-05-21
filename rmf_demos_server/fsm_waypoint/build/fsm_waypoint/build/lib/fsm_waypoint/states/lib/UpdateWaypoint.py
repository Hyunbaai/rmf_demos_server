# ros2
import rclpy

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical
import os

# supabase
from supabase import create_client, Client


class UpdateWaypoint(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.timeout = timeout
        self.node = node
        url: str = os.environ.get("SUPABASE_URL")
        key: str = os.environ.get("SUPABASE_KEY")
        self.supabase: Client = create_client(url, key)

    def execute(self, userdata):
        start_time = time.time()
        outcome = 'aborted'

        try:
            waypoint_id = userdata.blackboard.waypoint_id
            path_id = userdata.blackboard.path_id
            latitude, longitude = userdata.blackboard.initial_gps
            yaw = userdata.blackboard.heading

            if not all([waypoint_id, latitude, longitude, yaw]):
                error("Missing required fields in blackboard.")
                return 'aborted'

            # 1. 주어진 pathId 및 waypointId에 해당하는 Waypoint가 있는지 확인
            waypoint_response = self.supabase.table("Waypoint").select("*").eq("pathId", path_id).eq("id", waypoint_id).execute()

            if not waypoint_response.data:
                error(f"Failed to fetch waypoint info for path ID {path_id} and waypoint ID {waypoint_id}")
                return 'aborted'

            # 2. Waypoint 업데이트 (latitude, longitude, yaw)
            update_data = {
                'latitude': latitude,
                'longitude': longitude,
                'yaw': yaw
            }

            update_response = self.supabase.table("Waypoint").update(update_data).eq("id", waypoint_id).execute()
            if not update_response:
                error(f"Failed to update waypoint {waypoint_id}")
                return 'aborted'

            info(f"Waypoint {waypoint_id} updated successfully with new latitude: {latitude}, longitude: {longitude}, yaw: {yaw}")

        except Exception as e:
            error(f"An error occurred while updating waypoint: {str(e)}")
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