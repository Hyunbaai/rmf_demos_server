import smach

def init_blackboard():
    blackboard = smach.UserData()
    blackboard.initial_gps = (0, 0)
    blackboard.final_gps = (0, 0)
    blackboard.bearing = 0.0
    blackboard.heading = 0.0
    blackboard.wait_spin = False
    blackboard.direction = ''
    blackboard.distance = 0.0
    blackboard.robot_id = None
    blackboard.path_id = None
    blackboard.waypoint_id = None
    blackboard.path_type = None
    blackboard.bottom_led = None
    blackboard.top_led = None
    blackboard.lighting = None
    blackboard.fan = None
    blackboard.door = None
    blackboard.current_waypoints = []
    blackboard.current_waypoint_index = 0
    blackboard.obstacle_timeout = False
    blackboard.ongoing_request_cmd_id = None
    blackboard.path = []
    blackboard.last_request_completed = None
    return blackboard
