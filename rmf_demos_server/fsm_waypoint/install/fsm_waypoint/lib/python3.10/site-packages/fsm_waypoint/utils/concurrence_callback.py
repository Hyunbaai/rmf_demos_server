
from fsm_waypoint.utils.logger2 import info

def child_termination_cb(outcome_map):
    # info(f"child_termination_cb map: {outcome_map}") INFO: Concurrent Outcomes: 대체
    # 둘중 하나만 만족해도 termination, 달리 말하면 이외 다른건 종료조건이 아님.
    return any(outcome in ['succeeded', 'bearing', 'waypoint', 'path', 'create', 'update', 'cancel', 'navfn'] for outcome in outcome_map.values())


def outcome_cb(outcome_map):
    info(f"outcome_cb map: {outcome_map}")
    # if any(outcome_map.get(key) == 'succeeded' for key in ['DETECT_OBJECT', 'PLANNER_SELECT']):
    #     return 'done'

    # 종료 상태를 그대로 리턴
    if any(outcome == 'bearing' for outcome in outcome_map.values()):
        return 'bearing'
    elif any(outcome == 'waypoint' for outcome in outcome_map.values()):
        return 'waypoint'
    elif any(outcome == 'path' for outcome in outcome_map.values()):
        return 'path'
    elif any(outcome == 'create' for outcome in outcome_map.values()):
        return 'create'
    elif any(outcome == 'update' for outcome in outcome_map.values()):
        return 'update'
    elif any(outcome == 'cancel' for outcome in outcome_map.values()):
        return 'cancel'
    elif any(outcome == 'navfn' for outcome in outcome_map.values()):
        return 'navfn'
    elif any(outcome == 'succeeded' for outcome in outcome_map.values()):
        return 'succeeded'
    elif any(outcome == 'preempted' for outcome in outcome_map.values()):
        return 'preempted'
    return 'aborted'
