# Description: This file contains the follow_gps_path task.
import smach
from fsm_waypoint.states.subscriber import CheckGlobalCostmap
from fsm_waypoint.states.publisher import SoundPlay, ChangePlannerSelect
from fsm_waypoint.states.navigator import FollowGpsWaypointsByGeopose
from fsm_waypoint.states.lib import LoadingWaypoints


def follow_gps_path(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                         input_keys=['blackboard'],
                                         output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('SOUND_PLAY_START', SoundPlay(node, 'follow_gps_path'),
                               transitions={'succeeded': 'CHANGE_STRAIGHTLINE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHANGE_STRAIGHTLINE', ChangePlannerSelect(node, 'StraightLine'),
                               transitions={'succeeded': 'CHECK_GLOBAL_COSTMAP', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_GLOBAL_COSTMAP', CheckGlobalCostmap(node, 5),
                               transitions={'succeeded': 'LOADING_WAYPOINTS', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('LOADING_WAYPOINTS', LoadingWaypoints(node),
                               transitions={'succeeded': 'FOLLOW_GPS_WAYPOINTS', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('FOLLOW_GPS_WAYPOINTS', FollowGpsWaypointsByGeopose(node),
                               transitions={'succeeded': 'SOUND_PLAY_DONE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_DONE', SoundPlay(node, 'follow_gps_path_completed'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm