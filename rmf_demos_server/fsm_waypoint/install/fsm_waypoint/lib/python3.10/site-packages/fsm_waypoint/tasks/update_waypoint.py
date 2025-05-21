# Description: This file contains the state machine for updating the waypoint of the robot.
import smach
from fsm_waypoint.states.publisher import SoundPlay
from fsm_waypoint.states.subscriber import CheckGlobalCostmap, GetGpsFiltered, GetRobotHeading
from fsm_waypoint.states.lib import UpdateWaypoint


def update_waypoint(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                         input_keys=['blackboard'],
                                         output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('SOUND_PLAY_START', SoundPlay(node, 'update_waypoint'),
                               transitions={'succeeded': 'CHECK_GLOBAL_COSTMAP', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_GLOBAL_COSTMAP', CheckGlobalCostmap(node, 5),
                               transitions={'succeeded': 'GET_INITIAL_GPS_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_INITIAL_GPS_DATA', GetGpsFiltered(node, 'init'),
                               transitions={'succeeded': 'GET_HEADING_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_HEADING_DATA', GetRobotHeading(node),
                               transitions={'succeeded': 'UPDATE_WAYPOINT_WORK', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('UPDATE_WAYPOINT_WORK', UpdateWaypoint(node),
                               transitions={'succeeded': 'SOUND_PLAY_DONE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_DONE', SoundPlay(node, 'update_waypoint_completed'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm