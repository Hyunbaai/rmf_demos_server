# Description: This file contains the state machine for creating a waypoint.
import smach
from fsm_waypoint.states.publisher import SoundPlay
from fsm_waypoint.states.subscriber import CheckGlobalCostmap, GetGpsFiltered, GetRobotHeading
from fsm_waypoint.states.lib import CreateWaypoint


def create_waypoint(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                         input_keys=['blackboard'],
                                         output_keys=['blackboard'])

    with sm:
        # sound play "create waypoint"
        smach.StateMachine.add('SOUND_PLAY_START', SoundPlay(node, 'create_waypoint'),
                               transitions={'succeeded': 'CHECK_GLOBAL_COSTMAP', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_GLOBAL_COSTMAP', CheckGlobalCostmap(node, 5),
                               transitions={'succeeded': 'GET_INITIAL_GPS_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_INITIAL_GPS_DATA', GetGpsFiltered(node, 'init'),
                               transitions={'succeeded': 'GET_HEADING_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_HEADING_DATA', GetRobotHeading(node),
                               transitions={'succeeded': 'CREATE_WAYPOINT_WORK', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CREATE_WAYPOINT_WORK', CreateWaypoint(node),
                               transitions={'succeeded': 'SOUND_PLAY_DONE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_DONE', SoundPlay(node, 'create_waypoint_completed'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm