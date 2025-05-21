# Description: This file contains the state machine for the gps wheel control task.
import smach
from fsm_waypoint.states.subscriber import GetGpsFiltered, GetRobotHeading, CheckAccuracy
from fsm_waypoint.states.publisher import SoundPlay, ChangePlannerSelect
from fsm_waypoint.states.navigator import GpsWheelControlByCalculation


def gps_wheel_control(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                           input_keys=['blackboard'],
                                           output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('SOUND_PLAY_START', SoundPlay(node, 'gps_wheel_control'),
                               transitions={'succeeded': 'CHANGE_STRAIGHTLINE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHANGE_STRAIGHTLINE', ChangePlannerSelect(node, 'StraightLine'),
                               transitions={'succeeded': 'CHECK_ACCURACY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_ACCURACY', CheckAccuracy(node),
                               transitions={'succeeded': 'GET_INITIAL_GPS_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_INITIAL_GPS_DATA', GetGpsFiltered(node, 'init'),
                               transitions={'succeeded': 'GET_HEADING_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_HEADING_DATA', GetRobotHeading(node),
                               transitions={'succeeded': 'GPS_WHEEL_CONTROL', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GPS_WHEEL_CONTROL', GpsWheelControlByCalculation(node),
                               transitions={'succeeded': 'SOUND_PLAY_DONE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_DONE', SoundPlay(node, 'gps_wheel_control_completed'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm