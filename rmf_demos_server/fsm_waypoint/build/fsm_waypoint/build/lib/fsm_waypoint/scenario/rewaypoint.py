
import smach
from fsm_waypoint.states.publisher import SoundPlay, ChangePlannerSelect
from fsm_waypoint.states.navigator import GpsWheelControlByWaypoint

from fsm_waypoint.tasks import detect_object
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.tasks import network_error
from fsm_waypoint.tasks import wss_cancel_command
from fsm_waypoint.utils import child_termination_cb, outcome_cb


def gps_wheel_control_(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                           input_keys=['blackboard'],
                                           output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('SOUND_PLAY_START', SoundPlay(node, 'gps_wheel_control'),
                               transitions={'succeeded': 'CHANGE_STRAIGHTLINE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHANGE_STRAIGHTLINE', ChangePlannerSelect(node, 'StraightLine'),
                               transitions={'succeeded': 'GPS_WHEEL_CONTROL', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GPS_WHEEL_CONTROL', GpsWheelControlByWaypoint(node),
                               transitions={'succeeded': 'SOUND_PLAY_DONE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_DONE', SoundPlay(node, 'gps_wheel_control_completed'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm

def rewaypoint(node):
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'cancel', 'navfn'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('WSS_CANCEL_COMMAND', wss_cancel_command(node))
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        smach.Concurrence.add('NETWORK_ERROR', network_error(node))
        smach.Concurrence.add('GPS_WHEEL_CONTROL_', gps_wheel_control_(node))
        smach.Concurrence.add('DETECT_OBJECT', detect_object(node))
    return sm