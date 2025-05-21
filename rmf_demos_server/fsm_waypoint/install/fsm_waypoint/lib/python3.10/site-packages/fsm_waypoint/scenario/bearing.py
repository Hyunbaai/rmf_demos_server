
import smach
from fsm_waypoint.tasks import emergency_stop
from fsm_waypoint.tasks import network_error
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.tasks import wss_cancel_command
from fsm_waypoint.tasks import initialize_bearing
from fsm_waypoint.utils import child_termination_cb, outcome_cb


def bearing(node):
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'cancel'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('EMERGENCY_STOP', emergency_stop(node))
        smach.Concurrence.add('WSS_CANCEL_COMMAND', wss_cancel_command(node))
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        smach.Concurrence.add('INITIALIZE_BEARING', initialize_bearing(node))
        smach.Concurrence.add('NETWORK_ERROR', network_error(node))
    return sm