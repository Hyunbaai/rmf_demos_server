
import smach
from fsm_waypoint.tasks import network_error
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.tasks import wss_cancel_command
from fsm_waypoint.tasks import follow_gps_path
from fsm_waypoint.tasks import detect_object
from fsm_waypoint.utils import child_termination_cb, outcome_cb


def path(node):
    tmp = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'cancel', 'navfn'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with tmp:
        smach.Concurrence.add('WSS_CANCEL_COMMAND', wss_cancel_command(node))
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        smach.Concurrence.add('NETWORK_ERROR', network_error(node))
        smach.Concurrence.add('FOLLOW_GPS_PATH', follow_gps_path(node))
        smach.Concurrence.add('DETECT_OBJECT', detect_object(node))
    return tmp