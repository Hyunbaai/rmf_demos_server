# Description: Network error task for FSM waypoint
import smach
from fsm_waypoint.states.subscriber import GetNetworkStatus
from fsm_waypoint.states.publisher import SoundPlay, WheelControl
from fsm_waypoint.states.lib import CheckPingNetwork
from fsm_waypoint.utils import child_termination_cb, outcome_cb


def network_error(node):
    local_network = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                            input_keys=['blackboard'],
                            output_keys=['blackboard'])

    with local_network:
        smach.StateMachine.add('GET_NETWORK_STATUS', GetNetworkStatus(node),
                               transitions={'up': 'STOP', 'down': 'STOP',
                                            'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('STOP', WheelControl(node, 'Stop', 0, 1),  # 1 timeout은 예비용
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})

    remote_network = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                       input_keys=['blackboard'],
                                       output_keys=['blackboard'])

    with remote_network:
        smach.StateMachine.add('CHECK_PING_NETWORK', CheckPingNetwork(node),
                               transitions={'up': 'STOP', 'down': 'STOP',
                                            'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('STOP', WheelControl(node, 'Stop', 0, 1),  # 1 timeout은 예비용
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})

    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('LOCAL_NETWORK', local_network)
        smach.Concurrence.add('REMOTE_NETWORK', remote_network)
    return sm