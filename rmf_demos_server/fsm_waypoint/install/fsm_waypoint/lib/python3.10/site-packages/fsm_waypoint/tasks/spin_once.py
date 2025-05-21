# Description: This file contains the spin_once task.
import smach
from fsm_waypoint.states.lib import RunSpinOnce


def spin_once(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                   input_keys=['blackboard'],
                                   output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('RUN_SPIN_ONCE', RunSpinOnce(node),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm