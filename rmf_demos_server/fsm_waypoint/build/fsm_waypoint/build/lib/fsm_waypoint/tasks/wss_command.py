# Description: This file contains the state machines for the wss command, wss cancel command, and wss cancel navfn command.
import smach
from fsm_waypoint.states.subscriber import GetWssCommand, GetWssCancelCommand, GetWssNavFnCommand


def wss_command(node):
    sm = smach.StateMachine(
        outcomes=['bearing', 'waypoint', 'path', 'create', 'update', 'succeeded', 'preempted', 'aborted', 'cancel'],
        input_keys=['blackboard'],
        output_keys=['blackboard'])
    with sm:
        smach.StateMachine.add('GET_WSS_COMMAND', GetWssCommand(node),
                               transitions={'bearing': 'bearing', 'waypoint': 'waypoint', 'path': 'path',
                                            'create': 'create', 'update': 'update', 'cancel': 'cancel',
                                            'succeeded': 'succeeded',
                                            'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm


def wss_cancel_command(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'cancel'],
                            input_keys=['blackboard'],
                            output_keys=['blackboard'])
    with sm:
        smach.StateMachine.add('GET_WSS_CANCEL_COMMAND', GetWssCancelCommand(node),
                               transitions={'cancel': 'cancel',
                                            'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'aborted'})
    return sm


def wss_navfn_command(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'navfn'],
                            input_keys=['blackboard'],
                            output_keys=['blackboard'])
    with sm:
        smach.StateMachine.add('GET_WSS_NAVFN_COMMAND', GetWssNavFnCommand(node),
                               transitions={'navfn': 'navfn',
                                            'succeeded': 'succeeded', 'preempted': 'preempted', 'aborted': 'aborted'})
    return sm
