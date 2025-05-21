# Description: This file contains the emergency_stop task.
import smach
from fsm_waypoint.states.subscriber import GetA012Emergency, GetA012Release, GetBumperEmergency, GetBumperRelease
from fsm_waypoint.states.publisher import SoundPlay
from fsm_waypoint.utils import child_termination_cb, outcome_cb

def emergency_stop(node):
    sw_emergency = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                                input_keys=['blackboard'],
                                                output_keys=['blackboard'])

    with sw_emergency:
        smach.StateMachine.add('GET_BUMPER_EMERGENCY', GetBumperEmergency(node),
                               transitions={'bumper': 'BUMPER_PRESSED', 'succeeded': 'BUMPER_EMERGENCY_PRESSED', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_PRESSED', SoundPlay(node, 'bumper_emergency_pressed'),
                               transitions={'succeeded': 'GET_BUMPER_EMERGENCY_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_BUMPER_EMERGENCY_RELEASE', GetBumperRelease(node),
                               transitions={'bumper': 'BUMPER_PRESSED2', 'succeeded': 'BUMPER_EMERGENCY_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_RELEASE', SoundPlay(node, 'bumper_emergency_release'),
                               transitions={'succeeded': 'GET_BUMPER_EMERGENCY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_PRESSED', SoundPlay(node, 'bumper_pressed'),
                               transitions={'succeeded': 'GET_BUMPER_EMERGENCY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_PRESSED2', SoundPlay(node, 'bumper_pressed'),
                               transitions={'succeeded': 'GET_BUMPER_EMERGENCY_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})

    hw_emergency = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['blackboard'],
                                      output_keys=['blackboard'])

    with hw_emergency:
        smach.StateMachine.add('GET_A012_EMERGENCY', GetA012Emergency(node),
                               transitions={'succeeded': 'BUMPER_EMERGENCY_PRESSED', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_PRESSED', SoundPlay(node, 'bumper_emergency_pressed'),
                               transitions={'succeeded': 'GET_A012_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_A012_RELEASE', GetA012Release(node),
                               transitions={'succeeded': 'BUMPER_EMERGENCY_RELEASE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('BUMPER_EMERGENCY_RELEASE', SoundPlay(node, 'bumper_emergency_release'),
                               transitions={'succeeded': 'GET_A012_EMERGENCY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})

    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('SW_EMERGENCY', sw_emergency)
        smach.Concurrence.add('HW_EMERGENCY', hw_emergency)
    return sm