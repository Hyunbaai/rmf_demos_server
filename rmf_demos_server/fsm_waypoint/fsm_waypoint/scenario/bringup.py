
import smach
from fsm_waypoint.states.lib import PreemptedTimeout, CheckLifecycleStates
from fsm_waypoint.states.publisher import SoundPlay, AccessoryActionClient, WheelControl, ChangePlannerSelect
from fsm_waypoint.states.subscriber import GetRobotID, GetAccessoryStatus
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.utils import child_termination_cb, outcome_cb

def bringup(node):
    bringup_sim_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                      input_keys=['blackboard'],
                                      output_keys=['blackboard'])

    with bringup_sim_sm:
        smach.StateMachine.add('GET_ROBOT_ID', GetRobotID(node),
                               transitions={'succeeded': 'NANO_READY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('NANO_READY', SoundPlay(node, 'nano_ready'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        # desc: only once, when the robot is turned on
        # todo: version check, download files, etc

    bringup_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                              input_keys=['blackboard'],
                              output_keys=['blackboard'])

    with bringup_sm:
        smach.StateMachine.add('TOP_LED_ON', AccessoryActionClient(node, 'TopLedOn'),
                               transitions={'succeeded': 'LIGHT_ON', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('LIGHT_ON', AccessoryActionClient(node, 'LightAllOn'),
                               transitions={'succeeded': 'GET_ROBOT_ID', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_ROBOT_ID', GetRobotID(node),
                               transitions={'succeeded': 'NANO_READY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('NANO_READY', SoundPlay(node, 'nano_ready'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('DOOR_OPEN', AccessoryActionClient(node, 'DoorOpen'),
                               transitions={'succeeded': 'PREEMPTED_TIMEOUT', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('PREEMPTED_TIMEOUT', PreemptedTimeout(10),
                               transitions={'succeeded': 'DOOR_CLOSE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('DOOR_CLOSE', AccessoryActionClient(node, 'DoorClose'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        # desc: only once, when the robot is turned on
        # todo: version check, download files, etc

    bringup_v2_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                              input_keys=['blackboard'],
                              output_keys=['blackboard'])

    with bringup_v2_sm:
        # smach.StateMachine.add('ACCESSORY_STATUS', AccessoryActionClient(node, 'Status'),
        #                        transitions={'succeeded': 'GET_ACCESSORY_STATUS', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('GET_ACCESSORY_STATUS', GetAccessoryStatus(node),
        #                        transitions={'succeeded': 'TOP_LED_OFF', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('TOP_LED_OFF', AccessoryActionClient(node, 'TopLedOff', 10),
        #                        transitions={'succeeded': 'LIGHT_OFF', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('LIGHT_OFF', AccessoryActionClient(node, 'LightAllOff', 10),
        #                        transitions={'succeeded': 'NANO_READY', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('TOP_LED_ON', AccessoryActionClient(node, 'TopLedOn'),
        #                        transitions={'succeeded': 'TMP01', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('TMP01', PreemptedTimeout(1),
        #                        transitions={'succeeded': 'LIGHT_ON', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('LIGHT_ON', AccessoryActionClient(node, 'LightAllOn'),
        #                        transitions={'succeeded': 'GET_ROBOT_ID', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('GET_ROBOT_ID', GetRobotID(node),
        #                        transitions={'succeeded': 'DOOR_OPEN', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('DOOR_OPEN', AccessoryActionClient(node, 'DoorOpen'),
        #                        transitions={'succeeded': 'FIRST_TIMEOUT', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('FIRST_TIMEOUT', PreemptedTimeout(2),
        #                        transitions={'succeeded': 'DOOR_CLOSE', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('DOOR_CLOSE', AccessoryActionClient(node, 'DoorClose'),
        #                        transitions={'succeeded': 'SECOND_TIMEOUT', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('SECOND_TIMEOUT', PreemptedTimeout(2),
        #                        transitions={'succeeded': 'CHECK_FORWARD', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('CHECK_FORWARD', WheelControl(node, 'MoveDelta', 0.1, 2),
        #                        transitions={'succeeded': 'CHECK_BACKWARD', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('CHECK_BACKWARD', WheelControl(node, 'MoveDelta', -0.1, 2),
        #                        transitions={'succeeded': 'CHECK_LEFT_TURN', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('CHECK_LEFT_TURN', WheelControl(node, 'TurnDelta', -15, 5),
        #                        transitions={'succeeded': 'CHECK_RIGHT_TURN', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        # smach.StateMachine.add('CHECK_RIGHT_TURN', WheelControl(node, 'TurnDelta', 15, 5),
        #                        transitions={'succeeded': 'NANO_READY', 'preempted': 'preempted',
        #                                     'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_LIFECYCLE', CheckLifecycleStates(node),
                               transitions={'succeeded': 'NANO_READY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('NANO_READY', SoundPlay(node, 'nano_ready'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        # desc: only once, when the robot is turned on
        # todo: version check, download files, etc

    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                                default_outcome='aborted',
                                input_keys=['blackboard'],
                                output_keys=['blackboard'],
                                child_termination_cb=child_termination_cb,
                                outcome_cb=outcome_cb)

    with sm:
        smach.Concurrence.add('bringup', bringup_v2_sm)
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))

    return sm