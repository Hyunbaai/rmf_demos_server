# Description: This file contains the state machine for the task initialize_bearing.
import smach
from fsm_waypoint.states.lib import PreemptedTimeout, CalculateBearing
from fsm_waypoint.states.subscriber import CheckObjectDetect, GetGpsFiltered, CheckGlobalCostmap, CheckAccuracy
from fsm_waypoint.states.publisher import SoundPlay, WheelControl, RestartPM2
from fsm_waypoint.states.navigator import GoToPose


def initialize_bearing(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                                        input_keys=['blackboard'],
                                                        output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('SOUND_PLAY_START', SoundPlay(node, 'initialize_bearing_measurement'),
                               transitions={'succeeded': 'CHECK_OBJECT_DETECT', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_OBJECT_DETECT', CheckObjectDetect(node),
                               transitions={'succeeded': 'CHECK_ACCURACY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_ACCURACY', CheckAccuracy(node),
                               transitions={'succeeded': 'GET_INITIAL_GPS_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_INITIAL_GPS_DATA', GetGpsFiltered(node, 'init'),
                               transitions={'succeeded': 'MOVE_DELTA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MOVE_DELTA', WheelControl(node, 'MoveDelta', 1.2, 15),
                               transitions={'succeeded': 'WAIT', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('WAIT', PreemptedTimeout(2),
                               transitions={'succeeded': 'GET_FINAL_GPS_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_FINAL_GPS_DATA', GetGpsFiltered(node, 'final'),  # 2 seconds timeout
                               transitions={'succeeded': 'CALCULATE_BEARING', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CALCULATE_BEARING', CalculateBearing(node),  # 2 seconds timeout
                               transitions={'succeeded': 'ROTATE_TO_BEARING', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('ROTATE_TO_BEARING', WheelControl(node, 'TurnDelta', 90, 10),
                               transitions={'succeeded': 'RESTART_PM2', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('RESTART_PM2',
                               RestartPM2(node, "a012nano-a012-1", "pm2 restart dual_ekf_navsat", 10),
                               transitions={'succeeded': 'CHECK_GLOBAL_COSTMAP', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHECK_GLOBAL_COSTMAP', CheckGlobalCostmap(node, 5),
                               transitions={'succeeded': 'GO_TO_POSE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GO_TO_POSE', GoToPose(node, 10),
                               transitions={'succeeded': 'SOUND_PLAY_DONE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_DONE', SoundPlay(node, 'bearing_measurement_completed'),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm