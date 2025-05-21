# Description: This file contains the state machine for the detect object task.
import smach
from fsm_waypoint.states.subscriber import CheckAlertFromRecovery, GetWssNavFnCommand
from fsm_waypoint.states.publisher import SoundPlay


def detect_object(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'navfn'],
                            input_keys=['blackboard'],
                            output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('CHECK_ALERT_FROM_RECOVERY', CheckAlertFromRecovery(node, 10),
                               transitions={'succeeded': 'OBJECT_DETECTION', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('OBJECT_DETECTION', SoundPlay(node, 'object_detection'),
                               transitions={'succeeded': 'GET_WSS_NAVFN_COMMAND', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_WSS_NAVFN_COMMAND', GetWssNavFnCommand(node, 30),
                               transitions={'navfn': 'MANUAL_CONTROL',
                                            'succeeded': 'CHECK_ALERT_FROM_RECOVERY', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MANUAL_CONTROL', SoundPlay(node, 'manual_control', 5),
                               transitions={'succeeded': 'navfn', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm

# todo: Detected -> Undetected 시에 아래 명령으로 clean해주면 더 빠르게 진행됨.
# root@desktop:~/ws# ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap