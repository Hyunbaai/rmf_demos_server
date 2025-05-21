# Description: Select NavfnPlanner and move robot to initial position
import smach
from fsm_waypoint.states.subscriber import GetGpsFiltered, GetRobotHeading
from fsm_waypoint.states.publisher import ChangePlannerSelect, WheelControl, SoundPlay
from fsm_waypoint.states.navigator import GpsWheelControlByCalculation


def planner_select(node):
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'],
                                             input_keys=['blackboard'],
                                             output_keys=['blackboard'])

    with sm:
        smach.StateMachine.add('GET_INITIAL_GPS_DATA', GetGpsFiltered(node, 'init'),
                               transitions={'succeeded': 'GET_HEADING_DATA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GET_HEADING_DATA', GetRobotHeading(node),
                               transitions={'succeeded': 'SOUND_PLAY_BACKWARD', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('SOUND_PLAY_BACKWARD', SoundPlay(node, 'backward'),
                               transitions={'succeeded': 'MOVE_DELTA', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MOVE_DELTA', WheelControl(node, 'MoveDelta', -0.7, 7),
                               transitions={'succeeded': 'CHANGE_NAVFNPLANNER', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('CHANGE_NAVFNPLANNER', ChangePlannerSelect(node, 'NavfnPlanner'),
                               transitions={'succeeded': 'OBSTACLE_AVOIDANCE', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('OBSTACLE_AVOIDANCE', SoundPlay(node, 'obstacle_avoidance'),
                               transitions={'succeeded': 'GPS_WHEEL_CONTROL', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('GPS_WHEEL_CONTROL', GpsWheelControlByCalculation(node, 'forward', 2),
                               transitions={'succeeded': 'succeeded', 'preempted': 'preempted',
                                            'aborted': 'aborted'})
    return sm
