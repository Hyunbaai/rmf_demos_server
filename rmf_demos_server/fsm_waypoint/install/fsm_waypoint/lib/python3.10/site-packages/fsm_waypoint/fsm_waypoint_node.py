# Description: FSM for waypoint navigation
# ros2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

# fsm
import smach
from fsm_waypoint.utils import child_termination_cb, outcome_cb
from fsm_waypoint.tasks import spin_once
from fsm_waypoint.states.navigator import RMFGoToPose, RMFNavToPose
from fsm_waypoint.states.publisher import FleetManager, RMFRobotState
from fsm_waypoint.states.subscriber import RobotStateSubscriber

# utils
from fsm_waypoint.utils import debug, info, warning, error, critical
from fsm_waypoint.utils import init_blackboard
import uuid
import math
import time
import yaml
import os

def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return {"z": sy, "w": cy}

class WaitForInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.result = None
    def execute(self, userdata):
        info("Press 'Enter' to repeat:")
        self.result = 'aborted' # execute is not allowed to static method
        while True:
            user_input = input().strip().lower()
            self.result = 'succeeded'
            break
        return self.result


class SetInitialPoseState(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.node = node
        unique_name = f"set_initial_pose_{uuid.uuid4().hex}"
        self.navigator = BasicNavigator(unique_name)

    def execute(self, ud):
        ud.blackboard.wait_spin = True
        time.sleep(0.1)  # wait for spin_once to disable

        outcome = 'aborted'
        # tinybot1 cartographer map
        # initial_pose = {
        #     "x": 3.0899, # -2.4673, # rmf 10.4295
        #     "y": 0.5361, # 1.6716, # rmf −5.5703
        #     "yaw": 1.3286,
        # }
        # tinybot2 cartographer map
        # initial_pose = {
        #     "x": 12.7971,  # -2.4673, # rmf 10.4295
        #     "y": 2.7541,  # 1.6716, # rmf −5.5703
        #     "yaw": -0.696,
        # }
        # tinybot3 cartographer map
        initial_pose = {
            "x": 5.4117,  # -2.4673, # rmf 10.4295
            "y": -4.7234,  # 1.6716, # rmf −5.5703
            "yaw": 1.5691,
        }
        # gpuserver turtlebot3
        # initial_pose = {
        #     "x": -2.0,
        #     "y": -0.50,
        #     "yaw": 0.0,
        # }
        # gpuserver tinyRobot
        # initial_pose = {
        #     "x": 1.9042,
        #     "y": -0.7681,
        #     "yaw": 1.3286,
        # }
        # nuc #1
        # initial_pose = {
        #     "x": 0.08245692402124405,
        #     "y": 0.5682854652404785,
        #     "yaw": 1.57,
        # }
        # nuc #2
        # initial_pose = {
        #     "x": 0.017210671678185463,
        #     "y": -0.5261098146438599,
        #     "yaw": 0,
        # }

        # Set the initial pose
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = initial_pose["x"]
        pose.pose.position.y = initial_pose["y"]
        pose.pose.orientation.z = yaw_to_quaternion(initial_pose["yaw"])["z"]
        pose.pose.orientation.w = yaw_to_quaternion(initial_pose["yaw"])["w"]

        # Set the initial pose using the navigator
        self.navigator.setInitialPose(pose)
        debug('Setting initial pose')

        # Wait until Nav2 is active
        debug('Waiting for Nav2 to be active')
        # self.navigator.waitUntilNav2Active()  # fsm_waypoint.sh 에서 이미 확인했음.
        outcome = 'succeeded'

        ud.blackboard.wait_spin = False
        return outcome


def bringup(node):
    tmp = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with tmp:
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        smach.Concurrence.add('SET_INITIAL_POSE', SetInitialPoseState(node))
    return tmp

def ready(node, config):
    tmp = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['blackboard'],
                           output_keys=['blackboard'],
                           child_termination_cb=child_termination_cb,
                           outcome_cb=outcome_cb)

    with tmp:
        smach.Concurrence.add('SPIN_ONCE', spin_once(node))
        # smach.Concurrence.add('RMF_GO_TO_POSE', RMFNavToPose(node, config))
        smach.Concurrence.add('FLEET_MANAGER', FleetManager(node, config))
        # smach.Concurrence.add('ROBOT_STATE_SUBSCRIBER', RobotStateSubscriber(node))

    return tmp

# Create the state machine
def main(args=None):
    rclpy.init(args=args)
    node = Node('fsm_waypoint_node')
    node.declare_parameter('test01', 100.0)
    node.declare_parameter('test02', False)
    smach.set_loggers(info, warning, debug, error)
    # Create state machine
    top = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'overall_success'])
    top.userdata.blackboard = init_blackboard()

    config_file = os.getenv("CONFIG_FILE", "/home/zeta/rmf_demos_server/rmf_demos/config/office/tinyRobot_with_nav2_config.yaml")

    with open(config_file, "r") as f:
        config_yaml = yaml.safe_load(f)

    with top:
        # todo:
        #   set initial position
        #   subscribe robot_path_request
        #   goToPose
        # smach.StateMachine.add(
        #     'BRINGUP',
        #     bringup(node),
        #     transitions={'succeeded': 'READY', 'aborted': 'aborted', 'preempted': 'preempted'}
        # )
        smach.StateMachine.add(
            'READY',
            ready(node, config_yaml),
            transitions={'succeeded': 'WAIT_FOR_INPUT', 'aborted': 'aborted', 'preempted': 'preempted'}
        )
        smach.StateMachine.add(
            'WAIT_FOR_INPUT',
            WaitForInputState(),
            transitions={'succeeded': 'READY', 'aborted': 'aborted', 'preempted': 'preempted'}

        )
    outcome = top.execute()
    if outcome == 'overall_success':
        info('State machine executed overall successfully')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
