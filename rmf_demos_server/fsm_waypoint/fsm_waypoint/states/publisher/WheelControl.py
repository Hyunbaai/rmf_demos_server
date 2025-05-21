
# ros2
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical
import json
import math


class WheelControl(smach.State):
    def __init__(self, node, function, value, timeout):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node
        self.value = value
        self.function = function
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.pub_ = self.node.create_publisher(String,
                                               '/robot_command',
                                               qos_profile)

    def execute(self, userdata):
        start_time = time.time()

        while rclpy.ok():
            if self.function == 'MoveDelta':
                self.move(self.function, self.value)
            elif self.function == 'TurnDelta':
                debug(f"Current bearing: {userdata.blackboard.bearing} degrees")
                rotation_angle = self.value - userdata.blackboard.bearing # bearing N 0, E 90, S 180, W 270 based

                debug(f"Rotation angle: {rotation_angle} degrees")
                angle_radians = math.radians(rotation_angle)

                angle_radians = angle_radians * -1
                self.move(self.function, angle_radians)
            break

        outcome = 'aborted' # because timeout mandatory
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                self.move('Stop', 0.0)
                outcome = 'preempted'
                break
            time.sleep(0.1)
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    self.move('Stop', 0.0)
                    outcome = 'succeeded'
                    break
        return outcome

    def move(self, function, value):
        debug(f'WheelControl - Function: {function}, Value: {value}')
        command = {self.function: value}
        command_str = json.dumps(command)
        msg = String()
        msg.data = command_str
        self.pub_.publish(msg)