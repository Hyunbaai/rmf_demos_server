# ros2
import rclpy
from nav_msgs.msg import Odometry

# fsm
import smach

# utils
import time
import math
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical
from fsm_waypoint.utils import euler_from_quaternion

class GetRobotHeading(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(Odometry,
                                                  '/odometry/global',
                                                  self.odom_global_callback,
                                                  10)
        self.result = None  # initialize result
        start_time = time.time()
        outcome = 'aborted'
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break
            time.sleep(0.1)
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    outcome = 'succeeded'
                    break
            if self.result:
                userdata.blackboard.heading = self.result
                debug(f'Heading: {userdata.blackboard.heading} radians')
                outcome = 'succeeded'
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def odom_global_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion(msg.pose.pose.orientation)

        # Yaw 값 0 ~ 2pi로 정규화
        def normalize_yaw(yaw):
            two_pi = 2.0 * math.pi
            yaw = math.fmod(yaw, two_pi)
            if yaw < 0.0:
                yaw += two_pi
            return yaw

        current_yaw = normalize_yaw(yaw)
        self.result = current_yaw  # radian