# ros2
import rclpy
from std_msgs.msg import String

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class CheckAlertFromRecovery(smach.State):
    def __init__(self, node, stop_timeout=0, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])

        self.timeout = timeout
        self.sub01_ = None
        self.sub02_ = None
        self.node = node
        self.result = None
        self.stop_timeout = stop_timeout

        self.obstacle_detected = False
        self.robot_stopped = None
        self.stop_time = None
        self.obstacle_timeout = False

    def execute(self, userdata):
        self.sub01_ = self.node.create_subscription(
            String,
            '/alert_topic',
            self.alert_topic_callback,
            10
        )
        self.sub02_ = self.node.create_subscription(
            String,
            '/obstacle_topic',
            self.obstacle_topic_callback,
            10
        )

        self.result = None
        self.obstacle_detected = False
        # self.robot_stopped = None # Obstacle Clear 없이 next state로 진입해서
        self.stop_time = None
        self.obstacle_timeout = False

        start_time = time.time()
        outcome = 'aborted'
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                userdata.blackboard.obstacle_timeout = self.obstacle_timeout
                outcome = 'preempted'
                break
            time.sleep(0.1)
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    userdata.blackboard.obstacle_timeout = self.obstacle_timeout
                    outcome = 'succeeded'
                    break
            if self.result:
                outcome = 'succeeded'
                break
            if self.robot_stopped and self.obstacle_detected:
                if not self.stop_time:
                    debug(f'Robot stopped and obstacle detected, start timeout: {self.stop_timeout}')
                    self.stop_time = time.time()
                elif time.time() - self.stop_time > self.stop_timeout:
                    self.result = True
                    debug(f'Robot stopped and obstacle detected, end timeout: {self.stop_timeout}')
                    self.obstacle_timeout = True
                    userdata.blackboard.obstacle_timeout = self.obstacle_timeout

        self.node.destroy_subscription(self.sub01_)
        self.node.destroy_subscription(self.sub02_)
        return outcome

    def alert_topic_callback(self, msg):
        if msg.data == "Robot Stopped":
            self.robot_stopped = True
        elif msg.data == "Obstacle Clear":
            self.robot_stopped = False

    def obstacle_topic_callback(self, msg):
        if msg.data == "Detected":
            self.obstacle_detected = True
        elif msg.data == "Undetected":
            if self.obstacle_detected:
                debug('Detected -> Undetected, reset variables')
            self.obstacle_detected = False
            self.robot_stopped = False  # 재 진행시
            self.stop_time = None  # 재 진행시
            self.obstacle_timeout = False