
# ros2
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class ChangePlannerSelect(smach.State):
    def __init__(self, node, planner_name, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node
        self.planner_name = planner_name
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.pub_ = self.node.create_publisher(String, '/planner_selector',qos_profile)

    def execute(self, userdata):
        start_time = time.time()
        while rclpy.ok():
            msg = String()
            msg.data = self.planner_name
            self.pub_.publish(msg)
            debug(f'Sent planner: {msg.data}')
            break

        outcome = 'succeeded' # because timeout not mandatory
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
            else:
                break
        return outcome