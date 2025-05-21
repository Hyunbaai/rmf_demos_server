import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_fleet_msgs.msg import RobotState
import smach

# utils
from fsm_waypoint.utils import debug, info, warning, error, critical


class RobotStateSubscriber(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])

        self.node = node
        self.timeout = timeout
        self.result = None
        self.sub_ = None
        info('RobotStateSubscriber state initialized.')

    def execute(self, userdata):
        # Define QoS Profile
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL
        )
        # Create the subscription
        self.sub_ = self.node.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_cb,
            qos_profile=transient_qos
        )
        start_time = self.node.get_clock().now().to_msg().sec
        info('Waiting for RobotState messages...')
        self.result = 'aborted'

        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                info('Preempted by user.')
                self.result = 'preempted'
                break

            # Check for timeout
            if self.timeout > 0:
                elapsed_time = self.node.get_clock().now().to_msg().sec - start_time
                if elapsed_time >= self.timeout:
                    info('Timeout reached while waiting for RobotState messages.')
                    break

            # rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(self.sub_)
        return self.result

    def robot_state_cb(self, msg):
        """Callback function to process received RobotState messages."""
        separator = "-" * 50
        info(separator)
        info('Received RobotState message:')
        info(f'  Name: {msg.name}')
        info(f'  Model: {msg.model}')
        info(f'  Task ID: {msg.task_id}')
        info(f'  Sequence: {msg.seq}')
        info(f'  Mode: {msg.mode.mode}')
        info(f'  Battery: {msg.battery_percent:.2f}%')
        info(f'  Location: (x: {msg.location.x}, y: {msg.location.y}, yaw: {msg.location.yaw})')
        info(f'  Path length: {len(msg.path)}')

        # Optionally print the path
        if msg.path:
            for i, loc in enumerate(msg.path):
                info(f'    Path[{i}]: (x: {loc.x}, y: {loc.y}, yaw: {loc.yaw})')
        info(separator)

        self.result = 'succeeded'
