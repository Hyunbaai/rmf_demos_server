
# ros2
import rclpy
from rclpy.action import ActionClient
from accessory_msgs.action import AccessoryAction

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class AccessoryActionClient(smach.State):
    def __init__(self, node, label, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self._send_goal_future = None
        self._get_result_future = None
        self.timeout = timeout
        self.node = node
        self.label = label

        self._action_client = ActionClient(self.node, AccessoryAction, 'accessory_action')
        self.command_params = self.get_command_params(label)

    def get_command_params(self, label):
        groupedAccessoryCommands = {
            'door': [
                {'label': 'DoorOpen', 'target': 'D', 'command': '2', 'color': '0'},
                {'label': 'DoorClose', 'target': 'D', 'command': '1', 'color': '0'},
                {'label': 'DoorPause', 'target': 'D', 'command': '3', 'color': '0'}
            ],
            'light': [
                {'label': 'LightAllOff', 'target': 'L', 'command': '0', 'color': '0'},
                {'label': 'LightAllOn', 'target': 'L', 'command': '5', 'color': '0'}
            ],
            'topLed': [
                {'label': 'TopLedOff', 'target': 'T', 'command': '0', 'color': '0'},
                {'label': 'TopLedOn', 'target': 'T', 'command': '5', 'color': 'FF0000'}
            ],
            'bottomLed': [
                {'label': 'BottomLedOff', 'target': 'B', 'command': '0', 'color': '0'},
                {'label': 'BottomLedOn', 'target': 'B', 'command': '1', 'color': '0'}
            ],
            'status': [
                {'label': 'Status', 'target': 'S', 'command': '0', 'color': '0'}
            ]
        }

        for group in groupedAccessoryCommands.values():
            for command in group:
                if command['label'] == label:
                    return command
        return None

    def execute(self, userdata):
        if not self.command_params:
            error(f'label "{self.label}" cannot be found in the command parameters.')
            return 'aborted'

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            error('unable to connect to the Accessory Action server')
            return 'aborted'

        goal_msg = AccessoryAction.Goal()
        goal_msg.target = self.command_params['target']
        goal_msg.command = self.command_params['command']
        goal_msg.color = self.command_params['color']

        # send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self.node, self._send_goal_future)

        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            error('Reject Goal')
            return 'aborted'

        info('Accept Goal')

        self._get_result_future = goal_handle.get_result_async()

        start_time = time.time()
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if self.timeout and (time.time() - start_time) > self.timeout:
                info('timeout, cancel goal')
                goal_handle.cancel_goal_async()
                return 'succeeded'

            time.sleep(0.1)

            if self._get_result_future.done():
                result = self._get_result_future.result().result
                if result.success:
                    info('success accessory action server')
                    return 'succeeded'
                else:
                    error('failed accessory action server')
                    return 'aborted'

        return 'aborted'

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        info(f'feedback: {feedback}')