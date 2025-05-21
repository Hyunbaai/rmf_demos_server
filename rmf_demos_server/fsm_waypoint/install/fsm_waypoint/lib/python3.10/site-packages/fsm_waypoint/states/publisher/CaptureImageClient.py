
# ros2
import rclpy
from rclpy.action import ActionClient
from capture_image_msgs.action import CaptureImage

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class CaptureImageClient(smach.State):
    def __init__(self, node, device, direction, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self._get_result_future = None
        self._send_goal_future = None
        self.timeout = timeout
        self.node = node
        self.device = device
        self.direction = direction

        self._action_client = ActionClient(self.node, CaptureImage, 'capture_image')

    def execute(self, userdata):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            error('cannot connect to the Capture Image server')
            return 'aborted'
        if userdata.blackboard.robot_id is None:
            error('robot_id is None')
            userdata.blackboard.robot_id = 'empty'

        goal_msg = CaptureImage.Goal()
        goal_msg.device = self.device
        goal_msg.robot_id = userdata.blackboard.robot_id
        goal_msg.direction = self.direction

        # send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # wait for goal to be accepted
        rclpy.spin_until_future_complete(self.node, self._send_goal_future)

        # get the goal handle
        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            error('Reject Goal')
            return 'aborted'

        info('Accept Goal')

        # get the result future
        self._get_result_future = goal_handle.get_result_async()

        start_time = time.time()
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            if self.timeout and (time.time() - start_time) > self.timeout:
                error('timeout, cancel goal')
                goal_handle.cancel_goal_async()
                return 'aborted'

            time.sleep(0.1)
            # check if the result is available
            if self._get_result_future.done():
                result = self._get_result_future.result().result
                if result.upload_url:
                    info(f'upload url: {result.upload_url}')
                    return 'succeeded'
                else:
                    error('failed to capture image')
                    return 'aborted'

        return 'aborted'

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        info(f'feedback: {feedback}')