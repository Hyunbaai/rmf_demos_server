# ros2
import rclpy
from std_msgs.msg import Float32MultiArray

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class GetNetworkStatus(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['up', 'down', 'succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None
        self.previous_state = None  # 이전 네트워크 상태 추적

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(
            Float32MultiArray,
            'network_status',
            self.network_status_callback,
            10
        )
        self.result = None  # 결과 초기화
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
            if self.result is not None and self.result != self.previous_state:
                if self.result:
                    outcome = 'down'  # 네트워크가 다운되었을 때
                else:
                    outcome = 'up'  # 네트워크가 복구되었을 때
                self.previous_state = self.result  # 상태가 변경되었을 때에만 업데이트
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def network_status_callback(self, msg):
        # 메시지의 데이터가 모두 0인지 확인하여 상태를 업데이트
        is_network_down = all(value == 0 for value in msg.data)

        if self.result is None:
            # 초기 상태 설정
            self.result = is_network_down
            self.previous_state = is_network_down
        elif self.result != is_network_down:
            # 네트워크 상태가 변경되었을 때만 업데이트
            self.result = is_network_down