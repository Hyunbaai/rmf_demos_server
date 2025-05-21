# ros2
import rclpy
from lifecycle_msgs.srv import GetState

# fsm
import smach

# utils
import time
from fsm_waypoint.utils import debug, info, warning, error, critical


class CheckLifecycleStates(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.node = node
        self.nodes_to_check = [
            '/behavior_server',
            '/controller_server',
            '/waypoint_follower',
            '/bt_navigator',
        ]

    def execute(self, userdata):
        userdata.blackboard.wait_spin = True
        time.sleep(0.1)
        # 서비스 클라이언트를 생성
        service_clients = {
            node_name: self.node.create_client(GetState, f"{node_name}/get_state")
            for node_name in self.nodes_to_check
        }

        # 각 서비스가 사용 가능할 때까지 대기
        for node_name, client in service_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                warning(f"Service {node_name}/get_state not available.")
                userdata.blackboard.wait_spin = False
                return 'aborted'

        start_time = time.time()
        outcome = 'aborted'

        while True:
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break

            # 모든 노드 상태 확인
            all_active = True
            for node_name, client in service_clients.items():
                request = GetState.Request()
                try:
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self.node, future)
                    if future.result() is not None:
                        response = future.result()
                        state = response.current_state  # current_state 속성 사용
                        if state.label != 'active':
                            all_active = False
                            info(f"{node_name} is in state: {state.label}")
                            break
                        info(f"Response received: {response}")
                    else:
                        all_active = False
                        warning(f"No response from {node_name}/get_state.")
                        break
                except Exception as e:
                    error(f"Failed to get state for {node_name}: {str(e)}")
                    break

            if all_active:
                info("All nodes are in 'active' state.")
                outcome = 'succeeded'
                break

            # 타임아웃 처리
            if self.timeout:
                if time.time() - start_time > self.timeout:
                    warning("Timeout reached, returning 'aborted'")
                    outcome = 'aborted'
                    break

        userdata.blackboard.wait_spin = False
        return outcome