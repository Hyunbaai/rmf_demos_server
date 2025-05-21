# ros2
import rclpy

# fsm
import smach

# utils
import time
import subprocess
import re
import psutil
import os
from fsm_waypoint.utils import debug, info, warning, error, critical


class CheckPingNetwork(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'up', 'down'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard'])
        self.timeout = timeout
        self.node = node
        self.result = None
        self.previous_state = None  # 이전 네트워크 상태 추적
        self.target_ip: str = os.environ.get("TARGET_IPADDRESS")
        self.interface = self.detect_interface(["eth0", "enp4s0", "eno1"])
        self.timer = None
 
    def detect_interface(self, interfaces):
        for interface in interfaces:
            if interface in psutil.net_if_addrs():
                return interface
        error("No valid network interface found. Defaulting to 'eth0'.")
        return "eth0"

    def execute(self, userdata):
        self.timer = self.node.create_timer(1.0, self.ping_and_check)  # 1초마다 ping 체크

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
            # info(f'Network status: {"down" if self.result else "up"}, self.previous_state: {self.previous_state}')
            if self.result is not None and self.result != self.previous_state:
                if self.result:
                    outcome = 'up'  # 네트워크가 복구되었을 때
                else:
                    outcome = 'down'  # 네트워크가 다운되었을 때
                self.previous_state = self.result  # 상태가 변경되었을 때에만 업데이트
                break

        self.node.destroy_timer(self.timer)
        self.timer = None
        return outcome

    def ping_and_check(self):
        # info(f'Checking network status...')
        is_network_up = self.ping_host()
        if self.result is None:
            # 초기 상태 설정
            self.result = is_network_up
            self.previous_state = is_network_up
        elif self.result != is_network_up:
            # 네트워크 상태가 변경되었을 때만 업데이트
            self.result = is_network_up

    def ping_host(self):
        try:
            # info(f'Pinging {self.target_ip}...')
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1.0', self.target_ip],
                capture_output=True,
                text=True,
                check=True  # 비정상 종료 시 예외 발생
            )

            output = result.stdout + result.stderr  # stdout과 stderr 모두 확인
            # info(f'Ping result: {output.strip()}')

            # 출력에 "Destination Host Unreachable" 또는 "Request timeout"이 포함되었는지 확인
            if "Destination Host Unreachable" in output or "Request timeout" in output:
                warning(f"Ping failed: {output.strip()}")
                return False  # 네트워크 실패로 간주

            # 정상적인 ping 응답이 있는지 확인 (64 bytes from ...)
            if "64 bytes from" in output:
                return True  # 네트워크 성공

            return False  # 위 조건을 만족하지 않으면 실패로 간주

        except subprocess.CalledProcessError as e:
            # 비정상 종료 시 발생한 예외 처리
            error(f"Ping command failed: {e}")
            return False  # 네트워크 실패로 간주

        except Exception as e:
            error(f"Unexpected error: {e}")
            return False
