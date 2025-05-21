import time
import subprocess
import ast
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState
from threading import Event, Thread
from rmf_demos_tasks.utils.logger2 import info, warning, error, debug

class PatrolManager(Node):
    def __init__(self, domain_id, fleet_name, robots, locations):
        super().__init__('patrol_manager')
        self.domain_id = domain_id
        self.fleet_name = fleet_name
        self.robots = robots
        self.locations = locations
        self.current_task_id = None  # 공유 작업 ID
        self.task_completed_event = Event()  # 공유 작업 완료 이벤트

        # Subscribe to fleet states to monitor task progress
        self.subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.fleet_state_callback,
            10
        )

    def fleet_state_callback(self, msg):
        for robot in msg.robots:
            # 작업이 완료된 경우: task_id가 ''로 변경
            if robot.task_id == '' and self.current_task_id is not None:
                self.get_logger().info(f"Robot {robot.name} has completed task {self.current_task_id}")
                self.current_task_id = None  # 현재 작업 ID 초기화
                self.task_completed_event.set()  # 이벤트 트리거

    def run_dispatch_patrol(self, robot_name):
        while rclpy.ok():
            # Get the next location for the current robot
            current_location = self.locations.pop(0)
            self.locations.append(current_location)

            # Dispatch the patrol task
            self.get_logger().info(f"Dispatching patrol for {robot_name} to {current_location}")
            self._dispatch_task(robot_name, current_location)

            # Wait for the shared task to complete
            self.task_completed_event.wait()
            self.task_completed_event.clear()  # Reset the event

            # 작업 완료 후 대기 시간 설정
            self.get_logger().info(f"Robot {robot_name} completed task. Waiting before the next task...")
            time.sleep(2)  # 2초 대기

    def _dispatch_task(self, robot_name, location):
        cmd = [
            'bash', '-c',
            f'export ROS_DOMAIN_ID={self.domain_id} && '
            f'ros2 run rmf_demos_tasks dispatch_patrol '
            f'-F {self.fleet_name} -R {robot_name} -p {location}'
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            self.get_logger().info(f"Command executed: {result.stdout}")

            # Extract JSON-like response from the output
            try:
                output = result.stdout
                if 'Got response:' in output:
                    # Extract the response after "Got response:"
                    response_str = output.split('Got response:')[1].strip()

                    # Safely evaluate the response string
                    response = ast.literal_eval(response_str)
                    if response.get("success"):
                        self.current_task_id = response["state"]["booking"]["id"]  # 공유 작업 ID 설정
                        info(f"New task ID: {self.current_task_id}")
                    else:
                        error(f"Task dispatch for {robot_name} was not successful.")
                else:
                    error("No 'Got response:' found in the output.")
            except (ValueError, SyntaxError) as e:
                error(f"Failed to parse response: {e}")
        else:
            error(f"Failed to dispatch patrol for {robot_name}: {result.stderr}")


def main(args=None):
    rclpy.init(args=args)

    domain_id = 3
    fleet_name = "sim_bot"
    robots = ["tinybot1", "tinybot2", "tinybot3"]
    locations = [
        "coe",
        "pantry",
        "patrol_D2",
        "patrol_A1",
        "patrol_D1",
        "pantry",
        "lounge",
        "coe",
        "tinyRobot3_charger",
        "patrol_C",
        "patrol_B",
        "supplies",
        "patrol_A2",
        "tinyRobot2_charger",
        "tinyRobot1_charger",
    ]

    patrol_manager = PatrolManager(domain_id, fleet_name, robots, locations)

    # Start a separate thread for each robot
    threads = []
    for robot_name in robots:
        thread = Thread(target=patrol_manager.run_dispatch_patrol, args=(robot_name,))
        threads.append(thread)
        thread.start()

    try:
        rclpy.spin(patrol_manager)  # Spin to handle subscriptions
    except KeyboardInterrupt:
        patrol_manager.get_logger().info("Shutting down patrol manager...")
    finally:
        patrol_manager.destroy_node()
        rclpy.shutdown()

        # Wait for all threads to finish
        for thread in threads:
            thread.join()


if __name__ == '__main__':
    main()
