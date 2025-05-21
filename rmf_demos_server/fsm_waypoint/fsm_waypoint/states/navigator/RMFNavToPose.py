# ros2
import rclpy
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# nav2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from tf2_ros import Buffer, TransformListener, TransformException
from nav2_msgs.action import NavigateToPose

# rmf
from rmf_fleet_msgs.msg import RobotState, Location, RobotMode, PathRequest

# fsm
import smach

# utils
import time
import math
from fsm_waypoint.utils import debug, info, warning, error, critical
from fsm_waypoint.utils import latLonYaw2Geopose
import uuid
import numpy as np


MODE_MAP = {
    "IDLE": RobotMode.MODE_IDLE,
    "CHARGING": RobotMode.MODE_CHARGING,
    "MOVING": RobotMode.MODE_MOVING,
    "PAUSED": RobotMode.MODE_PAUSED,
    "WAITING": RobotMode.MODE_WAITING,
    "EMERGENCY": RobotMode.MODE_EMERGENCY,
    "GOING_HOME": RobotMode.MODE_GOING_HOME,
    "DOCKING": RobotMode.MODE_DOCKING,
    "ADAPTER_ERROR": RobotMode.MODE_ADAPTER_ERROR,
    "CLEANING": RobotMode.MODE_CLEANING,
}

def robot_to_rmf_yaw(robot_yaw):
    """
    Convert robot yaw to RMF yaw by applying a -90 degree adjustment.
    Args:
        robot_yaw (float): Robot yaw in radians.
    Returns:
        float: RMF yaw in radians.
    """
    return robot_yaw - math.pi  # web에서는 정상
    # return robot_yaw  # rviz2에서 정상

def rmf_to_robot_yaw(rmf_yaw):
    """
    Convert RMF yaw to robot yaw by applying a +90 degree adjustment.
    Args:
        rmf_yaw (float): RMF yaw in radians.
    Returns:
        float: Robot yaw in radians.
    """
    return rmf_yaw + math.pi  # web에서는 정상
    # return rmf_yaw  # rviz2에서 정상

def robot_to_rmf_transform(robot_x, robot_y, reference_coordinates):
    """
    Convert robot coordinates to RMF coordinates using reference points.
    """
    try:
        # Extract points
        robot_points = np.array(reference_coordinates["robot"])
        rmf_points = np.array(reference_coordinates["rmf"])

        # Compute transformation matrix (affine transformation)
        A = np.vstack([robot_points.T, np.ones(len(robot_points))]).T
        B = np.vstack([rmf_points.T, np.ones(len(rmf_points))]).T

        # Solve for transformation matrix using the least squares
        transformation_matrix, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

        # Apply transformation to robot coordinates
        transformed_point = np.dot(transformation_matrix.T, np.array([robot_x, robot_y, 1]))
        return transformed_point[0], transformed_point[1]
    except Exception as e:
        error(f"Error in coordinate transformation: {e}")
        return robot_x, robot_y  # Return original coordinates in case of error

def rmf_to_robot_transform(rmf_x, rmf_y, reference_coordinates):
    """
    Convert RMF coordinates to robot coordinates using affine transformation
    derived from reference points.

    Args:
        rmf_x (float): X coordinate in RMF frame.
        rmf_y (float): Y coordinate in RMF frame.
        reference_coordinates (dict): Reference coordinates with "rmf" and "robot" keys.

    Returns:
        (float, float): Transformed robot coordinates (x, y).
    """
    try:
        # Extract RMF and robot points from reference coordinates
        rmf_points = np.array(reference_coordinates["rmf"])
        robot_points = np.array(reference_coordinates["robot"])

        # Add homogeneous coordinates for affine transformation
        A = np.vstack([rmf_points.T, np.ones(len(rmf_points))]).T
        B = np.vstack([robot_points.T, np.ones(len(robot_points))]).T

        # Compute transformation matrix using least squares
        transformation_matrix, _, _, _ = np.linalg.lstsq(A, B, rcond=None)

        # Apply transformation to RMF coordinates
        rmf_coords = np.array([rmf_x, rmf_y, 1])  # Homogeneous coordinates
        robot_coords = np.dot(transformation_matrix.T, rmf_coords)

        return robot_coords[0], robot_coords[1]
    except Exception as e:
        raise ValueError(f"Error in RMF to robot coordinate transformation: {e}")

def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return {"z": sy, "w": cy}


def convert_waypoint_to_goal(msg):
    try:
        if len(msg.path) < 2:
            warning("Invalid path received. At least 2 waypoints are required (current and destination).")
            return None
        # Current robot position
        current_x = msg.path[0].x
        current_y = msg.path[0].y

        # Target position
        target_x = msg.path[1].x
        target_y = msg.path[1].y
        target_yaw = msg.path[1].yaw  # checking

        # Calculate yaw based on direction to target
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        calculated_yaw = math.atan2(delta_y, delta_x) # 나중에 사용

        # Create PoseStamped with calculated yaw
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        quat = quaternion_from_euler(0, 0, calculated_yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose
    except Exception as e:
        error(f"Error in convert_waypoint_to_goal: {e}")
    return None


class RMFNavToPose(smach.State):
    def __init__(self, node, config, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.last_publish_time = time.time()
        self.path = []
        self.sequence = 0
        self.robot_name = next(iter(config["rmf_fleet"]["robots"].keys()))
        # self.robot_name = "turtlebot3_1"  # gpuserver
        # self.robot_name = "turtlebot3_0"  # nuc #1
        # self.robot_name = "turtlebot3_2"  # nuc #2
        self.target_frame = "map"
        self.from_frame = "base_footprint"
        self.last_request_completed = None
        unique_name = f"rmf_go_to_pose_navigator_{uuid.uuid4().hex}"
        self.navigator = BasicNavigator(unique_name)
        self.timeout = timeout
        self.node = node
        self.sub_ = None
        self.ongoing_request_cmd_id = None  # Current task ID.
        self.ongoing_goal_handle = None  # New variable to store GoalHandle
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.result_goal = None

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.pub_ = self.node.create_publisher(RobotState,
                                               '/robot_state',
                                               qos_profile=transient_qos)

        self.reference_coordinates = config["fleet_manager"]["reference_coordinates"]["L1"]
        #self.timer = self.node.create_timer(0.01, self.update_robot_state)

    def send_goal(self, destination_pose, task_id):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            error("Action server not available for navigate_to_pose")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = destination_pose

        self.ongoing_request_cmd_id = task_id
        self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def cancel_goal(self, ongoing_request_cmd_id):
        try:
            if self.ongoing_goal_handle:  # Check if there's an ongoing goal
                future = self.ongoing_goal_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)
            else:
                warning("No ongoing goal to cancel.")
        except Exception as e:
            error(f"Failed to send cancel request({ongoing_request_cmd_id}): {e}")


    def cancel_done(self, future):
        try:
            result = future.result()
            if len(result.goals_canceling) > 0:
                info('Goal successfully canceled.')
            else:
                info('No goals were canceled.')
        except Exception as e:
            error(f'Cancel goal failed: {e}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            warning(f"Goal for task_id {self.ongoing_request_cmd_id} was rejected")
            self.ongoing_request_cmd_id = None
            return

        info(f"Goal for task_id {self.ongoing_request_cmd_id} accepted")
        self.ongoing_goal_handle = goal_handle  # Save the GoalHandle
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 3 or result.status == 4:  # SUCCEEDED
            info(f"Navigation succeeded for task_id {self.ongoing_request_cmd_id}")
        elif result.status == 6:  # UNKNOWN
             warning(f"Navigation UNKNOWN for task_id {self.ongoing_request_cmd_id}")
             # todo: unknown일 경우 path?, ongoing_request_cmd_id?, ongoing_goal_handle? 초기화
             self.ongoing_goal_handle = None
             self.result_goal = MODE_MAP["WAITING"]
             return
        else:
            warning(f"Navigation ended with status {result.status} for task_id {self.ongoing_request_cmd_id}")

        self.last_request_completed = self.ongoing_request_cmd_id
        self.ongoing_request_cmd_id = None
        self.ongoing_goal_handle = None
        self.path = []

    def execute(self, userdata):
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE  # 데이터 지속성을 없앰
        )
        self.sub_ = self.node.create_subscription(PathRequest,
                                                  '/robot_path_requests',
                                                  self.path_request_cb,
                                                  qos_profile=transient_qos
                                                  )
        self.ongoing_request_cmd_id = None  # reset ongoing_request_cmd_id
        self.path = [] # reset path
        userdata.blackboard.wait_spin = True
        time.sleep(0.1)

        start_time = time.time()
        outcome = 'aborted'
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                self.navigator.cancelTask()
                error('preempted: navigator.cancelTask()')
                outcome = 'preempted'
                break

            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except IndexError as e:
                error(f"IndexError: {e}")
                time.sleep(0.1)
            except rclpy._rclpy_pybind11.InvalidHandle:
                error('Node handle is invalid.')
                time.sleep(0.1)

            if self.timeout:
                if time.time() - start_time > self.timeout:
                    self.navigator.cancelTask()
                    error('timeout: navigator.cancelTask()')
                    outcome = 'succeeded'
                    break
            # if self.ongoing_request_cmd_id is not None and \
            #         self.navigator.isTaskComplete():
            #     result = self.navigator.getResult()
            #     if result == TaskResult.SUCCEEDED:
            #         pass
            #         # info(
            #         #     f"Navigation succeeded for task_id: {self.ongoing_request_cmd_id}")
            #
            #     elif result == TaskResult.FAILED:
            #         warning(
            #             f"Navigation failed for task_id: {self.ongoing_request_cmd_id}")
            #     else:
            #         warning(
            #             f"Navigation canceled for task_id: {self.ongoing_request_cmd_id}")

                # debug(f'Navigator result: {result}')
                # outcome = 'succeeded'
                # break

            # if self.ongoing_request_cmd_id is not None and \
            #             not self.navigator.isTaskComplete():
            #     pass
            #
            #     # feedback = self.navigator.getFeedback()
            #
            #     # if feedback and feedback.current_pose:
            #     #     # PoseStamped에서 pose 속성을 사용하여 position에 접근
            #     #     robot_x = feedback.current_pose.pose.position.x
            #     #     robot_y = feedback.current_pose.pose.position.y
            #     #     theta = math.atan2(
            #     #         feedback.current_pose.pose.orientation.z,
            #     #         feedback.current_pose.pose.orientation.w
            #     #     )
            #     #
            #     #     # Transform to RMF coordinates if necessary
            #     #     rmf_x, rmf_y = robot_to_rmf_transform(robot_x, robot_y, self.reference_coordinates)
            #     #     # info(f"Robot position from feedback: RMF coordinates x={rmf_x}, y={rmf_y}, theta={theta}")
            #     #     self.makeup_robot_state_publisher(rmf_x, rmf_y, theta)
            #
            #     # if feedback and feedback.distance_remaining:
            #     #     debug(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
            # else:
            #     # Check if the ongoing task is complete
            #     if self.ongoing_request_cmd_id is not None and self.navigator.isTaskComplete():
            #         info(
            #             f'Robot [{self.robot_name}] completed task with cmd id '
            #             f'[{self.ongoing_request_cmd_id}]'
            #         )
            #         self.last_request_completed = self.ongoing_request_cmd_id
            #         self.ongoing_request_cmd_id = None
            #         self.path = []  # Clear the path upon completion

            self.update_robot_state()  # <= 조건에 상관없이 진행

        userdata.blackboard.wait_spin = False
        self.node.destroy_subscription(self.sub_)
        return outcome

    def update_robot_state(self):
        transform_stamped = self.get_current_transform()
        if transform_stamped:
            # debug(f"Current transform: {transform_stamped}")
            # theta = math.atan2(
            #     2.0 * (transform_stamped.transform.rotation.w * transform_stamped.transform.rotation.z),
            #     1.0 - 2.0 * (transform_stamped.transform.rotation.z ** 2)
            # )
            orientation = euler_from_quaternion([
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w
            ])

            robot_x = transform_stamped.transform.translation.x
            robot_y = transform_stamped.transform.translation.y

            rmf_x, rmf_y = robot_to_rmf_transform(robot_x, robot_y, self.reference_coordinates)
            target_yaw_rmf = robot_to_rmf_yaw(orientation[2])
            self.makeup_robot_state_publisher(rmf_x, rmf_y, target_yaw_rmf)

            # info(f"RobotState details: name={robot_state_msg.name}, "
            #      f"task_id={robot_state_msg.task_id}, "
            #      f"path={robot_state_msg.path}, "
            #      f"battery_percent={robot_state_msg.battery_percent}")

            time.sleep(0.01)

    def makeup_robot_state_publisher(self, rmf_x, rmf_y, theta):
        """
        Publish the current robot state to the /robot_state topic.
        :param rmf_x: X-coordinate in RMF reference frame.
        :param rmf_y: Y-coordinate in RMF reference frame.
        :param theta: Orientation (yaw) of the robot in radians.
        """

        # Calculate the time since the last publish
        # current_time = time.time()
        # time_since_last_publish = current_time - self.last_publish_time
        #
        # # If less than 0.1 seconds (10Hz), sleep for the remaining time
        # if time_since_last_publish < 0.1:
        #     time.sleep(0.1 - time_since_last_publish)
        #
        # # Update the last publish time
        # self.last_publish_time = time.time()

        # Create current location
        current_location = Location()
        current_location.x = rmf_x
        current_location.y = rmf_y
        current_location.yaw = theta
        current_location.t = self.node.get_clock().now().to_msg()
        current_location.level_name = "L1"  # Example: Static level name

        # Determine robot mode
        current_mode = RobotMode()
        current_mode.mode = self.determine_mode()

        # Create RobotState message
        robot_state_msg = RobotState()
        robot_state_msg.name = self.robot_name
        robot_state_msg.battery_percent = 100.0
        robot_state_msg.location = current_location
        robot_state_msg.task_id = str(self.ongoing_request_cmd_id) if self.ongoing_request_cmd_id else str(self.last_request_completed)
        robot_state_msg.mode = current_mode
        robot_state_msg.seq = self.sequence = self.sequence + 1
        robot_state_msg.path = [self.path[1]] if len(self.path) > 1 else []

        # Publish the state
        # debug(f"Publishing robot state: {robot_state_msg}")
        self.pub_.publish(robot_state_msg)

    # todo: stop 처리는 path[0], path[1]이 같은 경우로 일치시에 navigator.cancelTask() 호출
    def path_request_cb(self, msg):
        try:
            # info(f"Received PathRequest with task_id: {msg.task_id}")
            # Check if the task is already being processed
            if self.ongoing_request_cmd_id and self.ongoing_request_cmd_id == msg.task_id:
                # warning(
                #     f"[{self.robot_name}] already processing task_id [{self.ongoing_request_cmd_id}] -- continuing as normal"
                # )
                if self.ongoing_request_cmd_id:  # msg.task_id가 같지 않더라도 ongoing_request_cmd_id가 있으면 cancel 했는데 그러면
                    # ongoing_request_cmd_id를 None이 아닌 경우가 꼭 complete task_id가 아니라도 있을수 있으므로 msg.task_id가 같은 경우 cancel_goal() 호출
                    warning("Another navigation task is already in progress. Cancelling previous task.")
                    self.cancel_goal(self.ongoing_request_cmd_id)
                return

            # Process the path request
            if len(msg.path) < 2:
                warning("Invalid path received. At least 2 waypoints are required (current and destination).")
                return


            self.path = msg.path

            # todo: if not is_complete_path, cancelTask()
            # Check if a task is already running
            # if not self.navigator.isTaskComplete():
            #     warning("Another navigation task is already in progress. Cancelling previous task.")
            #     self.navigator.cancelTask()  # Cancel the previous task
            #     # 이전 작업이 완전히 취소될 때까지 대기
            #     timeout = 5.0  # 최대 대기 시간 (초)
            #     start_time = time.time()
            #
            #     while True:
            #         # navigator의 상태를 확인
            #         if self.navigator.isTaskComplete():
            #             debug("Previous task successfully cancelled.")
            #             break
            #
            #         if time.time() - start_time > timeout:
            #             warning("Task cancellation timed out. Aborting new task.")
            #             return
            #         time.sleep(0.1)  # 반복 간격

            # transform rmf to robot
            current_x_rmf = msg.path[0].x
            current_y_rmf = msg.path[0].y
            msg.path[0].x, msg.path[0].y = rmf_to_robot_transform(current_x_rmf, current_y_rmf,
                                                                  self.reference_coordinates)
            target_x_rmf = msg.path[1].x
            target_y_rmf = msg.path[1].y
            msg.path[1].x, msg.path[1].y = rmf_to_robot_transform(target_x_rmf, target_y_rmf,
                                                                  self.reference_coordinates)

            target_yaw_robot = rmf_to_robot_yaw(msg.path[1].yaw)
            msg.path[1].yaw = target_yaw_robot
            info(f'Received PathRequest with task_id: {msg.task_id}, target_x_rmf: {target_x_rmf:.3f}, target_y_rmf: {target_y_rmf:.3f}, target_yaw: {msg.path[1].yaw:.3f}')

            destination_pose = convert_waypoint_to_goal(msg)
            if destination_pose is None:
                warning("Failed to generate a valid destination pose. Aborting navigation.")
                return

            # Update the current task ID and start navigation
            # self.ongoing_request_cmd_id = msg.task_id
            # self.navigator.goToPose(destination_pose)

            self.send_goal(destination_pose, msg.task_id)

            # debug(f"Started navigation to task_id: {msg.task_id}")
        except Exception as e:
            error(f"Error in path_request_cb: {e}")

    def get_current_transform(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.from_frame,
                rclpy.time.Time()
            )
            return transform_stamped
        except TransformException as ex:
            warning(f"Could not transform {self.from_frame} to {self.target_frame}")
            return None

    def determine_mode(self):
        if not self.path or len(self.path) < 2:
            self.result_goal = MODE_MAP["IDLE"]
        elif self.result_goal == MODE_MAP["WAITING"]:
            return self.result_goal
        else:
            self.result_goal = MODE_MAP["MOVING"]

        return self.result_goal