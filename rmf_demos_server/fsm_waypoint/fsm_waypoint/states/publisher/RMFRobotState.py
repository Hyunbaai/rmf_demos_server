
# ros2
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from tf2_ros import Buffer, TransformListener, TransformException

# fsm
import smach

# rmf
from rmf_fleet_msgs.msg import RobotState, Location, RobotMode

# utils
import time
import math
from fsm_waypoint.utils import debug, info, warning, error, critical
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


class RMFRobotState(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.last_request_completed = None
        self.robot_name = "turtlebot3_1"  # Example: Static robot name
        self.target_frame = "map"
        self.from_frame = "base_footprint"
        self.sequence = 0
        self.timeout = timeout
        self.node = node


        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.pub_ = self.node.create_publisher(RobotState,
                                               '/robot_state',
                                               QoSProfile(depth=10))
        self.reference_coordinates = {
            "rmf": [
                [8.9508, -6.6006],
                [7.1006, -9.1508],
                [12.3511, -9.2008],
                [11.0510, -11.8010]
            ],
            "robot": [
                [-1.04555, 2.5456],
                [-2.90519, 0.00186],
                [2.39611, -0.061773],
                [1.08783, -2.59750]
            ]
        }

    def execute(self, ud):

        start_time = time.time()
        outcome = 'succeeded' # because timeout not mandatory
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                outcome = 'preempted'
                break

            if self.timeout:
                if time.time() - start_time > self.timeout:
                    outcome = 'succeeded'
                    break

            transform_stamped = self.get_current_transform()
            if transform_stamped:
                # debug(f"Current transform: {transform_stamped}")
                theta = math.atan2(
                    2.0 * (transform_stamped.transform.rotation.w * transform_stamped.transform.rotation.z),
                    1.0 - 2.0 * (transform_stamped.transform.rotation.z ** 2)
                )
                robot_x = transform_stamped.transform.translation.x
                robot_y = transform_stamped.transform.translation.y

                rmf_x, rmf_y = robot_to_rmf_transform(robot_x, robot_y, self.reference_coordinates)

                current_location = Location()
                current_location.x = rmf_x
                current_location.y = rmf_y
                current_location.yaw = theta
                current_location.t = self.node.get_clock().now().to_msg()
                current_location.level_name = "L1"  # Example: Static level name
                current_mode = RobotMode()

                current_mode.mode = MODE_MAP["MOVING"]
                if ud.blackboard.ongoing_request_cmd_id is None:
                    current_mode.mode= MODE_MAP["IDLE"]

                robot_state_msg = RobotState()
                robot_state_msg.name = self.robot_name
                robot_state_msg.battery_percent = 100.0
                robot_state_msg.location = current_location
                robot_state_msg.task_id = str(ud.blackboard.last_request_completed) if ud.blackboard.last_request_completed else ""
                robot_state_msg.mode = current_mode
                robot_state_msg.seq = self.sequence = self.sequence + 1
                robot_state_msg.path = []

                self.pub_.publish(robot_state_msg)

                # info(f"RobotState details: name={robot_state_msg.name}, "
                #      f"task_id={robot_state_msg.task_id}, "
                #      f"path={robot_state_msg.path}, "
                #      f"battery_percent={robot_state_msg.battery_percent}")
                time.sleep(0.01)

        return outcome

    def get_current_transform(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.from_frame,
                rclpy.time.Time()
            )
            return transform_stamped
        except TransformException as ex:
            warning(f"Could not transform {self.from_frame} to {self.target_frame}: {ex}")
            return None
