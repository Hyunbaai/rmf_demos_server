# ros2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray

# fsm
import smach

# utils
import time
from datetime import datetime
from fsm_waypoint.utils import debug, info, warning, error, critical


class GetBumperRelease(smach.State):
    def __init__(self, node, timeout=0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'bumper'],
                             input_keys=['blackboard'],
                             output_keys=['blackboard']
                             )
        self.timeout = timeout
        self.sub_ = None
        self.node = node
        self.result = None

    def execute(self, userdata):
        self.sub_ = self.node.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        self.result = None  # initialize result
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
            if self.result:
                outcome = self.result
                break
        self.node.destroy_subscription(self.sub_)
        return outcome

    def diagnostics_callback(self, msg: DiagnosticArray):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # Find the diagnostic message related to "BumperEmergency"
        for status in msg.status:
            if status.name == "BumperEmergency":
                # info(f'[{current_time}] Received diagnostic message: {status.name}')
                # Find and parse the key-value pairs
                for key_value in status.values:
                    if key_value.key == "BumperEmergency":
                        json_data = key_value.value
                        parsed_data = self.parse_data(json_data)
                        self.evaluate_conditions(parsed_data)

    def parse_data(self, json_data):
        # Simple JSON-like string parsing logic (assuming fixed format)
        data = {}
        try:
            # Remove curly braces and split by commas
            json_data = json_data.strip('{}')
            pairs = json_data.split(', ')
            for pair in pairs:
                key, value = pair.split(':')
                key = key.strip('"')  # Remove quotes from key
                value = value.strip('"')  # Remove quotes from value
                data[key] = value
        except Exception as e:
            error(f"Error parsing data: {e}")
        return data

    def evaluate_conditions(self, parsed_data):
        # Check if pressure_pa is <= 0 and switch_state is "Pressed"
        pressure_pa = parsed_data.get("Pressure_PA", "") # not used
        switch_state = parsed_data.get("Switch_State", "")

        if pressure_pa and int(pressure_pa) < 0:
            self.result = 'bumper'
            info(f"Pressure_PA is {pressure_pa}, triggering emergency!")

        if switch_state == "Not_Pressed":
            self.result = 'succeeded'
            info(f"Switch_State is {switch_state}, triggering emergency!")

        if not self.result:
            pass
            #info("No emergency Release conditions met.")
