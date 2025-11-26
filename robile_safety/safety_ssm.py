### Implement the safety functionalities for the Robile by setting up
### a state machine and implementing all required states here

import rclpy
from rclpy.node import Node
import smach
import smach_ros

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import threading
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger


# Reference: https://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine [but note that the wiki is just for ROS1]


class BaseState(smach.State):
    """Base state class with common functionality"""

    def __init__(self, node: Node, outcomes: list):
        super().__init__(outcomes=outcomes)
        self.node = node
        self.logger = node.get_logger()

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """Helper to publish velocity commands"""
        if not hasattr(self, "cmd_vel_publisher"):
            self.cmd_vel_publisher = self.node.create_publisher(
                Twist, "/cmd_vel", 10
            )

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stop the robot by publishing empty Twist"""
        self.publish_cmd_vel()


class MonitorBatteryAndCollision(BaseState):
    """State to monitor the battery level and possible collisions"""

    def __init__(self, node: Node):
        super().__init__(
            node,
            outcomes=[
                "handle_battery_low",
                "handle_collision",
                "shutdown_requested",
            ],
        )
        self.battery_level = 100
        self.min_distance = float("inf")
        self.collision_distance_threshold = 0.25
        self.battery_threshold = 30.0
        self.shutdown_requested = False

        self.battery_subscriber = node.create_subscription(
            Float32, "/battery_voltage", self.battery_callback, 1
        )
        self.scan_subscriber = node.create_subscription(
            LaserScan, "/scan", self.scan_callback, 1
        )

        # Service to trigger shutdown
        self.shutdown_service = node.create_service(
            Trigger, "/request_shutdown", self.shutdown_callback
        )

        self.logger.info("MonitorBatteryAndCollision state initialized")

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def scan_callback(self, msg):
        self.min_distance = (
            round(min(msg.ranges), 2) if msg.ranges else float("inf")
        )

    def shutdown_callback(self, request, response):
        self.shutdown_requested = True
        response.success = True
        response.message = "Shutdown requested"
        return response

    def execute(self, userdata):
        while rclpy.ok():
            self.logger.warn(
                "Executing MonitorBatteryAndCollision state",
                throttle_duration_sec=1,
            )
            self.logger.info(
                f"Battery level: {self.battery_level}%, Min distance: {self.min_distance}m",
                throttle_duration_sec=1,
            )

            if self.shutdown_requested:
                self.logger.warn(
                    "Shutdown requested! Transitioning to shutdown state"
                )
                self.shutdown_requested = False
                return "shutdown_requested"

            if self.min_distance < self.collision_distance_threshold:
                self.logger.warn(
                    f"Collision detected ({self.min_distance=} < {self.collision_distance_threshold=})! Transitioning to handle_collision"
                )
                return "handle_collision"

            if self.battery_level < self.battery_threshold:
                self.logger.warn(
                    "Battery low! Transitioning to handle_battery_low"
                )
                return "handle_battery_low"

            time.sleep(0.1)


class RotateBase(BaseState):
    """State to rotate the Robile base"""

    def __init__(self, node: Node):
        super().__init__(node, outcomes=["charging"])
        self.logger.info("RotateBase state initialized")

    def execute(self, userdata):
        self.logger.warn(
            "Executing RotateBase state - Starting rotation for charging"
        )
        angular_vel_z = 2.0  # radians per second

        # Start rotating
        self.publish_cmd_vel(angular_z=angular_vel_z)

        return "charging"


class Charging(BaseState):
    """State to monitor battery level while charging (rotating)"""

    def __init__(self, node: Node):
        super().__init__(node, outcomes=["fully_charged"])
        self.logger = node.get_logger()
        self.battery_level = 0
        self.battery_threshold = 30.0

        self.battery_subscriber = node.create_subscription(
            Float32, "/battery_voltage", self.battery_callback, 1
        )
        self.logger.info("Charging state initialized")

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def execute(self, userdata):
        self.logger.warn("Monitoring battery while charging...")

        while rclpy.ok():
            self.logger.info(
                f"Charging... Battery level: {self.battery_level}%",
                throttle_duration_sec=1,
            )

            if self.battery_level >= self.battery_threshold:
                self.logger.info(
                    f"Battery charged! ({self.battery_level}% >= {self.battery_threshold}%). Stopping rotation."
                )
                # Stop rotation with empty Twist
                self.stop_robot()
                return "fully_charged"

            time.sleep(0.1)


class ManualControl(BaseState):
    """State to wait for manual control clearance"""

    def __init__(self, node: Node):
        super().__init__(node, outcomes=["resume_monitoring"])
        self.manual_clearance = False

        # Subscribe to a topic for manual clearance signal
        self.clearance_sub = node.create_subscription(
            Bool, "/manual_clearance", self.clearance_callback, 1
        )

        self.logger.info("ManualControl state initialized")

    def clearance_callback(self, msg):
        self.manual_clearance = msg.data

    def execute(self, userdata):
        # Stop the robot immediately upon entering this state
        self.stop_robot()
        self.logger.warn(
            "Robot stopped due to collision. Waiting for manual control clearance..."
        )
        self.logger.info(
            "Publish 'ros2 topic pub /manual_clearance std_msgs/msg/Bool \"data: true\"' to resume"
        )

        while rclpy.ok():
            if self.manual_clearance:
                self.logger.info(
                    "Manual clearance received. Resuming monitoring."
                )
                self.manual_clearance = False  # Reset flag
                return "resume_monitoring"
            time.sleep(0.5)


class Shutdown(BaseState):
    """State to handle shutdown - stops robot and waits for restart signal"""

    def __init__(self, node: Node):
        super().__init__(node, outcomes=["restart", "exit"])
        self.restart_requested = False

        # Service to restart after shutdown
        self.restart_service = node.create_service(
            Trigger, "/request_restart", self.restart_callback
        )

        self.logger.info("Shutdown state initialized")

    def restart_callback(self, request, response):
        self.restart_requested = True
        response.success = True
        response.message = "Restart requested"
        return response

    def execute(self, userdata):
        # Stop the robot immediately
        self.stop_robot()
        self.logger.warn("Robot shutdown. All motion stopped.")
        self.logger.info(
            "Call 'ros2 service call /request_restart std_srvs/srv/Trigger' to restart"
        )

        while rclpy.ok():
            if self.restart_requested:
                self.logger.info(
                    "Restart signal received. Resuming monitoring."
                )
                self.restart_requested = False
                return "restart"
            time.sleep(0.5)

        return "exit"


def main(args=None):
    """Main function to initialise and execute the state machine"""

    rclpy.init(args=args)
    node = rclpy.create_node("safety_state_machine")
    logger = node.get_logger()

    logger.info("Starting Safety State Machine")
    sm = smach.StateMachine(outcomes=["shutdown"])
    with sm:
        smach.StateMachine.add(
            "MONITOR_BATTERY_AND_COLLISION",
            MonitorBatteryAndCollision(node),
            transitions={
                "handle_battery_low": "ROTATE_BASE",
                "handle_collision": "MANUAL_CONTROL",
                "shutdown_requested": "SHUTDOWN",
            },
        )
        smach.StateMachine.add(
            "ROTATE_BASE",
            RotateBase(node),
            transitions={"charging": "CHARGING"},
        )
        smach.StateMachine.add(
            "CHARGING",
            Charging(node),
            transitions={"fully_charged": "MONITOR_BATTERY_AND_COLLISION"},
        )
        smach.StateMachine.add(
            "MANUAL_CONTROL",
            ManualControl(node),
            transitions={
                "resume_monitoring": "MONITOR_BATTERY_AND_COLLISION",
            },
        )
        smach.StateMachine.add(
            "SHUTDOWN",
            Shutdown(node),
            transitions={
                "restart": "MONITOR_BATTERY_AND_COLLISION",
                "exit": "shutdown",
            },
        )

    # Spin node in background
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    introspection_server = smach_ros.IntrospectionServer(
        "robile_safety_sm", sm, "/SM_ROOT"
    )
    introspection_server.start()
    outcome = sm.execute()
    logger.info(f"State Machine Outcome: {outcome}")

    introspection_server.stop()
    executor.shutdown()
    spin_thread.join(timeout=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
