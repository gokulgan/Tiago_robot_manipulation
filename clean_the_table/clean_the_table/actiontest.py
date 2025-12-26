import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from sensor_msgs.msg import LaserScan
from your_package_name.action import DoorDetection  # Import action file


class DoorDetectionNode(Node):
    def __init__(self):
        super().__init__('door_detection_node')

        # Action Server
        self._action_server = ActionServer(
            self,
            DoorDetection,
            'detect_door',
            self.execute_callback
        )

        # Action Client
        self._action_client = ActionClient(self, DoorDetection, 'detect_door')

        # Laser Scan Subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )
        self.latest_scan = None  # Store latest laser scan data

    def scan_callback(self, scan: LaserScan):
        """ Store latest laser scan data """
        self.latest_scan = scan

    def execute_callback(self, goal_handle):
        """ Action execution function (server-side) """
        self.get_logger().info("Door detection started...")

        while rclpy.ok():
            if self.latest_scan is None:
                feedback_msg = DoorDetection.Feedback()
                feedback_msg.feedback = "Waiting for scan data..."
                goal_handle.publish_feedback(feedback_msg)
                continue  # Keep waiting for scan

            front_index = len(self.latest_scan.ranges) // 2
            distance = self.latest_scan.ranges[front_index]

            # Send feedback
            feedback_msg = DoorDetection.Feedback()
            feedback_msg.feedback = f"Scanning... Distance: {distance:.2f}m"
            goal_handle.publish_feedback(feedback_msg)

            # Check if door is open
            if distance > 0.5:
                goal_handle.succeed()
                result = DoorDetection.Result()
                result.door_open = True
                self.get_logger().info("Door is open!")
                return result

        goal_handle.abort()
        result = DoorDetection.Result()
        result.door_open = False
        return result

    def send_goal(self):
        """ Action Client sends goal to detect door """
        self.get_logger().info("Sending goal to detect door...")
        goal_msg = DoorDetection.Goal()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ Handle server response """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """ Handle feedback updates """
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.feedback}")

    def get_result_callback(self, future):
        """ Handle final result """
        result = future.result().result
        if result.door_open:
            self.get_logger().info("Result: The door is OPEN!")
        else:
            self.get_logger().info("Result: The door is CLOSED!")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = DoorDetectionNode()
    node.send_goal()  # Start action client after initialization
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
