import rclpy
from rclpy.node import Node
from teensy_button.action import SendValue
from rclpy.action import ActionClient

class TeensyActionClient(Node):
    def __init__(self):
        super().__init__('teensy_action_client')
        self._action_client = ActionClient(self, SendValue, 'send_value')

    def send_goal(self, value_to_send):
        goal_msg = SendValue.Goal()
        goal_msg.value = value_to_send

        self.get_logger().info(f"Sending goal with value: {value_to_send}")
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.feedback}")

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {'Success' if result.success else 'Failed'}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeensyActionClient()

    # Wait for the action server to be available
    while not node._action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info('Action server not available, waiting again...')

    # Example: send a goal with value 10
    node.send_goal(10)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

