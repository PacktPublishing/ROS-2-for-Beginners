#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ResetCounter


class ResetCounterClientNode(Node):
    def __init__(self):
        super().__init__("reset_counter_client")
        self.client_ = self.create_client(ResetCounter, "reset_counter")

    def call_reset_counter(self, value):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        request = ResetCounter.Request()
        request.reset_value = value
        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_reset_counter_response)

    def callback_reset_counter_response(self, future):
        response = future.result()
        self.get_logger().info("Success flag: " + str(response.success))
        self.get_logger().info("Message: " + str(response.message))
 

def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClientNode()
    node.call_reset_counter(20)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
