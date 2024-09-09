#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from my_robot_interfaces.srv import ResetCounter


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.number_subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.reset_counter_service_ = self.create_service(ResetCounter, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self, msg: Int64):
        self.counter_ += msg.data
        self.get_logger().info("Counter:  " + str(self.counter_))

    def callback_reset_counter(self, request: ResetCounter.Request, response: ResetCounter.Response):
        if request.reset_value < 0:
            response.success = False
            response.message = "Cannot reset counter to a negative value"
        elif request.reset_value > self.counter_:
            response.success = False
            response.message = "Reset value must be lower than current counter value"
        else:
            self.counter_ = request.reset_value
            self.get_logger().info("Reset counter to " + str(self.counter_))
            response.success = True
            response.message = "Success"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()