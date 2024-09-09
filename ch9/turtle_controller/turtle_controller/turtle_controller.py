#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from my_robot_interfaces.srv import ActivateTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("color_1", [255, 0, 0])
        self.declare_parameter("color_2", [0, 255, 0])
        self.declare_parameter("turtle_velocity", 1.0)
        self.color_1_ = self.get_parameter("color_1").value
        self.color_2_ = self.get_parameter("color_2").value
        self.turtle_velocity_ = self.get_parameter("turtle_velocity").value

        self.is_active_ = True
        self.previous_x_ = 0.0
        self.set_pen_client_ = self.create_client(SetPen, "/turtle1/set_pen")
        self.activate_turtle_service_ = self.create_service(ActivateTurtle, "activate_turtle", self.callback_activate_turtle)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)

    def callback_pose(self, pose: Pose):
        if self.is_active_:
            cmd = Twist()
            if pose.x < 5.5:
                cmd.linear.x = self.turtle_velocity_
                cmd.angular.z = self.turtle_velocity_
            else:
                cmd.linear.x = self.turtle_velocity_ * 2.0
                cmd.angular.z = self.turtle_velocity_ * 2.0
            self.cmd_vel_pub_.publish(cmd)

            if pose.x > 5.5 and self.previous_x_ <= 5.5:
                self.previous_x_ = pose.x
                self.get_logger().info("Set color 1.")
                self.call_set_pen(self.color_1_[0], self.color_1_[1], self.color_1_[2])
            elif pose.x <= 5.5 and self.previous_x_ > 5.5:
                self.previous_x_ = pose.x
                self.get_logger().info("Set color 2.")
                self.call_set_pen(self.color_2_[0], self.color_2_[1], self.color_2_[2])

    def call_set_pen(self, r, g, b):
        while not self.set_pen_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        future = self.set_pen_client_.call_async(request)
        future.add_done_callback(self.callback_set_pen_response)

    def callback_set_pen_response(self, future):
        self.get_logger().info("Successfully changed pen color")

    def callback_activate_turtle(self, request: ActivateTurtle.Request, response: ActivateTurtle.Response):
        self.is_active_ = request.activate
        if request.activate:
            response.message = "Starting the turtle"
        else:
            response.message = "Stopping the turtle"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()