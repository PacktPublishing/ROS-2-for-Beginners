from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    turtle_params = os.path.join(
        get_package_share_directory("my_robot_bringup"), 
        "config", "turtle_params.yaml")
    
    turtlesim_1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="/t1",
        parameters=[turtle_params]
    )
    
    turtle_controller_1 = Node(
        package="turtle_controller",
        executable="turtle_controller",
        namespace="/t1",
        parameters=[turtle_params],
        remappings=[
            ("/turtle1/pose", "/t1/turtle1/pose"),
            ("/turtle1/cmd_vel", "/t1/turtle1/cmd_vel"),
            ("/turtle1/set_pen", "/t1/turtle1/set_pen")
        ]
    )

    turtlesim_2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="/t2",
        parameters= [turtle_params]
    )
    
    turtle_controller_2 = Node(
        package="turtle_controller",
        executable="turtle_controller",
        namespace="/t2",
        parameters=[turtle_params],
        remappings=[
            ("/turtle1/pose", "/t2/turtle1/pose"),
            ("/turtle1/cmd_vel", "/t2/turtle1/cmd_vel"),
            ("/turtle1/set_pen", "/t2/turtle1/set_pen")
        ]
    )

    ld.add_action(turtlesim_1)
    ld.add_action(turtle_controller_1)
    ld.add_action(turtlesim_2)
    ld.add_action(turtle_controller_2)
    return ld
