from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ik_solver_ns = LaunchConfiguration("ik_solver_ns")
    collision_check_service = LaunchConfiguration("collision_check_service")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ik_solver_ns", default_value="ik_solver"),
            DeclareLaunchArgument("collision_check_service", default_value="/collision_check"),
            Node(
                package="parallel_multigoal_planner",
                executable="parallel_multigoal_planner_node",
                name="parallel_multigoal_planner",
                output="screen",
                parameters=[
                    {
                        "ik_solver_ns": ik_solver_ns,
                        "collision_check_service": collision_check_service,
                    }
                ],
            ),
        ]
    )
