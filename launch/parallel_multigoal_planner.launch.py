from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ik_solver_ns = LaunchConfiguration("ik_solver_ns")
    planning_scene_service = LaunchConfiguration("planning_scene_service")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ik_solver_ns", default_value="ik_solver"),
            DeclareLaunchArgument("planning_scene_service", default_value="/get_planning_scene"),
            Node(
                package="parallel_multigoal_planner",
                executable="parallel_multigoal_planner_node",
                name="parallel_multigoal_planner",
                output="screen",
                parameters=[
                    {
                        "ik_solver_ns": ik_solver_ns,
                        "planning_scene_service": planning_scene_service,
                    }
                ],
            ),
        ]
    )
