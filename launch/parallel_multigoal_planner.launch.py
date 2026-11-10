import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ik_solver_ns = LaunchConfiguration("ik_solver_ns")
    planning_scene_service = LaunchConfiguration("planning_scene_service")
    mirrt_star_params = LaunchConfiguration("mirrt_star_params")
    default_mirrt_star_params = os.path.join(
        get_package_share_directory("parallel_multigoal_planner"), "config", "mirrt_star.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ik_solver_ns", default_value="ik_solver"),
            DeclareLaunchArgument("planning_scene_service", default_value="/get_planning_scene"),
            DeclareLaunchArgument(
                "mirrt_star_params",
                default_value=default_mirrt_star_params,
                description="ROS 2 parameter file with the mirrt* solver parameters (under mirrt_star.)",
            ),
            Node(
                package="parallel_multigoal_planner",
                executable="parallel_multigoal_planner_node",
                name="parallel_multigoal_planner",
                output="screen",
                parameters=[
                    mirrt_star_params,
                    {
                        "ik_solver_ns": ik_solver_ns,
                        "planning_scene_service": planning_scene_service,
                    }
                ],
            ),
        ]
    )
