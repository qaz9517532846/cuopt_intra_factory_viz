import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("cuopt_intra_factory_viz"), "rviz", "factory_demo.rviz"
    )

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("v_max", default_value="0.5", description="m/s"),
            DeclareLaunchArgument("a_lin", default_value="0.3", description="m/s^2"),
            DeclareLaunchArgument("w_max", default_value="1.0", description="rad/s"),
            DeclareLaunchArgument("a_ang", default_value="1.5", description="rad/s^2"),
            Node(
                package="cuopt_intra_factory_viz",
                executable="waypoint_marker_publisher_node",
                name="waypoint_marker_publisher",
                output="screen",
            ),
            Node(
                package="cuopt_intra_factory_viz",
                executable="vehicle_tf_broadcaster_node",
                name="vehicle_tf_broadcaster",
                output="screen",
                parameters=[
                    {
                        "v_max": LaunchConfiguration("v_max"),
                        "a_lin": LaunchConfiguration("a_lin"),
                        "w_max": LaunchConfiguration("w_max"),
                        "a_ang": LaunchConfiguration("a_ang"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
