# file: speed_planner/launch/curvature_speed_planner.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("speed_planner")
    default_cfg = os.path.join(pkg_share, "config", "curvature_speed_planner.yaml")

    cfg_arg = DeclareLaunchArgument("config", default_value=default_cfg)
    cfg = LaunchConfiguration("config")

    node = Node(
      package="speed_planner",
      executable="curvature_speed_planner_node",
      name="speed_planner",
      output="screen",
      parameters=[cfg],
      # remappings=[
      #   ("/local_planned_path", "/your/path/topic"),
      #   ("/current_speed", "/your/current_speed"),
      #   ("/desired_speed_profile", "/your/profile/topic"),
      #   ("/cmd/speed", "/your/cmd/topic"),
      # ]
    )

    return LaunchDescription([cfg_arg, node])
