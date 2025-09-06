from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('aeb_k')

    aeb_launch = os.path.join(pkg_share, 'launch', 'aeb_k.launch.py')
    cone_vis_launch = os.path.join(pkg_share, 'launch', 'cone_visualizer.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(aeb_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cone_vis_launch)),
    ])

