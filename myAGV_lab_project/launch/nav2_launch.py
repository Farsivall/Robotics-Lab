"""
launch/nav2_launch.py
Real-robot navigation launch (ROS2 Galactic).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    nav2_dir  = get_package_share_directory("nav2_bringup")
    myagv_dir = get_package_share_directory("myagv_lab")

    map_yaml_file = os.path.join(myagv_dir, "maps", "lab_map.yaml")
    params_file   = os.path.join(myagv_dir, "config", "nav2_params.yaml")

    # On Galactic, bringup_launch.py's `map` launch argument does not
    # reliably propagate into map_server's `yaml_filename` param, so bake
    # the map path into the params file directly instead of relying on it.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"yaml_filename": map_yaml_file},
        convert_types=True,
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map":          map_yaml_file,
                "use_sim_time": "false",
                "params_file":  configured_params,
                "autostart":    "true",
            }.items(),
        ),
    ])
