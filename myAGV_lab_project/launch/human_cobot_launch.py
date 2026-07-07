"""
launch/human_cobot_launch.py
Phase 3 real-robot launch — Nav2 + human-in-the-loop mission manager.

No cobot driver is needed: the human helper performs loading/unloading
manually when prompted by the terminal.

Required environment variables (set in ~/.bashrc):
  export STUDENT_ID="yourlastname"
  export DEEPSEEK_API_KEY="sk-..."
  export LANGFUSE_PUBLIC_KEY="pk-lf-..."
  export LANGFUSE_SECRET_KEY="sk-lf-..."
  export LANGFUSE_BASE_URL="https://cloud.langfuse.com"

Launch arguments:
  task         — Natural language task description
  use_fallback — "true" to skip the LLM and use hard-coded PDDL (default: false)
  scenario     — Fallback scenario (default: deliver_A)
  use_astar    — "true" to use A* instead of BFS (default: false)

Examples:
  ros2 launch myagv_lab human_cobot_launch.py \\
      task:="Deliver package_A to the delivery area and return home."

  ros2 launch myagv_lab human_cobot_launch.py \\
      use_fallback:=true scenario:=deliver_AB
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
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

    # ── Launch arguments ──────────────────────────────────────────────────────
    task_arg = DeclareLaunchArgument(
        "task",
        default_value="Deliver package_A to the delivery area and return home.",
        description="Natural language task description for the mission manager",
    )
    fallback_arg = DeclareLaunchArgument(
        "use_fallback",
        default_value="false",
        description="Skip the LLM and use a hard-coded PDDL scenario",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="deliver_A",
        description="Fallback scenario: deliver_A | deliver_AB | recharge_then_deliver",
    )
    astar_arg = DeclareLaunchArgument(
        "use_astar",
        default_value="false",
        description="Use A* instead of BFS in pyperplan",
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":          map_yaml_file,
            "use_sim_time": "false",
            "params_file":  configured_params,
            "autostart":    "true",
        }.items(),
    )

    # ── Human cobot manager (LLM mode, delayed 10 s for Nav2) ─────────────────
    manager_llm = TimerAction(
        period=10.0,
        actions=[Node(
            package="myagv_lab",
            executable="human_cobot_manager",
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("use_fallback"), "' != 'true'"])
            ),
            arguments=[
                "--real",
                "--task",     LaunchConfiguration("task"),
                "--scenario", LaunchConfiguration("scenario"),
            ],
            additional_env={
                "MYAGV_USE_SIM":        "0",
                "DEEPSEEK_API_KEY":     os.environ.get("DEEPSEEK_API_KEY",     ""),
                "LANGFUSE_PUBLIC_KEY":  os.environ.get("LANGFUSE_PUBLIC_KEY",  ""),
                "LANGFUSE_SECRET_KEY":  os.environ.get("LANGFUSE_SECRET_KEY",  ""),
                "LANGFUSE_BASE_URL":    os.environ.get("LANGFUSE_BASE_URL",    ""),
                "STUDENT_ID":           os.environ.get("STUDENT_ID",           ""),
            },
        )],
    )

    # ── Human cobot manager (fallback / no-LLM mode) ──────────────────────────
    manager_fallback = TimerAction(
        period=10.0,
        actions=[Node(
            package="myagv_lab",
            executable="human_cobot_manager",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_fallback")),
            arguments=[
                "--real",
                "--fallback",
                "--scenario", LaunchConfiguration("scenario"),
            ],
            additional_env={
                "MYAGV_USE_SIM": "0",
            },
        )],
    )

    return LaunchDescription([
        task_arg, fallback_arg, scenario_arg, astar_arg,
        nav2, manager_llm, manager_fallback,
    ])
