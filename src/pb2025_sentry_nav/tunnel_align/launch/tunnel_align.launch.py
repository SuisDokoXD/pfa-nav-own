import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory("tunnel_align")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_cboard_emulator = LaunchConfiguration("use_sim_cboard_emulator")
    params_file = LaunchConfiguration("params_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="red_standard_robot1",
                description="Robot namespace (empty string for un-namespaced reality bringup).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use Gazebo /clock time.",
            ),
            DeclareLaunchArgument(
                "use_sim_cboard_emulator",
                default_value="True",
                description="Run Gazebo C-board protocol emulator (set False on real robot with gimbal_bridge).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_dir, "config", "tunnel_align_params.yaml"),
                description="Full path to tunnel_align parameter YAML.",
            ),
            GroupAction(
                [
                    PushRosNamespace(namespace=namespace),
                    SetRemap("/tf", "tf"),
                    SetRemap("/tf_static", "tf_static"),
                    Node(
                        package="tunnel_align",
                        executable="tunnel_align_supervisor_node",
                        name="tunnel_align_supervisor",
                        output="screen",
                        parameters=[configured_params, {"use_sim_time": use_sim_time}],
                    ),
                    Node(
                        condition=IfCondition(use_sim_cboard_emulator),
                        package="tunnel_align",
                        executable="sim_cboard_emulator_node",
                        name="sim_cboard_emulator",
                        output="screen",
                        parameters=[configured_params, {"use_sim_time": use_sim_time}],
                    ),
                ]
            ),
        ]
    )
