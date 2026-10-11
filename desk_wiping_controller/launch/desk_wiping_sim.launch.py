import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_id = LaunchConfiguration("arm_id")
    initial_positions = LaunchConfiguration("initial_positions")
    use_rviz = LaunchConfiguration("use_rviz")

    description_share = get_package_share_directory("franka_description")
    bringup_share = get_package_share_directory("franka_bringup")
    controller_share = get_package_share_directory("desk_wiping_controller")

    robot_xacro = os.path.join(
        description_share, "robots", "sim", "panda_arm_sim.urdf.xacro"
    )
    scene_file = os.path.join(
        description_share, "mujoco", "franka", "desk_wiping_scene.xml"
    )
    rviz_file = os.path.join(
        description_share, "rviz", "visualize_franka.rviz"
    )
    controller_config = os.path.join(
        controller_share, "config", "desk_wiping_sim_controllers.yaml"
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            robot_xacro,
            " arm_id:=",
            arm_id,
            " hand:=true",
            " initial_positions:=",
            initial_positions,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_id",
                default_value="panda",
                description="Name of the Panda arm.",
            ),
            DeclareLaunchArgument(
                "initial_positions",
                default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
                description="Seven initial joint positions enclosed in quotes.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz.",
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(
                        bringup_share,
                        "launch",
                        "sim",
                        "launch_mujoco_ros_server.launch",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "modelfile": scene_file,
                    "verbose": "true",
                    "ns": "",
                    "mujoco_plugin_config": controller_config,
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {
                        "source_list": [
                            "/joint_states",
                            "/panda_gripper_sim_node/joint_states",
                        ],
                        "rate": 30,
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "desk_wiping_controller",
                    "--inactive",
                    "-c",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
