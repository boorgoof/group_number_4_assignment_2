from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ir_gripper", package_name="ir_movit_config").to_moveit_configs()

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    motion_controller_node = Node(
        package="motion_controller",
        executable="robot_manipulator_node",
        name="robot_manipulator",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        motion_controller_node
    ])