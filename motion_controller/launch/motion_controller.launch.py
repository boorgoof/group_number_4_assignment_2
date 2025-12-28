from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ir_gripper", package_name="ir_movit_config").to_moveit_configs()

    motion_controller_node = Node(
        package="motion_controller",
        executable="robot_manipulator_node",
        name="robot_manipulator",
        parameters=[
            {"use_sim_time": True},
            moveit_config.to_dict(),
        ],
        output="screen",
    )

    return LaunchDescription([motion_controller_node])
