from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            # PushRosNamespace("robodog"),

            Node(
                package="robodog_gpt",
                executable="control",
                name="control",
                output='screen'
            ),

            Node(
                package="robodog_gpt",
                executable="gpt_node",
                name="gpt_node",
                output='screen'
            ),

            Node(
                package='robodog_gpt',
                executable='audio',
                name='audio',
                output='screen',
            ),

            Node(
                package='robodog_rqt',
                executable='gui_node',
                name='robodog_gui',
                output='screen',
            ),
        ])
    ])
