from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="formation_controller",
            executable="fc_server",
            name="formation_control_server_00",
            parameters=[
                {"agent_id" : "00"},
                {"action_name" : "formation_control_00"},
                {"formation_control_gain" : 1.0}
            ],
            remappings=[
                ("/input/odom", "/agent_00/odom"),
                ("/output/cmd_vel", "/agent_00/commands/velocity")
            ]
        ),
        Node(
            package="formation_controller",
            executable="fc_server",
            name="formation_control_server_01",
            parameters=[
                {"agent_id" : "01"},
                {"action_name" : "formation_control_01"},
                {"formation_control_gain" : 1.0}
            ],
            remappings=[
                ("/input/odom", "/agent_01/odom"),
                ("/output/cmd_vel", "/agent_01/commands/velocity")
            ]
        ),
        Node(
            package="formation_controller",
            executable="fc_server",
            name="formation_control_server_02",
            parameters=[
                {"agent_id" : "02"},
                {"action_name" : "formation_control_02"},
                {"formation_control_gain" : 1.0}
            ],
            remappings=[
                ("/input/odom", "/agent_02/odom"),
                ("/output/cmd_vel", "/agent_02/commands/velocity")
            ]
        )
    ])