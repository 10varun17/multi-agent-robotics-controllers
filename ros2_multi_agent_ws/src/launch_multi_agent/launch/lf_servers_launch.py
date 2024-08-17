from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="leader_follower",
            executable="lf_server",
            name="leader_follower_server_00",
            parameters=[
                {"agent_id" : "00"},
                {"action_name" : "leader_follower_00"},
                {"is_leader" : True},
                {"leader_control_gain" : 0.7}
            ],
            remappings=[
                ("/input/odom", "/agent_00/odom"),
                ("/output/cmd_vel", "/agent_00/commands/velocity")
            ]
        ),
        Node(
            package="leader_follower",
            executable="lf_server",
            name="leader_follower_server_01",
            parameters=[
                {"agent_id" : "01"},
                {"action_name" : "leader_follower_01"},
                {"formation_control_gain" : 20.0}
            ],
            remappings=[
                ("/input/odom", "/agent_01/odom"),
                ("/output/cmd_vel", "/agent_01/commands/velocity")
            ]
        ),
        Node(
            package="leader_follower",
            executable="lf_server",
            name="leader_follower_server_02",
            parameters=[
                {"agent_id" : "02"},
                {"action_name" : "leader_follower_02"},
                {"formation_control_gain" : 20.0}
            ],
            remappings=[
                ("/input/odom", "/agent_02/odom"),
                ("/output/cmd_vel", "/agent_02/commands/velocity")
            ]
        ),
        Node(
            package="leader_follower",
            executable="lf_server",
            name="leader_follower_server_03",
            parameters=[
                {"agent_id" : "03"},
                {"action_name" : "leader_follower_03"},
                {"formation_control_gain" : 20.0}
            ],
            remappings=[
                ("/input/odom", "/agent_03/odom"),
                ("/output/cmd_vel", "/agent_03/commands/velocity")
            ]
        )
    ])