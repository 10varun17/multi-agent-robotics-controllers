from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="leader_follower",
            executable="lf_client",
            name="leader_follower_client_00",
            parameters=[
                {"agent_id" : "00"},
                {"action_name" : "leader_follower_00"},
                {"agent_nbrs" : "01"},
                {"is_leader" : True},
            ]
        ),
        Node(
            package="leader_follower",
            executable="lf_client",
            name="leader_follower_client_01",
            parameters=[
                {"agent_id" : "01"},
                {"action_name" : "leader_follower_01"},
                {"agent_nbrs" : "00, 02, 03"}
            ]
        ),
        Node(
            package="leader_follower",
            executable="lf_client",
            name="leader_follower_client_02",
            parameters=[
                {"agent_id" : "02"},
                {"action_name" : "leader_follower_02"},
                {"agent_nbrs" : "01, 03"}
            ]
        ),
        Node(
            package="leader_follower",
            executable="lf_client",
            name="leader_follower_client_03",
            parameters=[
                {"agent_id" : "03"},
                {"action_name" : "leader_follower_03"},
                {"agent_nbrs" : "01, 02"}
            ]
        )
    ])