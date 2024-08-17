from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="formation_controller",
            executable="fc_client",
            name="formation_control_client_00",
            parameters=[
                {"agent_id" : "00"},
                {"action_name" : "formation_control_00"},
                {"agent_nbrs" : "01, 02"}
            ]
        ),
        Node(
            package="formation_controller",
            executable="fc_client",
            name="formation_control_client_01",
            parameters=[
                {"agent_id" : "01"},
                {"action_name" : "formation_control_01"},
                {"agent_nbrs" : "00, 02"}
            ]
        ),
        Node(
            package="formation_controller",
            executable="fc_client",
            name="formation_control_client_02",
            parameters=[
                {"agent_id" : "02"},
                {"action_name" : "formation_control_02"},
                {"agent_nbrs" : "00, 01"}
            ]
        )
    ])