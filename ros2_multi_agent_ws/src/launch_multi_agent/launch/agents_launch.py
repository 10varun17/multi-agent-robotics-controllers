from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():
    for arg in sys.argv:
        if arg.startswith("num_agents:="):
            num_agents = int(arg.split(":=")[1])
            
    ld = LaunchDescription()
    for i in range(num_agents):
        node = Node(
            package="agents",
            executable="agent_node",
            name="agent_node_" + str(0) + str(i),
            parameters=[
                {"agent_id" : str(0) + str(i)}
            ]
        )
        ld.add_action(node)

    return ld