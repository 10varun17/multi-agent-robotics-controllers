import sys
from multi_agent_task_mgmt.action import TaskOperation
from multi_agent_task_mgmt.msg import Task
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from diagnostic_msgs.msg import KeyValue
import collections
from rclpy.action import ActionClient
Tasking = collections.namedtuple("Tasking", ['service', 'task_name'])

class LeaderFollowerClient(Node):

    def __init__(self):
        super().__init__('leader_follower_client')
        self._declare_parameters()

        self._agent_nbrs = KeyValue(
            key = self.get_parameter("agent_id").get_parameter_value().string_value,
            value = self.get_parameter("agent_nbrs").get_parameter_value().string_value
        )
        self._action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._is_leader = self.get_parameter("is_leader").get_parameter_value().bool_value
        self._cli = ActionClient(self, action_type=TaskOperation, action_name=self._action_name) 

        while not self._cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Service not available for {self._action_name}, waiting again...')
        
        self._srv_client_futures = []
        self._prior_tasking : Tasking = None

        self.get_logger().info(f"Leader Follower client node {self._agent_nbrs.key} started with action name {self._action_name}!")

    def _declare_parameters(self):
        agent_id_desc = ParameterDescriptor(
            description="Agent id",
            type = ParameterType.PARAMETER_STRING,
            read_only = True
        )
        self.declare_parameter("agent_id", None, agent_id_desc)

        agent_nbrs_desc = ParameterDescriptor(
            description = "Agent's neighbors",
            type = ParameterType.PARAMETER_STRING,
            read_only = True
        )
        self.declare_parameter("agent_nbrs", None, agent_nbrs_desc)

        action_name_desc = ParameterDescriptor(
            description="Action name",
            type = ParameterType.PARAMETER_STRING,
            read_only = True
        )
        self.declare_parameter("action_name", None, action_name_desc)

        leader_desc = ParameterDescriptor(
            description="True if the agent is the leader",
            type = ParameterType.PARAMETER_BOOL,
            read_only = True
        )
        self.declare_parameter("is_leader", False, leader_desc)

    def _build_task_req(self, desired_distance):
        task_op_req = TaskOperation.Goal()
        task_op_req.task.task_name = self._action_name
        task_op_req.task.agent_nbrs = self._agent_nbrs
        task_op_req.task.formation_shape = []
        task_op_req.task.desired_distance = []
        for key, value in desired_distance.items():
            task_op_req.task.desired_distance.append(KeyValue(key=key, value=value))
        return task_op_req
        
    def _send_task_req(self, service_client, task_req):
        if service_client is not None:
            self._srv_client_futures.append(service_client.send_goal_async(task_req))
            self._prior_tasking = Tasking(service_client, task_req.task.task_name)

def main(args=None):
    rclpy.init(args=args)
    leader_follower_client = LeaderFollowerClient()

    desired_distance ={
        "00_01": "0.25",
        "01_02": "0.25",
        "01_03": "0.25",
        "02_03": "0.25",

        "01_00": "0.25",
        "02_01": "0.25",
        "03_01": "0.25",
        "03_02": "0.25"
    }
    task_req = leader_follower_client._build_task_req(desired_distance)
    leader_follower_client._send_task_req(leader_follower_client._cli, task_req)
    leader_follower_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()