import sys
from multi_agent_task_mgmt.action import TaskOperation
from multi_agent_task_mgmt.msg import Task
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import typing
import numpy as np
from diagnostic_msgs.msg import KeyValue
import collections
from rclpy.action import ActionClient
Tasking = collections.namedtuple("Tasking", ['service', 'task_name'])

class FormationControllerActionClient(Node):

    def __init__(self):
        super().__init__("formation_control_client")
        self._declare_parameters()

        self._agent_nbrs = KeyValue(
            key = self.get_parameter("agent_id").get_parameter_value().string_value,
            value = self.get_parameter("agent_nbrs").get_parameter_value().string_value
        )
        self._action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._is_position_based = self.get_parameter("is_position_based").get_parameter_value().bool_value
        self._cli = ActionClient(self, action_type=TaskOperation, action_name=self._action_name) 

        while not self._cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(f'Service not available for {self._action_name}, waiting again...')
        
        self._srv_client_futures = []
        self._prior_tasking : Tasking = None
        self.get_logger().info(f"Client node {self._agent_nbrs.key} started with action name {self._action_name}!")
    
    def _declare_parameters(self):
        agent_id_desc = ParameterDescriptor(
            description="Agent key",
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

        control_law_type_desc = ParameterDescriptor(
            description="True if the control law is position based",
            type = ParameterType.PARAMETER_BOOL,
            read_only = True
        )
        self.declare_parameter("is_position_based", False, control_law_type_desc)
    
    def _build_task_req(self, desired_distance, formation_shape : typing.Dict = None):
        task_op_req = TaskOperation.Goal()
        task_op_req.task.task_name = self._action_name
        task_op_req.task.agent_nbrs = self._agent_nbrs
        task_op_req.task.formation_shape = []
        task_op_req.task.desired_distance = []

        if self._is_position_based:
            for key, value in formation_shape.items():
                task_op_req.task.formation_shape.append(KeyValue(key=key, value=value))

        for key, value in desired_distance.items():
            task_op_req.task.desired_distance.append(KeyValue(key=key, value=value))
        return task_op_req
        
    def _send_task_req(self, service_client, task_req):
        if service_client is not None:
            self._srv_client_futures.append(service_client.send_goal_async(task_req))
            self._prior_tasking = Tasking(service_client, task_req.task.task_name)

def main(args=None):
    rclpy.init(args=args)
    fc_client_node = FormationControllerActionClient()

    ## Position-based formation control
    z0 = np.array([4.64, 1.25, 0.0])
    z1 = np.array([4.24, 2.08, 0.0])
    z2 = np.array([5.24, 2.08, 0.0])
    # formation_shape = {
    #     "00": f"{z0[0]}, {z0[1]}, {z0[2]}",
    #     "01": f"{z1[0]}, {z1[1]}, {z1[2]}",
    #     "02": f"{z2[0]}, {z2[1]}, {z2[2]}",
    # }
    # desired_distance ={
    #     "00_01": f"{np.linalg.norm(z0 - z1)}",
    #     "00_02": f"{np.linalg.norm(z0 - z2)}",
    #     "01_02": f"{np.linalg.norm(z1 - z2)}",
        
    #     "01_00": f"{np.linalg.norm(z0 - z1)}",
    #     "02_00": f"{np.linalg.norm(z0 - z2)}",
    #     "02_01": f"{np.linalg.norm(z1 - z2)}"
    # }

    # Distance-based formation control
    desired_distance ={
        "00_01": "1.0",
        "00_02": "1.0",
        "01_02": "1.0",
        
        "01_00": "1.0",
        "02_00": "1.0",
        "02_01": "1.0"
    }

    task_req = fc_client_node._build_task_req(desired_distance)
    fc_client_node._send_task_req(fc_client_node._cli, task_req)
    fc_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()