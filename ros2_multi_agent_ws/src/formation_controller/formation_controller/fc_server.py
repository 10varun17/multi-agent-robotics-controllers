import sys
import numpy as np
from numpy import ndarray
import transforms3d as tf3d
import time
import csv

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from multi_agent_task_mgmt.action import TaskOperation
from multi_agent_task_mgmt.msg import Result, Agent, AgentUpdate
from networked_controllers.controllers import position_based_formation, distance_based_formation
from transformations.non_linear_transformations import non_linear_transform
from transformations.coordinate_transformations import get_transformation_matrix, transform_to_local

class FormationControllerActionServer(Node):
    def __init__(self):
        super().__init__("formation_control_server")
        self._declare_parameters()

        # Create callback groups
        self._mecb = MutuallyExclusiveCallbackGroup()
        self._rcb = ReentrantCallbackGroup()
        
        global agent_id
        agent_id = self.get_parameter("agent_id").get_parameter_value().string_value
        self._action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._is_position_based = self.get_parameter("is_position_based").get_parameter_value().bool_value
        self._v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self._omega_max = self.get_parameter("omega_max").get_parameter_value().double_value
        self._exec_period = self.get_parameter("exec_period").get_parameter_value().double_value
        self._formation_tol = self.get_parameter("formation_tolerance").get_parameter_value().double_value
        self._formation_control_gain = self.get_parameter("formation_control_gain").get_parameter_value().double_value

        # Create an instance of the action server
        self._formation_control_server = ActionServer(
            self,
            action_type=TaskOperation,
            action_name=self._action_name,
            execute_callback=self._execute_cb,
            callback_group=self._mecb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb
        )

        # Flags
        self._is_subbed = False # to wait for the message to be received on the odom topic
        self._is_enabled = False # to start/stop timer callback
        self._formation_achieved = False # to stop the formation control
        self._first_iter = True # first check of the formation control achieved condition

        # Initialize the variables to store the information about the agents
        self._agent_id = None
        self._nbr_ids = None # an array of agent's identifiers
        self._curr_loc = None
        self._curr_yaw = None

        # A dictionary that maps two agents to their desired distance in the formation. eg. {"00_01" : 2.05, "01_02": 2.058}
        self._desired_distance = {}  
        if self._is_position_based:
            self._ksi = {}

        # Initialize variables for publishing and subscription to my agent's cmd vel and odom topics
        self._cmd_vel_topic, self._odom_topic = "/output/cmd_vel", "/input/odom"
        self._cmd_vel_pub, self._odom_sub = None, None

        # Initialize the topic of the NeighborStatus Node
        self._agents_status_topic = "/agents_status"
        self._agents_locs_in_map = {} # a dictionary that maps an agent's id to its current location in the map frame

        # Coordinate transforms
        self._from_map_frame = "map"
        self._to_local_frame = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Create a publisher to the cmd_vel topic and subscription to the odom topic of my agent and to the nbr status
        self._cmd_vel_pub = self.create_publisher(
            msg_type=Twist, 
            topic=self._cmd_vel_topic, 
            qos_profile=10, 
            callback_group=self._rcb
        )
        self._odom_sub = self.create_subscription(
            msg_type=Odometry, 
            topic=self._odom_topic, 
            callback=self._odom_cb, 
            qos_profile=10, 
            callback_group=self._rcb
        )
        self._nbr_status_sub = self.create_subscription(
            msg_type=AgentUpdate, 
            topic=self._agents_status_topic, 
            callback=self._agents_status_cb, 
            qos_profile=10, 
            callback_group=self._rcb
        )

        self._timer = self.create_timer(self._exec_period, self._timer_cb)
        self.get_logger().info(f"Server node {agent_id} started with action name {self._action_name}")

        ### Data Analysis
        # Dynamic variables for each agent
        global data
        self._time_stamp = "time_stamp" + agent_id
        self._x = "x" + agent_id
        self._y = "y" + agent_id
        self._lin_vel_inp = "v_input" + agent_id
        self._omega_inp = "omega_input" + agent_id

        # Dictionary to store the data while the formation is in execution
        data = {
            self._time_stamp: [],
            self._x: [],
            self._y: [],
            self._lin_vel_inp: [],
            self._omega_inp: []
        }

        if self._is_position_based:
            self._tau_x = "tau_x" + agent_id
            self._tau_y = "tau_y" + agent_id
            data[self._tau_x] = data.get(self._tau_x, [])
            data[self._tau_y] = data.get(self._tau_y, [])
    
    def _declare_parameters(self):
        action_name_desc = ParameterDescriptor(
            description="Action name",
            type = ParameterType.PARAMETER_STRING,
            read_only = True
        )
        self.declare_parameter("action_name", None, action_name_desc)

        agent_id_desc = ParameterDescriptor(
            description="Agent id",
            type = ParameterType.PARAMETER_STRING,
            read_only = True
        )
        self.declare_parameter("agent_id", None, agent_id_desc)

        control_law_type_desc = ParameterDescriptor(
            description="True if the control law is position based",
            type = ParameterType.PARAMETER_BOOL,
            read_only = True
        )
        self.declare_parameter("is_position_based", False, control_law_type_desc)

        vel_desc = ParameterDescriptor(
            description="Maximum linear velocity",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("v_max", 0.4, vel_desc)

        omega_desc = ParameterDescriptor(
            description="Maximum angular velocity",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("omega_max", 0.523, omega_desc)

        exec_desc = ParameterDescriptor(
            description="Execution Period",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("exec_period", 0.1, exec_desc)

        form_tol_desc = ParameterDescriptor(
            description="Formation tolerance",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("formation_tolerance", 0.01, form_tol_desc)

        fcg_desc = ParameterDescriptor(
            description="Formation control gain",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("formation_control_gain", 1.0, fcg_desc)

    def _odom_cb(self, odom_msg: Odometry):
        self._is_subbed = True
        self._to_local_frame = odom_msg.header.frame_id
        curr_position = odom_msg.pose.pose.position
        self._curr_loc = np.array([curr_position.x, curr_position.y, curr_position.z])
        curr_quat = odom_msg.pose.pose.orientation
        _, _, self._curr_yaw = tf3d.euler.quat2euler([curr_quat.w, curr_quat.x, curr_quat.y, curr_quat.z])

    def _agents_status_cb(self, agents_status : AgentUpdate):
        """
        This will fill the dictionary self._agents_locs_in_map.
        """
        for agent_msg in agents_status.agents:
            agent_msg:Agent = agent_msg
            odom_msg :Odometry = agent_msg.odom_msg
            curr_pos = odom_msg.pose.pose.position
            curr_loc = np.array([curr_pos.x, curr_pos.y, curr_pos.z]) # in map frame
            self._agents_locs_in_map[agent_msg.id] = curr_loc
    
    def _is_formation_achieved(self):
        if self._first_iter:
            self._first_iter = False
            self._formation_achieved = False # This is the first iteration
        else:
            dist_diff = []

            for nbr in self._nbr_ids:
                data["curr" + self._agent_id + "_" + nbr] = data.get("curr" + self._agent_id + "_" + nbr, [])
                data["des" + self._agent_id + "_" + nbr] = data.get("des" + self._agent_id + "_"  + nbr, [])

                q_curr_in_map = np.around(self._agents_locs_in_map[self._agent_id], 5)
                q_nbr_in_map = np.around(self._agents_locs_in_map[nbr], 5)

                curr_dist = np.linalg.norm(q_curr_in_map - q_nbr_in_map)
                desired_dist = self._desired_distance[self._agent_id + "_" + nbr]

                data["curr" + self._agent_id + "_" + nbr].append(curr_dist)
                data["des" + self._agent_id + "_" + nbr].append(desired_dist)

                dist_diff.append(curr_dist - desired_dist)

            dist_diff = np.array(dist_diff)
            self._formation_achieved = np.all(np.absolute(dist_diff) <= self._formation_tol)
        return self._formation_achieved

    def _stop_agent(self):
        self._is_enabled = False
        self._formation_achieved = True
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)
    
    def _goal_cb(self, goal):
        # Check if all of your neighbors are in present in the communication network
        nbrs_id = goal.task.agent_nbrs.value.split(", ")
        for my_nbr in nbrs_id:
            while my_nbr not in self._agents_locs_in_map.keys():
                time.sleep(0.05)
        self.get_logger().info(f"All agents are in the network of {self.get_name()}!")
        return GoalResponse.ACCEPT
    
    def _cancel_cb(self, goal):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    def _execute_cb(self, goal_handle:ServerGoalHandle):
        # Let the server subscribe to the odom topic
        while not self._is_subbed:
            time.sleep(0.05)

        # Get the request and setup the class variables
        task_name = goal_handle.request.task.task_name
        agent_nbrs = goal_handle.request.task.agent_nbrs            # KeyValue(key:"00", value: "01, 02")
        desired_distance = goal_handle.request.task.desired_distance

        self._agent_id = agent_nbrs.key
        self._nbr_ids = agent_nbrs.value.split(", ")    # Result: ["01", "02"]
        
        # Setup the desired distance (and formation shape) between/among the agents
        for kv in desired_distance:
            self._desired_distance[kv.key] = float(kv.value)

        if self._is_position_based:
            formation_shape = goal_handle.request.task.formation_shape
            for kv in formation_shape:
                x_str, y_str, z_str = kv.value.split(", ")
                self._ksi[kv.key] = np.array([float(x_str), float(y_str), float(z_str)])

        self._is_enabled = True
        # Execute the action
        self.get_logger().info(f"Executing the action {task_name}")
        while not self._formation_achieved:
            for my_nbr in self._nbr_ids:
                if my_nbr not in self._agents_locs_in_map.keys():
                    goal_handle.is_active = False
                    goal_handle.canceled()
                    self.get_logger().info(f"Agent {my_nbr} out of network. Aborting the goal.")
                    return TaskOperation.Result()
            time.sleep(0.05)

        goal_handle.succeed()
        taskop_result = TaskOperation.Result()
        result = Result()
        result.result_code = Result.RESULT_NOM
        result.status = "Okay"
        taskop_result.result = result
        return taskop_result
    
    def _get_log_msg(self):
        curr_loc = self._agents_locs_in_map[self._agent_id]
        if not self._is_position_based:
            msg = f"curr_loc: {curr_loc}\n"
            for nbr in self._nbr_ids:
                curr_dist = np.linalg.norm(curr_loc - self._agents_locs_in_map[nbr])
                key = nbr + "_" + self._agent_id
                dist_msg = f"   desired_dist_to_agent_{nbr}: {self._desired_distance[key]}\n"
                dist_msg += f"   curr_dist_to_agent_{nbr}: {curr_dist}\n\n"
                msg += dist_msg
        else:
            msg = f"curr_loc: {curr_loc}, des_loc: {self._ksi[self._agent_id]}"
        return msg

    def _timer_cb(self):
        if self._is_enabled:
            if not self._is_formation_achieved():
                twist_goal = Twist()
                t_now = time.time_ns()
                
                # Get the transformation matrix
                transform_obj = self._tf_buffer.lookup_transform(
                    target_frame=self._to_local_frame,
                    source_frame=self._from_map_frame,
                    time=rclpy.time.Time()
                )
                transform = transform_obj.transform
                transform_mat = get_transformation_matrix(transform)

                # Current locations
                q_curr = self._curr_loc # already in local frame
                q_nbrs = np.array([transform_to_local(transform_mat, self._agents_locs_in_map[nbr_id]) for nbr_id in self._nbr_ids])
                
                # Apply the control law
                desired_dist = []
                for nbr in self._nbr_ids:
                    desired_dist.append(self._desired_distance[nbr + "_" + self._agent_id])
                ctrl_inp = distance_based_formation(q_curr, q_nbrs, desired_dist, self._formation_control_gain)

                if self._is_position_based:
                    # Desired locations
                    z_ref = np.array(transform_to_local(transform_mat, self._ksi[self._agent_id]))
                    z_nbrs = np.array([transform_to_local(transform_mat, self._ksi[nbr_id]) for nbr_id in self._nbr_ids])
                    ctrl_inp = position_based_formation(q_curr, q_nbrs, z_ref, z_nbrs, self._formation_control_gain)

                # Transform to twist
                v, omega = non_linear_transform(delx=ctrl_inp[0], dely=ctrl_inp[1], x_r=0.1, curr_yaw=self._curr_yaw)
                v = min(max(-self._v_max, v), self._v_max)
                omega = min(max(-self._omega_max, omega), self._omega_max)

                # Publish the command to my agent's velocity topic
                twist_goal.linear.x = v
                twist_goal.angular.z = omega
                self._cmd_vel_pub.publish(twist_goal)

                # Data analysis purpose
                q_curr_in_map = self._agents_locs_in_map[self._agent_id]
                data[self._time_stamp].append(t_now*1e-9)
                data[self._x].append(q_curr_in_map[0])
                data[self._y].append(q_curr_in_map[1])
                data[self._lin_vel_inp].append(v)
                data[self._omega_inp].append(omega)
                if self._is_position_based:
                    z_ref_in_map = self._ksi[self._agent_id]
                    tau = q_curr_in_map - z_ref_in_map
                    data[self._tau_x].append(tau[0])
                    data[self._tau_y].append(tau[1])
            else:
                self._stop_agent()
                self.get_logger().info(f"Final State {self.get_name()}\n{self._get_log_msg()}")
                
def main(args=None):
    rclpy.init(args=args)
    try:
        formation_control = FormationControllerActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(formation_control)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit()
    
    csv_file = "fc_data" + agent_id + ".csv"
    with open(csv_file, "w") as outfile:
        writer = csv.writer(outfile)
        writer.writerow(data.keys())
        writer.writerows(zip(*data.values()))
    formation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()