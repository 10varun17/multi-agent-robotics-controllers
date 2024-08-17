import sys
import numpy as np
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
from multi_agent_task_mgmt.msg import Result, AgentUpdate

from networked_controllers.controllers import distance_based_formation, leader_controller
from transformations.non_linear_transformations import non_linear_transform
from transformations.coordinate_transformations import get_transformation_matrix, transform_to_local

class LeaderFollowerServer(Node):
    def __init__(self):
        super().__init__("leader_follower_server")
        self._declare_parameters()

        # Create callback groups
        self._mecb = MutuallyExclusiveCallbackGroup()
        self._rcb = ReentrantCallbackGroup()
        
        global agent_id
        agent_id = self.get_parameter("agent_id").get_parameter_value().string_value
        self._action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self._omega_max = self.get_parameter("omega_max").get_parameter_value().double_value
        self._exec_period = self.get_parameter("exec_period").get_parameter_value().double_value
        self._formation_tol = self.get_parameter("formation_tolerance").get_parameter_value().double_value
        self._is_leader = self.get_parameter("is_leader").get_parameter_value().bool_value
        self._leader_control_gain = self.get_parameter("leader_control_gain").get_parameter_value().double_value
        self._formation_control_gain = self.get_parameter("formation_control_gain").get_parameter_value().double_value

        # Create an instance of the action server
        self._leader_follower_server = ActionServer(
            self,
            action_type=TaskOperation,
            action_name=self._action_name,
            execute_callback=self._execute_cb,
            callback_group=self._mecb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb
        )

        if self._is_leader:
            self._waypoints_in_map = np.array([[-2,0,0], [-.5,-1,0], [2.5,-1,0], [2.5,1,0], [-.5,1,0]])
            self._target = 0
            self._proximity = 0.05

        # Flags
        self._is_subbed = False # to wait for the message to be received on the odom topic
        self._is_enabled = False # to start/stop timer callback

        # Initialize the variables to store the information about the agents
        self._agent_id = None
        self._nbr_ids = None # an array of agent's identifiers
        self._curr_loc, self._curr_yaw = None, None
        self._desired_distance = {}

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

        # Create a publisher to the cmd_vel topic and subscription to the odom topic of my agent
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

        # Create a subscription to the neighbor status topic
        self._agents_status_sub = self.create_subscription(
            msg_type=AgentUpdate, topic=self._agents_status_topic, 
            callback=self._agents_status_cb, qos_profile=10, callback_group=self._rcb)

        self._timer = self.create_timer(self._exec_period, self._timer_cb)
        self.get_logger().info(f"Leader Follower Server {agent_id} started with action name {self._action_name}")

        ## Data Analysis
        # Dynamic variables for each agent
        global time_stamp, x, y, lin_vel_inp, omega_inp
        time_stamp = "time_stamp" + agent_id
        x = "x" + agent_id
        y = "y" + agent_id
        lin_vel_inp = "v_input" + agent_id
        omega_inp = "omega_input" + agent_id
        
        # Dictionary to store the data while the formation is in execution
        global data
        data = {
            time_stamp: [],
            x: [],
            y: [],
            lin_vel_inp: [],
            omega_inp: []
        }

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

        leader_desc = ParameterDescriptor(
            description="True if the agent is the leader",
            type= ParameterType.PARAMETER_BOOL,
            read_only= True
        )
        self.declare_parameter("is_leader", False, leader_desc)

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
        self.declare_parameter("omega_max", 1.57, omega_desc)

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

        lg_desc = ParameterDescriptor(
            description="Leader control gain",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("leader_control_gain", 0.7, lg_desc)

        fcg_desc = ParameterDescriptor(
            description="Formation control gain",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("formation_control_gain", 20.0, fcg_desc)

    def _odom_cb(self, odom_msg: Odometry):
        self._is_subbed = True
        self._to_local_frame = odom_msg.header.frame_id
        curr_position = odom_msg.pose.pose.position
        self._curr_loc = np.array([curr_position.x, curr_position.y, curr_position.z])
        curr_quat = odom_msg.pose.pose.orientation
        _, _, self._curr_yaw = tf3d.euler.quat2euler([curr_quat.w, curr_quat.x, curr_quat.y, curr_quat.z])

    def _agents_status_cb(self, agents_status : AgentUpdate):
        for agent_msg in agents_status.agents:
            odom_msg :Odometry = agent_msg.odom_msg
            curr_pos = odom_msg.pose.pose.position
            curr_loc = np.array([curr_pos.x, curr_pos.y, curr_pos.z]) # in map frame
            self._agents_locs_in_map[agent_msg.id] = curr_loc

    def _stop_agent(self):
        self._is_enabled = False
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)
    
    def _goal_cb(self, goal):
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

        # Get the request
        task_name = goal_handle.request.task.task_name
        agent_nbrs = goal_handle.request.task.agent_nbrs            # KeyValue(key:"00", value: "01, 02")
        desired_distance = goal_handle.request.task.desired_distance

        self._agent_id = agent_nbrs.key
        self._nbr_ids = agent_nbrs.value.split(", ")    # Result: ["01", "02"]
        
        # Setup the desired distance between the agents
        for kv in desired_distance:
            self._desired_distance[kv.key] = float(kv.value)

        self._is_enabled = True
        self.get_logger().info(f"Executing the action {task_name}")
        if self._is_leader:
            self.get_logger().info(f"Moving to target {self._target}")

        for my_nbr in self._nbr_ids:
            while my_nbr not in self._agents_locs_in_map.keys():
                time.sleep(0.05)

        taskop_result = TaskOperation.Result()
        goal_handle.succeed()
        result = Result()
        result.result_code = Result.RESULT_NOM
        result.status = "Okay"
        taskop_result.result = result
        return taskop_result

    def _timer_cb(self):
        if self._is_enabled:
            twist_goal = Twist()

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
            desired_dist = []
            for nbr in self._nbr_ids:
                desired_dist.append(self._desired_distance[nbr + "_" + self._agent_id])

            if self._is_leader:
                q_curr_in_map = self._agents_locs_in_map[self._agent_id]
                target_in_map = self._waypoints_in_map[self._target]
                target_loc = transform_to_local(transform_mat, target_in_map)
                distance_to_target = np.linalg.norm(q_curr_in_map - target_in_map)
                ctrl_inp = self._leader_control_gain * (target_loc - q_curr)
                ctrl_inp = leader_controller(q_curr, q_nbrs, desired_dist, target_loc, self._formation_control_gain, self._leader_control_gain)
                if distance_to_target <= self._proximity:
                    self._target = (self._target + 1) % 5
                    self.get_logger().info(f"Moving to target {self._target}")
            else:
                ctrl_inp = distance_based_formation(q_curr, q_nbrs, desired_dist, self._formation_control_gain)

            v, omega = non_linear_transform(ctrl_inp[0], ctrl_inp[1], 0.1, self._curr_yaw)
            v = min(max(-self._v_max, v), self._v_max)
            omega = min(max(-self._omega_max, omega), self._omega_max)

            # Publish the command to my agent's velocity topic
            twist_goal.linear.x = v
            twist_goal.angular.z = omega
            self._cmd_vel_pub.publish(twist_goal)

            # Data analysis purpose
            t_now = time.time_ns()
            q_curr_in_map = self._agents_locs_in_map[self._agent_id]
            data[time_stamp].append(t_now*1e-9)
            data[x].append(q_curr_in_map[0])
            data[y].append(q_curr_in_map[1])
            data[lin_vel_inp].append(v)
            data[omega_inp].append(omega)

            for nbr in self._nbr_ids:
                data["curr" + self._agent_id + "_" + nbr] = data.get("curr" + self._agent_id + "_" + nbr, [])
                data["des" + self._agent_id + "_" + nbr] = data.get("des" + self._agent_id + "_"  + nbr, [])

                q_curr_in_map = np.around(q_curr_in_map, 5)
                q_nbr_in_map = np.around(self._agents_locs_in_map[nbr], 5)

                curr_dist = np.linalg.norm(q_curr_in_map - q_nbr_in_map)
                desired_dist = self._desired_distance[self._agent_id + "_" + nbr]

                data["curr" + self._agent_id + "_" + nbr].append(curr_dist)
                data["des" + self._agent_id + "_" + nbr].append(desired_dist)
                
def main(args=None):
    rclpy.init(args=args)
    try:
        leader_follower = LeaderFollowerServer()
        executor = MultiThreadedExecutor()
        executor.add_node(leader_follower)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit()

    csv_file = "lf_data" + agent_id + ".csv"
    with open(csv_file, "w") as outfile:
        writer = csv.writer(outfile)
        writer.writerow(data.keys())
        writer.writerows(zip(*data.values()))
    leader_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()