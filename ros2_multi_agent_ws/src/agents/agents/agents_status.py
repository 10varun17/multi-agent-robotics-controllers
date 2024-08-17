import rclpy 
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import numpy as np

from multi_agent_task_mgmt.msg import Agent, AgentUpdate
from nav_msgs.msg import Odometry

import rclpy.time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from transformations.coordinate_transformations import get_transformation_matrix, transform_to_map

class AgentsStatus(Node):
    def __init__(self):
        super().__init__("agents_status_node")
        self._stat_pub = self.create_publisher(AgentUpdate, "/agents_status", 10)
        self._declare_parameters()
        self._num_agents = self.get_parameter("num_agents").get_parameter_value().integer_value

        self._to_map_frame = "map"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._exec_period = 0.1
        self._rcb = ReentrantCallbackGroup()
        self._odom_topics = {}
        self._is_subbed = {}
        self._odom_subs = {}
        self._ids = {}
        self._odoms = {}

        for num in range(self._num_agents):
            key = str(0) + str(num)
            odom_topic = "/agent_" + key + "/odom"
            self._odom_topics[key] = self._odom_topics.get(key, odom_topic)
            self._is_subbed[key] = self._is_subbed.get(key, False)
            self._odom_subs[key] = self._odom_subs.get(key, self._create_sub(key))
            self._ids[key] = self._ids.get(key, key)
            self._odoms[key] = self._odoms.get(key, Odometry())

        self.get_logger().info("Agents status node started")
        self._timer = self.create_timer(self._exec_period, self._timer_cb)
    
    def _declare_parameters(self):
        num_desc = ParameterDescriptor(
            description="Number of agents",
            type = ParameterType.PARAMETER_INTEGER,
            read_only = True 
        )
        self.declare_parameter("num_agents", None, num_desc)

    def _create_sub(self, agent_key):
        return self.create_subscription(msg_type=Odometry,
                                        topic=self._odom_topics[agent_key],
                                        callback=self._create_cb(agent_key),
                                        qos_profile=10,
                                        callback_group=self._rcb)
    
    def _create_cb(self, agent_key):
        def callback(odom_msg:Odometry):
            self._odoms[agent_key].header.frame_id = odom_msg.header.frame_id
            self._odoms[agent_key].pose.pose.position.x = odom_msg.pose.pose.position.x
            self._odoms[agent_key].pose.pose.position.y = odom_msg.pose.pose.position.y
            self._is_subbed[agent_key] = True
        return callback             
    
    def _timer_cb(self):
        is_subbed = np.array(list(self._is_subbed.values()))
        if np.all(is_subbed):
            agents = []
            for agent_id, odom in zip(self._ids, self._odoms.values()):
                coord = np.array([
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z
                ])
                transform_obj = self._tf_buffer.lookup_transform(
                    target_frame=odom.header.frame_id,
                    source_frame="map",
                    time=rclpy.time.Time()
                )
                transform = transform_obj.transform
                transform_mat = get_transformation_matrix(transform)
                coord_in_map = transform_to_map(transform_mat, coord)
                odom.pose.pose.position.x = coord_in_map[0]
                odom.pose.pose.position.y = coord_in_map[1]

                agent_msg = Agent()
                agent_msg.id = agent_id
                agent_msg.odom_msg = odom
                agents.append(agent_msg)

            agent_update = AgentUpdate()
            agent_update.agents = agents
            self._stat_pub.publish(agent_update)

def main(args = None):
    rclpy.init(args=args)
    agents_status_node = AgentsStatus()
    rclpy.spin(agents_status_node)
    agents_status_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()