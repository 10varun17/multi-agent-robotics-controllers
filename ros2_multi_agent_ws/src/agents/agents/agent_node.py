import rclpy 
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from numpy.random import normal
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class Agent(Node):
    def __init__(self):
        super().__init__("agent_node")

        self._declare_parameters()
        self._agent_id = self.get_parameter("agent_id").get_parameter_value().string_value
        self._noise = self.get_parameter("noise").get_parameter_value().double_value

        self._stat_pub = self.create_publisher(Odometry, "/agent_" + self._agent_id + "/odom", 10)
        self._vel_sub = self.create_subscription(Twist, 
                                                 "/agent_" + self._agent_id + "/commands/velocity", 
                                                 self._vel_cb, 10)
       
        self._exec_period = 0.0
        self._dt = 0.01 # for calculating next x, y, and theta
        self._timer = self.create_timer(self._exec_period, self._timer_cb)

        self._odom = Odometry()
        self._x_next = self._odom.pose.pose.position.x
        self._y_next = self._odom.pose.pose.position.y
        curr_quat = np.array([ 
            self._odom.pose.pose.orientation.x,
            self._odom.pose.pose.orientation.y,
            self._odom.pose.pose.orientation.z,
            self._odom.pose.pose.orientation.w 
        ])
        rot = R.from_quat(curr_quat) # input should be in xyzw
        new_rot = rot.as_euler("xyz", degrees=False)
        self._theta_next = new_rot[2]
        self._odom.header.frame_id = "agent_" + self._agent_id + "/odom"

        self.get_logger().info(f"Agent node {self._agent_id} started!")

    def _declare_parameters(self):
        agent_desc = ParameterDescriptor(
            description="Agent id",
            type = ParameterType.PARAMETER_STRING,
            read_only = True
        )
        self.declare_parameter("agent_id", None, agent_desc)

        noise_desc = ParameterDescriptor(
            description="Noise",
            type = ParameterType.PARAMETER_DOUBLE,
            read_only = True
        )
        self.declare_parameter("noise", 0.0, noise_desc)

    def _timer_cb(self):
        self._stat_pub.publish(self._odom)

    def _vel_cb(self, twist: Twist):
        self.get_logger().info(f"agent_{self._agent_id} received the twist message!")
        v = twist.linear.x
        w = twist.angular.z

        self._x_next += self._dt * (v * np.cos(self._theta_next))
        self._y_next += self._dt * (v * np.sin(self._theta_next))
        self._theta_next += self._dt * w

        noise = normal(0.0, self._noise)
        self._x_next += noise
        self._y_next += noise
        self._theta_next += noise/10

        self._odom.pose.pose.position.x = self._x_next
        self._odom.pose.pose.position.y = self._y_next
        
        rot = R.from_euler('xyz', [0., 0., self._theta_next], degrees=False) # yaw
        new_quat = rot.as_quat() # returns in x, y, z, w
        self._odom.pose.pose.orientation.x = new_quat[0]
        self._odom.pose.pose.orientation.y = new_quat[1]
        self._odom.pose.pose.orientation.z = new_quat[2]
        self._odom.pose.pose.orientation.w = new_quat[3] 

def main(args = None):
    rclpy.init(args=args)
    agent_node = Agent()
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()