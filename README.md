# Multi-agent Control
This repository contains the formation controller and leader follower controller for multi agents. This repo is an older version of the controllers that I developed during my summer research at the robotics lab at the University of Richmond. 
### agents

A ROS2 package that includes python scripts to create virtual agents (`agent_node.py`) and to create a network to communicate agents's status (current_position and current_yaw) between the controllers and the agents.

### launch_multi_agent

A ROS2 package that contains launch files for launching the agents, controllers, and the clients that send the task information to the controllers.

### multi_agent_task_mgmt

A ROS2 package that develops the message types required for the data transfer between the controllers and the agents_status node.

### networked_controllers
The package that maintains the control laws used by different controllers in the HIVE lab.

### transformations
The package that maintains the coordinate transformations between the `map` frame and the robot's local `odom` frame and the non-linear transformation to transform single integrator inputs to unicycle inputs.

### support_python
Data analysis scripts to simulate the formation control between three agents and leader follower control between four agents.


### Cloning into the repo
Clone this git repository using the following command:

```
git clone https://github.com/10varun17/multi-agent-robotics-controllers.git
```

### Building the packages
From the root of the repo `multi-agent-robotics-controllers`, navigate to the root of the ROS2 workspace.

```
cd ros2_multi_agent_ws
```

Build the packages.
```
colcon build
```

Source `setup.bash`.
```
source /install/setup.bash
```

### Creating new terminals
We need a lot of terminals for running the controllers, and there's one thing common in all the terminals. Every new terminal should have `ros2_multi_agent_ws` as the current working directory. After navigating to the directory, we'll need to source the `setup.bash` file for each terminal. Moving onwards, whenever it's specified to create a new terminal, don't forget to run the following command in the `ros2_multi_agent_ws` workspace.

```
source /install/setup.bash
```

### Creating the transformations
Before creating the agents, we need to set up the transformations between the map frame and the agent's local frame. The controllers are configured to control the agents by using static transformations between the `map` frame and the agent's local frame `agent_xx/odom`. To set up the static transforms, we run the following command:

```ROS2
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll map agent_xx/odom
```

`x, y, z` are the coordinates of the agent's initial position in the `map` frame and `yaw` is the rotation about z-axis in the `map` frame. The controllers are configured for 2D motion, so `z` must always be zero as well as `pitch` and `roll`. `yaw` should be in radians. An example of the correct command to setup the transform between the `map` frame and the agent `00`'s local frame is:

```ROS2
ros2 run tf2_ros static_transform_publisher 3 -2 0 1.57 0 0 map agent_00/odom
```

For `n` agents, we need to create `n` terminals (remember to source the `setup.bash` file in `ros2_multi_agent_ws` directory) and static transforms by running the same command, with changes in the initial position, yaw, and name of the agent's local frame (`agent_xx/odom`).

### Creating the agents
Before creating the agents, we need to run the following command to launch the `agents_status_node`. This node is the communication bridge between the agents and the controllers. Open a new terminal (last reminder to source `setup.bash` file) and run the following command:

```ROS2
ros2 run agents agents_status_node --ros-args -p "num_agents:=n"
```

`n` is the number of agents. While the controllers can be used with any number of agents, the clients are currently configured to run with 3 agents for formation control and 4 agents for leader follower. To run the controllers for other number of agents, we need to change the parameters for the clients and the goal request sent by the clients to the controllers. More on it shortly.

A correct example of this command for formation control with 3 agents is:

```ROS2
ros2 run agents agents_status_node --ros-args -p "num_agents:=3"
```

We're now ready to create the agents. To do so, open a new terminal and run the following command:

```ROS2
ros2 launch launch_multi_agent agents_launch.py num_agents:=n
```

It's recommended to have the same value for `n` while running the commands for `agents_launch.py` and `agents_status_node`. Unlike the command for `agents_status_node`, double quotes are optional for `agents_launch.py`.

### Setting up and starting the controllers 
To launch the controller tasks, run the following commands as per the need.

#### Formation Control
Open a new terminal and run
```ROS2
ros2 launch launch_multi_agent fc_tasks_launch.py
```
Open a new terminal and run
```ROS2
ros2 launch launch_multi_agent fc_clients_launch.py
```

#### Leader Follower
Open a new terminal and run
```ROS2
ros2 launch launch_multi_agent lf_tasks_launch.py
```
Open a new terminal and run
```ROS2
ros2 launch launch_multi_agent lf_clients_launch.py
```

## Changes needed for running the controllers in `n` number of agents

If we need to run the controllers for `n` not equal to 3 for formation control and 4 for leader follower, we need to change the clients for the formation control task and the leader follower task as well as the client and task launch files for the respective tasks.

### Formation Control
Currently, the formation control client is configured to control 3 agents with position-based and distance-based formation control. 

#### Position-based formation control
Edit the following lines of code in `multi-agent-robotics-controllers/ros2_multi_agent_ws/src/formation_controller/formation_controller/fc_client.py` as per the requirement.

```python
### Position based
z0 = np.array([4.64, 1.25, 0.0])
z1 = np.array([4.24, 2.08, 0.0])
z2 = np.array([5.24, 2.08, 0.0])
desired_locations = {
    "00": (z0[0], z0[1], z0[2]),
    "01": (z1[0], z1[1], z1[2]),
    "02": (z2[0], z2[1], z2[2]),
}
desired_distances ={
    "00_01": np.linalg.norm(z0 - z1),
    "00_02": np.linalg.norm(z0 - z2),
    "01_02": np.linalg.norm(z1 - z2),
    
    "01_00": np.linalg.norm(z0 - z1),
    "02_00": np.linalg.norm(z0 - z2),
    "02_01": np.linalg.norm(z1 - z2)
}
```
Note:- For `desired_distances` dictionary, it's mandatory to have all the permutations of distances. For example, the distance between agents `i` and `j`, `d_ij` is equal to the distance between agents `j` and `i`, `d_ji`, but we need to provide the key value pairs for both distances.

Since the formation controller is configured to use the distance-based formation by default, we need to make changes to the `_declare_parameters` function in `fc_client.py` and `fc_task.py` python files if we want to use the position-based formation. In `_declare_parameters` function of `fc_client.py` and `fc_task.py`, change the default value of the parameter `is_position_based` to `True`:
```python
self.declare_parameter("is_position_based", True, control_law_type_desc)
```

#### Distance-based formation control
Edit the following lines of code in `multi-agent-robotics-controllers/ros2_multi_agent_ws/src/formation_controller/formation_controller/fc_client.py` as per the requirement.

```python
### Distance based
desired_distances ={
    "00_01": 1.0,
    "00_02": 1.0,
    "01_02": 1.0,
    
    "01_00": 1.0,
    "02_00": 1.0,
    "02_01": 1.0
}
```
Note:- For `desired_distances` dictionary, it's mandatory to have all the permutations of distances. For example, the distance between agents `i` and `j`, `d_ij` is equal to the distance between agents `j` and `i`, `d_ji`, but we need to provide the key value pairs for both distances.

### Leader Follower
Edit the following lines of code in `multi-agent-robotics-controllers/ros2_multi_agent_ws/src/leader_follower/leader_follower/lf_client.py` as per the requirement.

```python
### Distance based
desired_distances ={
    "00_01": "0.25",
    "01_02": "0.25",
    "01_03": "0.25",
    "02_03": "0.25",

    "01_00": "0.25",
    "02_01": "0.25",
    "03_01": "0.25",
    "03_02": "0.25"
}
```

Note:- For `desired_distances` dictionary, it's mandatory to have all the permutations of distances. For example, the distance between agents `i` and `j`, `d_ij` is equal to the distance between agents `j` an `i`, `d_ji`, but we need to provide the key value pairs for both distances.

### Client's launch file change
In `multi-agent-robotics-controllers/ros2_multi_agent_ws/src/launch_multi_agent/launch/`, open the launch files for respective tasks and make the changes to the current node parameters and add new nodes as per the property of the new graph with `n` agents. The parameters should be self-explanatory in the launch files.

**⚠️ IMPORTANT:** After making the changes, start all the process again from the section **`Building the packages`.**