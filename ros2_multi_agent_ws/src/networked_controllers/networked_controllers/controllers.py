import numpy as np

def rendezvous(q_agent, q_nbrs, w_nbrs):
    """
    Implements the basic rendezvous equation for one agent assuming linear dynamics.

    q_agent : NDArray
        Agents are represented in a real plane or volume (R^2, R^3).
    q_nbrs : NDArray
        All neighbors have the same dimension as q_agent.
    w_nbrs : NDArray
        The weight between q_agent and each ith neighbor, e.g. w_nbrs[0]
        is the weight on this edge: q_agent---w[0]---q_nbrs[0].

    Returns:
    u : Sequence
        Single integrator control inputs for the agent q_agent.
    """
    agent_dim = len(q_agent)
    # Init the control inputs
    u = np.zeros((agent_dim,))
    for q_nbr, w_nbr in zip(q_nbrs, w_nbrs):
        diff = w_nbr * (q_agent - q_nbr)
        u = u + diff
    return -u

def position_based_formation(q_agent, q_nbrs, z_ref, z_nbrs, formation_control_gain=1.0):
    """
    Implements the relative positions based formation controller.

    q_agent : Sequence
        Agents are represented in a real plane or volume (R^2, R^3).
    q_nbrs : Sequence[Sequence]
        All neighbors have the same dimension as q_agent.
    z_ref : Sequence
        The desired position for q_agent in the formation shape.
    z_nbrs : Sequence[Sequence]
        The desired positions for the neighbors of q_agent in the formation shape.
    formation_control_gain: float
        control gain for the algorithm

    Returns:
    u : Sequence
        Single integrator control inputs for the agent q_agent.
    """
    agent_dim = len(q_agent)
    u = np.zeros((agent_dim,))
    for q_nbr, z_nbr in zip(q_nbrs, z_nbrs):
        u += formation_control_gain*((q_agent - q_nbr) - (z_ref - z_nbr))
    return -u

def distance_based_formation(q_agent, q_nbrs, desired_dist, formation_control_gain = 1):
    """
    Implements the distance-based formation controller.

    Args:
    q_agent : Sequence
        Agents are represented in a real plane or volume (R^2, R^3).
    q_nbrs : Sequence[Sequence]
        All neighbors have the same dimension as q_agent.
    desired_dist : Sequence of floats
        An array of distance between neigbor j and agent i. Eg. [d_10, d_20, d_30]
        Length of desired_dist is equal to the length of q_nbrs.
    formation_control_gain: float
        control gain for the algorithm

    Returns:
    u : Sequence
        Single integrator control inputs for the agent q_agent.
    """
    agent_dim = len(q_agent)
    u = np.zeros((agent_dim, ))
    for q_nbr, q_nbr, dist_nbr_agent in zip(q_nbrs, q_nbrs, desired_dist):
        relative_position = q_nbr - q_agent
        inter_agent_dist_error = np.power(np.linalg.norm(q_nbr - q_agent), 2) - np.power(dist_nbr_agent, 2)
        u += formation_control_gain * inter_agent_dist_error * relative_position
    return u

def leader_controller(q_agent, q_nbrs, desired_dist, q_target, formation_control_gain=1.0, leader_control_gain=1.0):
    """
    Implements the distance-based formation controller.

    Args:
    q_agent : Sequence
        Agents are represented in a real plane or volume (R^2, R^3).
    q_nbrs : Sequence[Sequence]
        All neighbors have the same dimension as q_agent.
    desired_dist : Sequence of floats
        An array of distance between neigbor j and agent i. Eg. [d_10, d_20, d_30]
        Length of desired_dist is equal to the length of q_nbrs.
    q_target:
        the target loccation for the leader
    formation_control_gain: float
        control gain for the algorithm
    leader_control_gain: float
        control gain for the algorithm

    Returns:
    u : Sequence
        Single integrator control inputs for the agent q_agent.
    """
    agent_dim = len(q_agent)
    u = np.zeros((agent_dim, ))
    for q_nbr, q_nbr, dist_nbr_agent in zip(q_nbrs, q_nbrs, desired_dist):
        relative_position = q_nbr - q_agent
        inter_agent_dist_error = np.power(np.linalg.norm(q_nbr - q_agent), 2) - np.power(dist_nbr_agent, 2)
        u += formation_control_gain * inter_agent_dist_error * relative_position + leader_control_gain * (q_target - q_agent)
    return u