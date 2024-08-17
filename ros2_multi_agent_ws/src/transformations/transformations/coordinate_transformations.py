from geometry_msgs.msg import Transform
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_transformation_matrix(transform: Transform):
    """
    Returns a transformation matrix from a Transform object

    Args:
    transform: Transform object with source (map) frame and target (local) frame

    Returns:
    a transformation matrix from the map frame to the local frame
    """
    quat = transform.rotation
    quat_arr = np.array([quat.x, quat.y, quat.z, quat.w])
    translation = transform.translation
    rot_mat = R.from_quat(quat_arr).as_matrix()
    translation_vect = np.array([translation.x, translation.y, translation.z]).reshape(3,1)
    transform_mat = np.hstack([rot_mat, translation_vect])
    return np.vstack([transform_mat, np.array([0., 0., 0., 1.])])

def transform_to_local(transform_mat, coord):
    """
    Transforms coordinates from the map frame to the local frame

    Args:
    transform_mat: transformation matrix from the map frame to the local frame
    coord (N,): coordinates in the map frame

    Returns:
    coordinates (N,) in the local frame
    """
    coord = np.concatenate((coord, np.array([1])), axis=0).reshape(4,1)
    transformed_coord = (transform_mat@coord).ravel()
    return np.delete(transformed_coord, -1, 0)

def transform_to_map(transform_mat, coord):
    """
    Transforms coordinates from the local frame to the map frame

    Args:
    transform_mat: transformation matrix from the map frame to the local frame
    coord (N,): coordinates in the local frame

    Returns:
    coordinates (N,) in the map frame
    """
    coord = np.concatenate((coord, np.array([1])), axis=0).reshape(4,1)
    transformed_coord = (np.linalg.inv(transform_mat)@coord).ravel()
    return np.delete(transformed_coord, -1, 0)