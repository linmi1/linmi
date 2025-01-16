import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_pointcloud(points, position_data,ori_position_data):
    """
    Transforms the initial point cloud to its new position and orientation,
    considering different coordinate systems for initial and current states.

    Parameters:
    points (numpy.ndarray): The initial point cloud of shape (n, 3).
    ori_position_data (dict): Dictionary containing initial 'orientation' (quaternion as tuple) and 'position' (tuple of x, y, z).
    position_data (dict): Dictionary containing the current 'orientation' (quaternion as tuple) and 'position' (tuple of x, y, z).

    Returns:
    numpy.ndarray: The transformed point cloud of shape (n, 3).
    """
    # Extract initial orientation and position
    ori_orientation = ori_position_data['orientation']  # Quaternion (w, x, y, z)
    ori_position = np.array(ori_position_data['position'])  # Translation (x, y, z)

    # Extract current orientation and position
    current_orientation = position_data['orientation']  # Quaternion (w, x, y, z)
    current_position = np.array(position_data['position'])  # Translation (x, y, z)

    rotation = R.from_euler('zyx', ori_orientation, degrees=False)
    # 转换为四元数 (x, y, z, w)
    ori_orientation = rotation.as_quat()  # 返回顺序为 [x, y, z, w]

    # Convert quaternions to rotation matrices
    ori_rotation = R.from_quat([ori_orientation[1], ori_orientation[2], ori_orientation[3], ori_orientation[0]])
    ori_rotation_matrix = ori_rotation.as_matrix()

    current_rotation = R.from_quat([current_orientation[1], current_orientation[2], current_orientation[3], current_orientation[0]])
    current_rotation_matrix = current_rotation.as_matrix()

    # Step 1: Transform points to the global coordinate system based on ori_position_data
    global_points = np.dot(points, ori_rotation_matrix.T) + ori_position

    # Step 2: Transform global points to the current coordinate system based on position_data
    transformed_points = np.dot(global_points - ori_position, current_rotation_matrix.T) + current_position

    return transformed_points
def move_points(points, position_data,all_paras, position_id):
    frame = len(position_data)
    frame-=1
    ori_position_data = all_paras["objects"][position_id]
    position_data = position_data[frame]["objects"][position_id]
    transformed_points = transform_pointcloud(points, position_data, ori_position_data)
    return transformed_points