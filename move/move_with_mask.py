import numpy as np
import open3d as o3d
from sklearn.neighbors import LocalOutlierFactor


def  move(xyz, prob_obj3d,masks ,dir1, dir2,rotation_matrix= np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]]),

         ):


    def filter_outliers(points, contamination=0.05):
        """
        使用 Local Outlier Factor (LOF) 过滤离群点。
        :param points: 3D 点云，形状为 (N, 3) 的 NumPy 数组。
        :param contamination: 离群点比例，默认 5%。
        :return: 过滤后的点云。
        """
        lof = LocalOutlierFactor(n_neighbors=20, contamination=contamination)
        inliers = lof.fit_predict(points) == 1
        return points[inliers],inliers

    def compute_aabb_with_open3d(points):
        """
        使用 Open3D 的 get_axis_aligned_bounding_box 计算轴对齐包围盒 (AABB)。
        :param points: 3D 点云，形状为 (N, 3) 的 NumPy 数组。
        :return: 包围盒的顶点、长宽高和中心点。
        """
        # Step 1: 创建 Open3D 点云对象
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)

        # Step 2: 计算轴对齐包围盒
        aabb = point_cloud.get_axis_aligned_bounding_box()

        # Step 3: 提取包围盒的信息
        min_bound = aabb.get_min_bound()  # 最小边界
        max_bound = aabb.get_max_bound()  # 最大边界
        lengths = max_bound - min_bound  # 长宽高
        center = (min_bound + max_bound) / 2  # 中心点

        # Step 4: 计算 AABB 的 8 个顶点
        corners = np.array([
            [min_bound[0], min_bound[1], min_bound[2]],
            [min_bound[0], min_bound[1], max_bound[2]],
            [min_bound[0], max_bound[1], min_bound[2]],
            [min_bound[0], max_bound[1], max_bound[2]],
            [max_bound[0], min_bound[1], min_bound[2]],
            [max_bound[0], min_bound[1], max_bound[2]],
            [max_bound[0], max_bound[1], min_bound[2]],
            [max_bound[0], max_bound[1], max_bound[2]],
        ])

        return lengths, center, corners


    def orthogonalize_and_normalize(vectors):
        """
        对输入的向量集合进行 Gram-Schmidt 正交化，并归一化。
        :param vectors: 输入向量集合，形状为 (3, 3)，每行为一个向量。
        :return: 正交单位向量集合，形状为 (3, 3)。
        """
        orthogonal_vectors = []
        for i in range(len(vectors)):
            # 从当前向量中减去它在之前所有正交向量上的投影
            v = vectors[i]
            for u in orthogonal_vectors:
                v = v - np.dot(v, u) * u  # 去掉投影部分
            # 如果当前向量的模长接近于 0，说明它与之前的向量线性相关
            if np.linalg.norm(v) < 1e-8:
                raise ValueError("Input vectors are linearly dependent and cannot form a basis.")
            # 归一化
            v = v / np.linalg.norm(v)
            orthogonal_vectors.append(v)
        return np.array(orthogonal_vectors)

    def transform_point_cloud(points, origin,  dir3,dir1,dir2):
        """
        将点云转换到指定的新坐标系。

        :param points: 原始点云，形状为 (N, 3)
        :param origin: 新坐标系的原点，形状为 (3,)
        :param dir1: 新坐标系的 x 轴方向，形状为 (3,)
        :param dir2: 新坐标系的 y 轴方向，形状为 (3,)
        :param dir3: 新坐标系的 z 轴方向，形状为 (3,)
        :return: 转换后的点云，形状为 (N, 3)
        """
        # 确保 dir1, dir2, dir3 构成正交单位基

        basis_vectors = np.array([dir1, dir2, dir3])
        R = orthogonalize_and_normalize(basis_vectors)  # 正交化并归一化

        # 平移后的点云
        points_shifted = points - origin  # 将点云平移到新坐标系的原点

        # 旋转变换 (R.T 将点从全局坐标系转到新坐标系)
        transformed_points = points_shifted @ R.T  # (N, 3) x (3, 3)

        return transformed_points


    # 示例主函数
    def process_points(points, contamination=0.01):
        """
        处理立方体点云，过滤噪声并计算长宽高和中心点。
        :param points: 3D 点云，形状为 (N, 3) 的 NumPy 数组。
        :param contamination: 离群点比例，默认 5%。
        :return: (长宽高, 中心点位置)。
        """
        # 过滤离群点
        filtered_points, inliers = filter_outliers(points, contamination)
        print(f"原始点数: {len(points)}, 过滤后点数: {len(filtered_points)}")

        return filtered_points , inliers

    def transfer_data_from_paras(shape,orientation,position,halfExtents):
        pybullet_params = {
            "shape": shape,
            "position": position.tolist(),
            "orientation": orientation,
            "mass": 1,
            "halfExtents": halfExtents,
            "restitution": 0.9,
            "lateralFriction": 0.2
        }
        return pybullet_params

    def transfer_data(points, ground_points, shape,dir1,dir2,dir3,given_para,gt_position_orientation):
        """
        将点云数据转换为 PyBullet 参数，支持任意方向的物体。

        参数:
            object_points (np.ndarray): 物体点云 (Nx3)。
            ground_points (np.ndarray): 地面点云 (Mx3)。
            shape (str): 物体的形状，比如 'box'。

        返回:
            dict: 包含中心点、长宽高、旋转矩阵、地面中心点的 PyBullet 参数。
        """
        origin = np.mean(ground_points, axis=0)



        new_points = transform_point_cloud(points, origin, dir1, dir2, dir3)

        position=[(np.max(new_points,0)+np.min(new_points,0))[0]/2,(np.max(new_points,0)+np.min(new_points,0))[1]/2,abs((np.max(new_points,0)+np.min(new_points,0))[2])/2+0.1]
        orientation=[0,0,1]
        half_extents=(np.max(new_points,0)-np.min(new_points,0))/2

        # Step 6: 构建PyBullet参数
        pybullet_params = {
                    "shape": shape,
                    "position": position,
                    "orientation": orientation,
                    "mass": 1,
                    "halfExtents": half_extents.tolist(),
                    "restitution": 0.9,
                    "lateralFriction": 0.2
                }




        return pybullet_params

    import numpy as np
    given_paras=True

    translation_vector = dir1
    dir3 = np.cross(dir1, dir2)  # 计算垂直于 dir1 和 dir2 的方向



    # 对保留的点进行旋转和平移




    ground_point=np.array([[-0.896619, 2.151502, 7.364537],
        [-0.892097, 2.277001, 7.288441],
        [-1.687844, 2.677529, 7.149015],
        [-1.780950, 2.620311, 7.228006]])

    dis=[-0.38, -0.57, 0.343]
    second_position=([[-0.821441, 1.361694, 6.057915],
         [-0.887555, 1.276855, 6.120371],
         [-1.788663, 1.643163, 5.831327],
         [-1.738436, 1.743733, 5.772118],
        [-0.896619, 2.151502, 7.364537],
        [-0.892097, 2.277001, 7.288441],
        [-1.687844, 2.677529, 7.149015],
        [-1.780950, 2.620311, 7.228006]])
    if given_paras:
        dir1 = [0, 1, 1.3]  # (俯视)
        dir2 = [1.4, 2.1, -1.6]  # (侧视)
        dir3 = np.cross(dir1, dir2)  # (主视)
        extent= [1.26, 0.19, 2]
        halfExtents = [i/2 for i in extent]
    first_position = np.subtract(second_position, dis)
    positions=np.append([np.array(first_position)],[np.array(second_position)],axis=0)
    position = second_position
    for i in range(0,4):
        position = np.add(position, dis)

        positions =np.append(positions,[np.array(position)],axis=0)
    object_points = positions


    from move.get_position import get_position
    from move.pybullet.pybullet_1 import pybullet_simulate
    from move.move_all_points import move_points
    shape="box"
    all_paras = {}
    object_paras=[]
    positions_orientations = get_position()
    if not given_paras:
        for object_point in enumerate(object_points):
            pybullet_params= transfer_data(object_point, ground_point, shape,dir1,dir2,dir3,given_para,position_orientation)
            object_paras.append(pybullet_params)
    else:
        for keys in positions_orientations:
            position_orientation = positions_orientations[keys]
            orientation = position_orientation["orientation"]
            position = np.add(position_orientation["position"],halfExtents)
            pybullet_params = transfer_data_from_paras(shape,orientation,position,halfExtents)
            object_paras.append(pybullet_params)

    all_paras.update({"objects":object_paras})

    all_paras.update({
        "camera":{
            "distance": 10,
            "yaw": 0,
            "pitch": -10,
            "target": [0.5, 0, 0.1]},
        "video": {
            "width": 640,
            "height": 480,
            "fps": 60,
            "output": "simulation_with_soft_body.avi"
        },
        "frames": 600
    })
    #要改第一个数rolling ，euler_angles =  [roll, pitch, yaw]
    all_paras["objects"][-1]["orientation"] = [0.2, 0, 0]
    position_data=pybullet_simulate(all_paras)
    #position_data第0个物体是桌面，id=1-7，其中 id=7是倾斜的物体
    position_id = 1
    given_order_idx=[22,72,83,96,3,2]#2是倾斜的物体（红色）
    order_points=[]
    for i in given_order_idx:
        for j, mask in masks:
            if i == j:
                mask = mask.bool().squeeze().cpu().numpy()
                points = xyz[mask]
                points, inliers = process_points(points)
                mask[mask] = inliers
                transformed_points=move_points(points, position_data,all_paras,position_id)
                # rotated_points = points @ rotation_matrix.T
                # transformed_points = rotated_points - translation_vector
                xyz[mask] = transformed_points
                position_id+=1

    return xyz