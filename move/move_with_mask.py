import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors

from sklearn.neighbors import LocalOutlierFactor

def  move(xyz, mask ,translation_vector,rotation_matrix= np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]]),

         ):
    mask = mask.bool().squeeze().cpu().numpy()

    import numpy as np
    from sklearn.neighbors import NearestNeighbors

    # 假设您的输入数据
    # xyz: 包含所有点的 ndarray (12000, 3)
    # mask: 用于选择某些点的布尔数组，与 xyz 的长度相同
    # rotation_matrix: 旋转矩阵 (3, 3)
    # translation_vector: 平移向量 (3,)
    points = xyz[mask]

    # 对保留的点进行旋转和平移



    def filter_outliers(points, contamination=0.05):
        """
        使用 Local Outlier Factor (LOF) 过滤离群点。
        :param points: 3D 点云，形状为 (N, 3) 的 NumPy 数组。
        :param contamination: 离群点比例，默认 5%。
        :return: 过滤后的点云。
        """
        lof = LocalOutlierFactor(n_neighbors=20, contamination=contamination)
        inliers = lof.fit_predict(points) == 1
        return points[inliers]

    def compute_bounding_box(points):
        """
        计算点云的轴对齐包围盒 (AABB)。
        :param points: 3D 点云，形状为 (N, 3) 的 NumPy 数组。
        :return: 长、宽、高和中心点。
        """
        min_coords = points.min(axis=0)  # 每个轴的最小值
        max_coords = points.max(axis=0)  # 每个轴的最大值
        lengths = max_coords - min_coords  # 长宽高
        center = (max_coords + min_coords) / 2  # 中心点
        return lengths, center

    # 示例主函数
    def process_points(points, contamination=0.05):
        """
        处理立方体点云，过滤噪声并计算长宽高和中心点。
        :param points: 3D 点云，形状为 (N, 3) 的 NumPy 数组。
        :param contamination: 离群点比例，默认 5%。
        :return: (长宽高, 中心点位置)。
        """
        # 过滤离群点
        filtered_points = filter_outliers(points, contamination)
        print(f"原始点数: {len(points)}, 过滤后点数: {len(filtered_points)}")

        # 计算包围盒
        lengths, center = compute_bounding_box(filtered_points)
        print(f"立方体的长宽高: {lengths}, 中心点: {center}")
        return lengths, center

    # 示例点云数据


    # 调用主函数
    process_points(points)


    rotated_points = points @ rotation_matrix.T
    transformed_points = rotated_points - translation_vector

    # 更新 xyz 中的非奇异点
    xyz[mask] = transformed_points

    # 最终更新后的 mask 是 new_mask


    # xyz[mask, 0] += move*  dir[0]
    # xyz[mask, 1] +=  move*  dir[2]
    # xyz[mask, 2] +=  move*  dir[1]
    return xyz