import numpy as np
from scipy.spatial.transform import Rotation as R


# 读取 .txt 文件并解析内容
def parse_transformations(file_path):
    objects = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        current_object = None
        matrix = []
        for line in lines:
            line = line.strip()
            if line.startswith("Object:"):
                if current_object and matrix:
                    objects[current_object] = np.array(matrix)
                current_object = line.split(":")[1].strip()
                matrix = []
            elif line:
                matrix.append([float(x) for x in line.split()])
        if current_object and matrix:
            objects[current_object] = np.array(matrix)
    return objects


# 将变换矩阵转换为 PyBullet 格式
def convert_to_pybullet_format(objects):
    pybullet_data = {}
    for obj_name, matrix in objects.items():
        # 提取位置
        origin_dir=[0,0,1]
        position = matrix[:3, 3]

        # 提取旋转矩阵并转换为欧拉角（ZYX顺序）
        rotation_matrix = matrix[:3, :3]
        # quaternion = np.dot(rotation_matrix,origin_dir)
        quaternion = R.from_matrix(rotation_matrix).as_euler('zyx', degrees=False)
        # quaternion = R.from_matrix(rotation_matrix).as_quat()
        pybullet_data[obj_name] = {
            "position": position.tolist(),
            "orientation": quaternion.tolist()
        }
    return pybullet_data


# 主函数
def get_position():
    input_file = "object_poses.txt"  # 替换为你的 .txt 文件路径
    objects = parse_transformations(input_file)
    pybullet_data = convert_to_pybullet_format(objects)

    # 打印结果
    for obj_name, data in pybullet_data.items():
        print(f"{obj_name}:")
        print(f'  "position": {data["position"]},')
        print(f'  "orientation": {data["orientation"]}')
    return pybullet_data

# 执行主函数
if __name__ == "__main__":
    get_position()
