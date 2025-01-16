import numpy as np
from colmap_utils import read_model, write_model, rotmat_to_qvec

# 配置路径
model_path = "/home/vision/work/linmi2/gaussian_grouping2/data/color_gupai/distorted/sparse/ori"  # 输入 COLMAP 模型路径
output_path = "/home/vision/work/linmi2/gaussian_grouping2/data/color_gupai/distorted/sparse/0"  # 输出修改后的模型路径
main_image_name = "frame_Camera_1.png"  # 主轴视角图像名



# 读取 COLMAP 模型
cameras, images, points3D = read_model(model_path, ext=".bin")

# 找到主轴视角对应的图像
main_image_id = None
for image_id, image in images.items():
    if image.name == main_image_name:
        main_image_id = image_id
        main_image = image
        break

if main_image_id is None:
    raise ValueError(f"{main_image_name} not found in the COLMAP model.")

# 获取主轴视角的旋转矩阵和位移
R_main = main_image.qvec2rotmat()
t_main = main_image.tvec

# 计算主图像到世界坐标系的逆变换
R_main_inv = R_main.T
t_main_inv = -R_main_inv @ t_main

# 更新所有图像的位姿
print("Updated image poses:")
for image_id, image in images.items():
    R_image = image.qvec2rotmat()
    t_image = image.tvec

    # 变换到新坐标系
    R_new = R_main_inv @ R_image
    t_new = R_main_inv @ t_image + t_main_inv

    # 创建新的 Image 对象，替换旧的
    new_image = image._replace(qvec=rotmat_to_qvec(R_new), tvec=t_new)
    images[image_id] = new_image

    # 输出结果
    if image_id == main_image_id or image_id < 5:  # 仅输出主视角和部分图像
        print(f"Image ID: {image_id}")
        print(f"New qvec: {new_image.qvec}")
        print(f"New tvec: {new_image.tvec}\n")

# 更新所有三维点的坐标
print("Updated 3D points:")
for i, (point_id, point) in enumerate(points3D.items()):
    new_xyz = R_main_inv @ point.xyz + t_main_inv

    # 创建新的 Point3D 对象，替换旧的
    new_point = point._replace(xyz=new_xyz)
    points3D[point_id] = new_point

    # 输出部分结果
    if i < 5:  # 仅输出前5个三维点
        print(f"Point ID: {point_id}")
        print(f"New xyz: {new_point.xyz}\n")

# 保存修改后的模型
write_model(cameras, images, points3D, output_path, ext=".bin")

print(f"Model successfully updated. New model saved to {output_path}.")
