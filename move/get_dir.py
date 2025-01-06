def get_dirs(name_image1,name_image2,images_bin_path  ,
    cameras_bin_path
             ):
    import numpy as np
    from move.read_write_model import read_images_binary, read_cameras_binary

    def quaternion_to_rotation_matrix(qw, qx, qy, qz):
        """Convert quaternion to rotation matrix."""
        q = np.array([qw, qx, qy, qz])
        q = q / np.linalg.norm(q)  # Ensure unit quaternion
        w, x, y, z = q
        R = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
        return R

    def compute_axis_differences(images):
        results = []

        # 获取第一张图片的四元数
        first_image = list(images.values())[0]
        first_rotation = quaternion_to_rotation_matrix(*first_image.qvec)  # 解包 qvec

        # 提取第一张图片的三个轴方向
        first_x_axis = first_rotation[:, 0]
        first_y_axis = first_rotation[:, 1]
        first_z_axis = first_rotation[:, 2]

        # 遍历其余图片，计算三个轴方向差异
        for image_id, image in images.items():
            current_rotation = quaternion_to_rotation_matrix(*image.qvec)
            current_x_axis = current_rotation[:, 0]
            current_y_axis = current_rotation[:, 1]
            current_z_axis = current_rotation[:, 2]

            # 计算与第一帧 Z 轴的夹角
            cos_theta = np.dot(first_z_axis, current_z_axis) / (np.linalg.norm(first_z_axis) * np.linalg.norm(current_z_axis))
            angle_with_first_z_axis = np.arccos(np.clip(cos_theta, -1.0, 1.0)) * (180.0 / np.pi)  # 转换为角度

            results.append({
                'image_id': image_id,
                'image_name': image.name,
                'x_axis': current_x_axis,
                'y_axis': current_y_axis,
                'z_axis': current_z_axis,
                'angle_with_first_z_axis': angle_with_first_z_axis
            })

        return results

    # Main script


    # Load binary files
    images = read_images_binary(images_bin_path)
    cameras = read_cameras_binary(cameras_bin_path)

    # Compute axis differences
    axis_differences = compute_axis_differences(images)

    # 打印结果
    for result in axis_differences:
        # print(f"Image: {result['image_name']}")
        # print(f"  X-axis: {result['x_axis']}")
        # print(f"  Y-axis: {result['y_axis']}")
        # print(f"  Z-axis: {result['z_axis']}")
        # print(f"  Angle with first frame Z-axis: {result['angle_with_first_z_axis']:.2f}°\n")
        if name_image1==result['image_name']:
            x1=[result['x_axis'][2],result['y_axis'][2],result['z_axis'][2]]
        if name_image2==result['image_name']:
            x2 = [-result['x_axis'][0], -result['y_axis'][0], -result['z_axis'][0]]
    return x1,x2
