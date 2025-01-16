import pybullet as p
import pybullet_data
import time
import json
import numpy as np
import cv2
import os


def save_first_frame_for_views(output_folder, num_views, camera_distance, camera_target):
    """
    保存第0帧的图像信息，从多个视角进行拍摄（从上往下斜45度拍摄）。
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)


    camera_pitch = 45
    camera_up = [0, 0, 1]  # 默认上方向为 Z 轴

    # 遍历视角
    for view_idx in range(num_views):
        # 计算每个视角的偏移角度
        camera_yaw = 360 * view_idx / num_views  # 均匀分布视角

        # 计算摄像机的位置
        camera_eye_x = camera_target[0] + camera_distance * np.cos(np.radians(camera_yaw))
        camera_eye_y = camera_target[1] + camera_distance * np.sin(np.radians(camera_yaw))
        camera_eye_z = camera_target[2] + camera_distance * np.sin(np.radians(camera_pitch))
        camera_position = [camera_eye_x, camera_eye_y, camera_eye_z]

        # 生成视图矩阵和投影矩阵
        view_matrix = p.computeViewMatrix(camera_position, camera_target, camera_up)
        projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100)

        # 捕捉图像
        img = p.getCameraImage(640, 480, viewMatrix=view_matrix, projectionMatrix=projection_matrix,
                               renderer=p.ER_TINY_RENDERER)
        rgb_array = np.reshape(np.array(img[2]), (480, 640, 4))[:, :, :3]

        # 保存图像
        output_path = os.path.join(output_folder, f"view_{view_idx:02d}_frame_0.png")
        cv2.imwrite(output_path, cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR))
        print(f"Saved frame 0 for view {view_idx} at {output_path}")

def save_as_video(video_config,frames_to_simulate, p, object_ids,camera_position,camera_up,camera_target):
    video_width, video_height = video_config["width"], video_config["height"]
    fps = video_config["fps"]
    output_path = video_config["output"]

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (video_width, video_height))


    # 初始化存储物体位置信息的列表
    position_data = []

    for frame_idx in range(frames_to_simulate):
        p.stepSimulation()
        time.sleep(1 / fps)  # 控制模拟速度

        # 捕捉屏幕图像
        view_matrix = p.computeViewMatrix(camera_position, camera_target, camera_up)
        projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=video_width / video_height, nearVal=0.1,
                                                         farVal=100)
        img = p.getCameraImage(video_width, video_height, viewMatrix=view_matrix, projectionMatrix=projection_matrix,
                               renderer=p.ER_TINY_RENDERER)
        rgb_array = np.reshape(np.array(img[2]), (video_height, video_width, 4))[:, :, :3]

        # 写入视频帧
        video_writer.write(cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR))

        # 获取每个物体的位置信息和方向
        frame_positions = {"frame": frame_idx, "objects": []}
        for obj_id in object_ids:
            position, orientation = p.getBasePositionAndOrientation(obj_id)
            frame_positions["objects"].append({
                "id": obj_id,
                "position": position,
                "orientation": orientation
            })

        # 记录数据
        position_data.append(frame_positions)

    # 保存物体位置信息到 JSON 文件
    position_output_path = os.path.splitext(output_path)[0] + "_positions.json"
    with open(position_output_path, "w") as f:
        json.dump(position_data, f, indent=4)
    print(f"物体位置信息已保存为 {position_output_path}")

    # 释放资源
    video_writer.release()
    p.disconnect()

    print(f"模拟完成，视频已保存为 {output_path}")
    return position_data


def pybullet_simulate(config):
    # 初始化PyBullet环境
    p.connect(p.DIRECT)  # 使用DIRECT模式
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加载默认数据
    p.setGravity(0, 0, -9.8)

    # 添加地面
    plane_id = p.loadURDF("plane.urdf")

    # 创建物体
    object_ids = []
    for obj in config["objects"]:
        shape = obj["shape"]
        position = obj["position"]
        orientation = p.getQuaternionFromEuler(obj["orientation"])
        mass = obj.get("mass", 1)

        restitution = obj.get("restitution", 0.8)  # 默认恢复系数
        lateral_friction = obj.get("lateralFriction", 0.3)  # 默认摩擦系数

        if shape == "box":
            half_extents = obj["halfExtents"]
            collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        elif shape == "sphere":
            radius = obj["radius"]
            collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
        elif shape == "cylinder":
            radius = obj["radius"]
            height = obj["height"]
            collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
        else:
            print(f"Unsupported shape: {shape}")
            continue

        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            basePosition=position,
            baseOrientation=orientation
        )
        # 设置物体动态属性：恢复系数和摩擦系数
        p.changeDynamics(body_id, -1, restitution=restitution, lateralFriction=lateral_friction)
        object_ids.append(body_id)

    # 配置摄像机参数
    camera_config = config["camera"]
    camera_distance = camera_config["distance"]
    camera_yaw = camera_config["yaw"]
    camera_pitch = camera_config["pitch"]
    camera_target = camera_config["target"]

    # 计算摄像机位置和方向
    camera_eye_x = camera_target[0] + camera_distance * np.cos(np.radians(camera_yaw))
    camera_eye_y = camera_target[1] + camera_distance * np.sin(np.radians(camera_yaw))
    camera_eye_z = camera_target[2] + camera_distance * np.sin(np.radians(-camera_pitch))
    camera_position = [camera_eye_x, camera_eye_y, camera_eye_z]
    camera_up = [0, 0, 1]  # 默认上方向为Z轴

    # 配置视频保存
    video_config = config["video"]
    frames=config["frames"]
    # save_as_video(video_config,p)
    # save_first_frame_for_views("output", 360, camera_distance, camera_target)
    ids = [i for i in range(0,7)]
    position_data=save_as_video(video_config,frames,p,ids,camera_position,camera_up,camera_target)
    return position_data
