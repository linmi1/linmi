import pybullet as p
import pybullet_data
import time
import json
import numpy as np
import cv2

# 从配置文件加载场景配置
config_path = "output_config.json"  # 配置文件路径
with open(config_path, "r") as f:
    config = json.load(f)

# 初始化PyBullet环境
p.connect(p.DIRECT)  # 使用DIRECT模式
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加载默认数据
p.setGravity(0, 0, -9.8)

# 添加地面
plane_id = p.loadURDF("plane.urdf")

# 添加柔性物体
soft_body_config = config.get("soft_body", None)
if soft_body_config:
    soft_body_path = soft_body_config["file_path"]
    soft_body_scale = soft_body_config.get("scale", 1.0)
    soft_body_position = soft_body_config.get("position", [0, 0, 1])
    soft_body_orientation = p.getQuaternionFromEuler(soft_body_config.get("orientation", [0, 0, 0]))

    # 加载柔性物体
    soft_body_id = p.loadSoftBody(
        fileName=soft_body_path,
        basePosition=soft_body_position,
        baseOrientation=soft_body_orientation,
        scale=soft_body_scale,
        mass=soft_body_config.get("mass", 1),
        useNeoHookean=soft_body_config.get("useNeoHookean", 0),
        useBendingSprings=soft_body_config.get("useBendingSprings", 1),
        springElasticStiffness=soft_body_config.get("springElasticStiffness", 40),
        springDampingStiffness=soft_body_config.get("springDampingStiffness", 0.1),
        springBendingStiffness=soft_body_config.get("springBendingStiffness", 1),
        useSelfCollision=soft_body_config.get("useSelfCollision", 1),
        frictionCoeff=soft_body_config.get("frictionCoeff", 0.5),
        useFaceContact=soft_body_config.get("useFaceContact", 1)
    )
    print(f"Loaded soft body with ID: {soft_body_id}")

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
video_width, video_height = video_config["width"], video_config["height"]
fps = video_config["fps"]
output_path = video_config["output"]

fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writer = cv2.VideoWriter(output_path, fourcc, fps, (video_width, video_height))

# 模拟并保存帧
frames_to_simulate = config["frames"]
for i in range(frames_to_simulate):
    p.stepSimulation()
    time.sleep(1 / fps)  # 控制模拟速度

    # 捕捉屏幕图像
    view_matrix = p.computeViewMatrix(camera_position, camera_target, camera_up)
    projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=video_width / video_height, nearVal=0.1, farVal=100)
    img = p.getCameraImage(video_width, video_height, viewMatrix=view_matrix, projectionMatrix=projection_matrix,
                           renderer=p.ER_TINY_RENDERER)
    rgb_array = np.reshape(np.array(img[2]), (video_height, video_width, 4))[:, :, :3]

    # 写入视频帧
    video_writer.write(cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR))

# 释放资源
video_writer.release()
p.disconnect()

print(f"模拟完成，视频已保存为 {output_path}")
