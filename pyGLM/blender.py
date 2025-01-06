import bpy


def create_domino(location, size=(0.1, 0.02, 0.3), name="Domino"):
    """创建一个多米诺骨牌"""
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    domino = bpy.context.object
    domino.scale = (size[0] / 2, size[1] / 2, size[2] / 2)
    domino.name = name
    # 添加刚体物理属性
    bpy.ops.rigidbody.object_add()
    domino.rigid_body.type = 'ACTIVE'
    domino.rigid_body.friction = 0.8
    domino.rigid_body.mass = 1
    return domino


def create_ground():
    """创建地面"""
    bpy.ops.mesh.primitive_plane_add(size=5, location=(0, 0, 0))
    ground = bpy.context.object
    ground.name = "Ground"
    # 添加刚体物理属性
    bpy.ops.rigidbody.object_add()
    ground.rigid_body.type = 'PASSIVE'
    ground.rigid_body.friction = 0.9
    return ground


def setup_scene():
    """设置场景，包括地面和多米诺骨牌"""
    # 删除默认对象
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    # 创建地面
    create_ground()

    # 创建多米诺骨牌
    num_dominos = 4
    spacing = 0.15  # 间隔距离
    start_x = -spacing * (num_dominos - 1) / 2
    dominoes = []
    for i in range(num_dominos):
        location = (start_x + i * spacing, 0, 0.15)
        domino = create_domino(location, name=f"Domino_{i + 1}")
        dominoes.append(domino)

    # 倾斜第一个骨牌
    dominoes[0].rotation_euler[1] = 0.2  # 绕 Y 轴倾斜

    return dominoes


def render_simulation(output_path):
    """设置渲染参数并保存动画为视频"""
    bpy.context.scene.rigidbody_world.point_cache.frame_start = 1
    bpy.context.scene.rigidbody_world.point_cache.frame_end = 250
    bpy.context.scene.frame_set(1)

    # 设置渲染引擎为 EEVEE
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'
    bpy.context.scene.render.filepath = output_path
    bpy.context.scene.render.image_settings.file_format = 'FFMPEG'
    bpy.context.scene.render.ffmpeg.format = 'MPEG4'
    bpy.context.scene.render.ffmpeg.codec = 'H264'
    bpy.context.scene.render.ffmpeg.constant_rate_factor = 'HIGH'
    bpy.context.scene.render.ffmpeg.ffmpeg_preset = 'GOOD'

    # 设置帧范围
    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = 250

    # 开始渲染动画
    bpy.ops.render.render(animation=True)


# 设置场景并渲染
setup_scene()
render_simulation(output_path="/tmp/domino_simulation.mp4")
