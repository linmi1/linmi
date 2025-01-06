import glfw
from OpenGL.GL import *
from OpenGL.GL.shaders import compileShader, compileProgram
import glm
import numpy as np
import time
import cv2

# 顶点数据 (矩形多米诺骨牌)
vertices = [
    -0.1, 0.0, -0.5,  1.0, 0.0, 0.0,
     0.1, 0.0, -0.5,  0.0, 1.0, 0.0,
     0.1, 1.0, -0.5,  0.0, 0.0, 1.0,
    -0.1, 1.0, -0.5,  1.0, 1.0, 0.0,

    -0.1, 0.0,  0.5,  1.0, 0.0, 0.0,
     0.1, 0.0,  0.5,  0.0, 1.0, 0.0,
     0.1, 1.0,  0.5,  0.0, 0.0, 1.0,
    -0.1, 1.0,  0.5,  1.0, 1.0, 0.0,
]

indices = [
    0, 1, 2, 2, 3, 0,  # Back face
    4, 5, 6, 6, 7, 4,  # Front face
    0, 1, 5, 5, 4, 0,  # Bottom face
    2, 3, 7, 7, 6, 2,  # Top face
    1, 2, 6, 6, 5, 1,  # Right face
    0, 3, 7, 7, 4, 0   # Left face
]

vertices = np.array(vertices, dtype=np.float32)
indices = np.array(indices, dtype=np.uint32)

# 初始化 GLFW
if not glfw.init():
    raise Exception("GLFW can't be initialized")

glfw.window_hint(glfw.VISIBLE, glfw.FALSE)  # 隐藏窗口
window = glfw.create_window(800, 600, "Domino Gravity Simulation", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window can't be created")

glfw.make_context_current(window)
glEnable(GL_DEPTH_TEST)

# 着色器代码
vertex_shader = """
#version 130
in vec3 aPos;
in vec3 aColor;

out vec3 ourColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = aColor;
}
"""

fragment_shader = """
#version 130
in vec3 ourColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(ourColor, 1.0);
}
"""

# 编译和链接着色器
vertex = compileShader(vertex_shader, GL_VERTEX_SHADER)
fragment = compileShader(fragment_shader, GL_FRAGMENT_SHADER)
shader = compileProgram(vertex, fragment)

# VAO、VBO、EBO 设置
VAO = glGenVertexArrays(1)
VBO = glGenBuffers(1)
EBO = glGenBuffers(1)

glBindVertexArray(VAO)

glBindBuffer(GL_ARRAY_BUFFER, VBO)
glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)

glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO)
glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)

glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * vertices.itemsize, ctypes.c_void_p(0))
glEnableVertexAttribArray(0)

glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * vertices.itemsize, ctypes.c_void_p(12))
glEnableVertexAttribArray(1)

glBindBuffer(GL_ARRAY_BUFFER, 0)
glBindVertexArray(0)

# 相机设置
view = glm.lookAt(glm.vec3(0, 2, 6), glm.vec3(0, 0.5, 0), glm.vec3(0, 1, 0))
projection = glm.perspective(glm.radians(45.0), 800 / 600, 0.1, 100.0)

# 多米诺骨牌的初始状态
dominos = [
    {"position": glm.vec3(-1 + 0.3 * i, 0.0, 0.0), "rotation": 0.0, "velocity": 0.0, "angular_velocity": 0.0}
    for i in range(10)
]

def aabb_collision(a, b):
    """简单的 AABB 碰撞检测"""
    return abs(a[0] - b[0]) < 0.2 and abs(a[1] - b[1]) < 1.0

# 帧时间和力模拟
last_time = glfw.get_time()
gravity = 9.8
fall_force = 10.0

t = 0
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (800, 600))
while t < 1000:
    t += 1
    glfw.poll_events()

    # 计算帧时间
    current_time = glfw.get_time()
    delta_time = current_time - last_time
    last_time = current_time

    # 更新物理状态
    for i, domino in enumerate(dominos):
        if domino["rotation"] < 90.0:
            domino["angular_velocity"] += gravity * delta_time  # 模拟重力
            domino["rotation"] += domino["angular_velocity"] * delta_time

            if domino["rotation"] >= 90.0:
                domino["rotation"] = 90.0

            # 检查碰撞
            if i + 1 < len(dominos):
                next_domino = dominos[i + 1]
                if aabb_collision(domino["position"], next_domino["position"]):
                    next_domino["angular_velocity"] += fall_force

    # 清屏
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # 渲染每个多米诺骨牌
    glUseProgram(shader)
    glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm.value_ptr(view))
    glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm.value_ptr(projection))

    for domino in dominos:
        model = glm.mat4(1.0)
        model = glm.translate(model, domino["position"])
        model = glm.rotate(model, glm.radians(domino["rotation"]), glm.vec3(0, 0, 1))
        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm.value_ptr(model))

        glBindVertexArray(VAO)
        glDrawElements(GL_TRIANGLES, len(indices), GL_UNSIGNED_INT, None)

    # 保存帧到视频
    width, height = glfw.get_framebuffer_size(window)
    data = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
    frame = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)[::-1]
    out.write(frame)

    glfw.swap_buffers(window)

out.release()
glfw.terminate()