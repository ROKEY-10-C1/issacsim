"""[L3 예제] 장면 객체: 추가, 조회, 이동

실행: isaac_python L3_scene_objects.py
"""

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCone, VisualCone
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# 1) 떨어지는 파란 큐브 (5 cm)
# world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/Blue", name="blue", position=np.array([0.0, 0.0, 1.0]),
#         scale=np.array([0.05, 0.05, 0.05]), size=1.0, color=np.array([0, 0, 1.0]),
#     )
# )
# # 2) 움직이지 않는 회색 벽 (30 x 3 x 30 cm)
# world.scene.add(
#     FixedCuboid(
#         prim_path="/World/Wall", name="wall", position=np.array([0.5, 0.0, 0.15]),
#         scale=np.array([0.3, 0.03, 0.3]), size=1.0, color=np.array([0.5, 0.5, 0.5]),
#     )
# )
# # 3) 보이기만 하는 빨간 타겟 (파란 큐브 둘레를 돌며 벽을 그대로 통과)
# red = world.scene.add(
#     VisualCuboid(
#         prim_path="/World/Red", name="red", position=np.array([0.5, 0.0, 0.15]),
#         size=0.05, color=np.array([1.0, 0, 0]),
#     )
# )

red = world.scene.add(
    DynamicCone(
        prim_path="/World/Red", name="red", position=np.array([0.4, 0.0, 0.5]),
        radius=0.025, height=0.05, color=np.array([1.0, 0, 0]),
    )
)

yellow = world.scene.add(
    VisualCone(
        prim_path="/World/Yellow", name="yellow", position=np.array([0.4, 0.0, 0.8]),
        radius=0.025, height=0.05,  color=np.array([1.0, 1.0, 0]),
    )
)



world.reset()

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    red_pos, red_ori = red.get_world_pose()
    yellow.set_world_pose(
        position=red_pos + np.array([0, 0, 0.3]),
        orientation=red_ori,
    )


    if frame % 60 == 0:  # 처음 300 프레임 동안 60 프레임마다 출력
        red_position, red_orientation = red.get_world_pose()
        red_angles = quat_to_euler_angles(red_orientation, degrees=True)
        print(f"{frame} 프레임 | red 위치 {np.round(red_position, 3)}, 각도(deg) {np.round(red_angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()


# Dynamic
# 중력에 의해 떨어지고 바닥과 충돌

# Fixed
# 공중에 고정되어 있고 충돌 가능

# Visual
# 화면에만 존재하며 다른 물체가 통과 가능


# red_pos  # [x, y, z]
# red_ori  # [w, x, y, z]

# red.set_world_pose(
#     position=np.array([0.5, 0.0, 0.8])
# )


# 구분	Euler Angle	Quaternion
# 구성	[Roll, Pitch, Yaw]	[w, x, y, z]
# 데이터 개수	3개	4개
# 표현 방식	각 축의 회전 각도	3차원 회전을 수학적으로 표현
# 단위	Degree 또는 Radian	단위 없음
# 직관성	높음	낮음
# 짐벌락(Gimbal Lock)	발생 가능	표현 자체에서는 발생하지 않음
# 주요 용도	사용자 입력, 각도 확인	로봇 제어, 자세 연산, 회전 보간