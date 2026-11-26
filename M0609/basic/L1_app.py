"""[L1 예제] Isaac Sim standalone 의 가장 기본 골격 (4단계)

실행: isaac_python L1_app.py   (창을 닫으면 종료)
"""

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


# Traceback (most recent call last):
#   File "/home/rokey/cobot3_ws/isaacpjt/M0609/basic/L1_app.py", line 9, in <module>
#     from isaacsim.core.api import World
# ModuleNotFoundError: No module named 'isaacsim.core'
from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0)                # 2. World

print("World 생성 성공")

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
from isaacsim.core.version import get_version

print("Isaac Sim 버전:", get_version()[0])

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0

while simulation_app.is_running():
    simulation_app.update()
    frame += 1

    if frame % 60 == 0:
        print(f"{frame} 프레임")
        frame += 1


    if frame >= 300:
        print(f"300 프레임이 지나 종료합니다")
        break
# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
