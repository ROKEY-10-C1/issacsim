"""[L8 연습 문제] L7 의 pick and place 를 IK 로 만든다

- 큐브 pick 위치 (0.45, 0.20), place 위치 (0.45, -0.25)
- 자세 4개(큐브 위 / 집기 / 놓을 위치 위 / 내려놓기)를 관절 각도로 직접 적지 않고 IK 로 계산한다
- Task, StepController, STEPS, 메인 루프는 L7 과 같다

실행: isaac_python L8_ik_quiz.py
"""

from pathlib import Path

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
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# 파일 경로 (L8 과 같다)
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"                        # 장면에 넣을 로봇 (M0609 + RG2 + D455)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # IK 용: 링크 길이, 관절 축, 관절 한계
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # IK 용: 움직일 관절 (팔 6개)

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
TCP_FRAME = "gripper_center"  # IK 로 위치를 맞출 기준점: 그리퍼 손가락 사이

# 큐브
CUBE_SIZE = 0.0515                                         # 큐브 한 변 (m)
PICK_XY = (0.45, 0.20)                                     # 큐브를 집을 위치 (m)
PLACE_XY = (0.45, -0.25)                                   # 큐브를 놓을 위치 (m)
CUBE_POSITION = np.array([*PICK_XY, CUBE_SIZE / 2])        # 큐브 중심: 바닥 위에 놓인 높이

# TCP 높이 (m)
ABOVE_Z = 0.20                    # 큐브 위: 그리퍼 끝이 바닥에서 약 0.2 m
GRASP_Z = CUBE_SIZE / 2 + 0.015   # 집기·내려놓기: 손가락 사이가 큐브 중심보다 1.5 cm 위 (≈ 0.041 m)

# TCP 자세: 그리퍼가 바닥을 본다 (L8 의 roll 180, pitch 0)
#   yaw 180: 홈 자세(joint_6 = 0)와 같은 방향으로 그리퍼를 둔다 -> joint_6 이 joint_1 과 같은 각도로 나온다
#   yaw 0 이면 그리퍼가 반 바퀴 돌아 joint_6 이 -157 / -210 deg 가 된다 (손목이 불필요하게 크게 돈다)
DOWN = euler_angles_to_quat(np.radians([180, 0, 180]))     # 쿼터니언 [w, x, y, z]

TOLERANCE_DEG = 1.0  # 팔 관절이 모두 목표와 이만큼 안으로 들어오면 "도착"
GRIPPER_FRAMES = 60  # 그리퍼 단계는 이만큼 기다린 뒤 다음 단계로 (60 프레임 = 1 초)


# ------------------------------------------------------------
# ★ IK: 목표 TCP 위치 4개 -> 팔 관절 자세 4개 (deg)
# ------------------------------------------------------------
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)


def solve_ik(label, position, warm_start_deg):
    """목표 TCP 위치 [x, y, z] (그리퍼는 아래 방향) -> 팔 관절 6개 (deg)

    warm_start_deg 에서 가까운 답을 찾는다. 바로 앞 자세를 넣으면 팔이 크게 꺾이지 않는다.
    """
    joints, success = ik_solver.compute_inverse_kinematics(
        TCP_FRAME,                                    # 목표에 맞출 프레임
        np.array(position),                           # 목표 위치 [x, y, z] (m)
        DOWN,                                         # 목표 자세 (그리퍼 아래 방향)
        warm_start=np.radians(warm_start_deg),        # 계산을 시작할 관절값 (rad)
    )
    x, y, z = position
    if not success:
        raise RuntimeError(f"IK 해 없음 | {label} | 목표 x {x:.2f} y {y:5.2f} z {z:.3f} m: 팔이 닿지 않는 위치나 자세다")
    deg = np.degrees(joints)
    print(f"IK | {label} | 목표 x {x:.2f} y {y:5.2f} z {z:.3f} m -> joint_1 ~ 6 {np.round(deg).astype(int)} deg")
    return deg


# 자세 4개: 앞 자세를 warm start 로 넘겨 이어지는 답을 고른다
ABOVE_PICK_DEG = solve_ik("큐브 위", [*PICK_XY, ABOVE_Z], HOME_DEG)
PICK_DEG = solve_ik("집기", [*PICK_XY, GRASP_Z], ABOVE_PICK_DEG)
ABOVE_PLACE_DEG = solve_ik("놓을 위치 위", [*PLACE_XY, ABOVE_Z], ABOVE_PICK_DEG)
PLACE_DEG = solve_ik("내려놓기", [*PLACE_XY, GRASP_Z], ABOVE_PLACE_DEG)

# 단계 목록: (이름, 종류, 값)  -- L7 과 같은 형식. 팔 자세만 IK 결과로 바뀌었다
#   종류 "arm"     : 값 = 팔 관절 목표 (deg). 도착하면 다음 단계
#   종류 "gripper" : 값 = "open" 또는 "close". GRIPPER_FRAMES 가 지나면 다음 단계
STEPS = [
    ("큐브 위로", "arm", ABOVE_PICK_DEG),
    ("내려가기", "arm", PICK_DEG),
    ("그리퍼 닫기", "gripper", "close"),
    ("들어 올리기", "arm", ABOVE_PICK_DEG),
    ("옆으로 옮기기", "arm", ABOVE_PLACE_DEG),
    ("내려놓기", "arm", PLACE_DEG),
    ("그리퍼 열기", "gripper", "open"),
    ("올라가기", "arm", ABOVE_PLACE_DEG),
]


# ------------------------------------------------------------
# (가) Task: 로봇(팔 + 그리퍼)과 큐브가 놓인 환경 (L7 과 같다)
# ------------------------------------------------------------
class PickEnvTask(BaseTask):
    """장면(바닥 + M0609 + RG2 그리퍼 + 큐브)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/m0609_rg2_d455")

        gripper = ParallelGripper(
            end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0.0]),
            joint_closed_positions=np.array([1.0]),
            use_mimic_joints=True,
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path="/World/m0609_rg2_d455",
                name="m0609",
                end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",
                gripper=gripper,
            )
        )
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=CUBE_POSITION,               # pick 위치 (0.45, 0.20)
                scale=np.array([CUBE_SIZE] * 3),
                size=1.0,
                color=np.array([0, 0, 1.0]),
            )
        )
        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._cube.name] = self._cube

    # 언제: 매 world.reset() 끝에 (처음 시작할 때, 그리고 Stop -> Play 때)
    # 할 일: 초기 자세(팔 홈 + 그리퍼 열림)로 되돌린다. 아래 3 가지를 모두 맞춰야 매번 같은 상태에서 시작한다
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)          # 관절 12개 (팔 6 + 그리퍼 6)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        # 그리퍼 열림 각도 (finger_joint, 나머지 mimic 관절은 0 = 열림)
        home[self._robot.get_dof_index("finger_joint")] = self._robot.gripper.joint_opened_positions[0]
        zero = np.zeros(self._robot.num_dof)

        # 1) 기본 상태: 다음 world.reset() 이 로봇을 되돌릴 자세 (등록 안 하면 USD 에 저장된 자세로 간다)
        self._robot.set_joints_default_state(positions=home, velocities=zero)
        # 2) 지금 상태: 관절 위치를 홈으로 순간이동하고, 남아 있던 속도를 없앤다
        self._robot.set_joint_positions(home)
        self._robot.set_joint_velocities(zero)
        # 3) 관절 구동 목표: Stop 전 마지막 명령(올라가기 자세)이 남아 있으면 팔이 그쪽으로 끌려가므로 홈으로 바꾼다
        self._robot.apply_action(ArticulationAction(joint_positions=home))

    def get_observations(self):
        return {
            self._robot.name: {"joint_positions": self._robot.get_joint_positions()},
            self._cube.name: {"position": self._cube.get_world_pose()[0]},
        }

    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},
            "cube_name": {"value": self._cube.name},
        }


# ------------------------------------------------------------
# (나) Controller: 단계 목록(STEPS)을 하나씩 진행한다 (L7 과 같다)
# ------------------------------------------------------------
class StepController(BaseController):
    """STEPS 의 단계를 차례로 진행한다. 팔 단계는 관절 목표를, 그리퍼 단계는 그리퍼 명령을 돌려준다"""

    def __init__(self, name, gripper, arm_indices):
        super().__init__(name)
        self._gripper = gripper
        self._arm_indices = arm_indices
        self.reset()

    def reset(self):
        self.step = 0
        self._step_frames = 0

    def is_done(self):
        return self.step >= len(STEPS)

    def forward(self, current_arm):
        if self.is_done():
            return None
        name, kind, value = STEPS[self.step]
        self._step_frames += 1

        if kind == "arm":
            action = ArticulationAction(
                joint_positions=np.radians(value),
                joint_indices=self._arm_indices,
            )
            finished = np.max(np.abs(np.degrees(current_arm) - value)) < TOLERANCE_DEG
        else:
            action = self._gripper.forward(action=value)
            finished = self._step_frames >= GRIPPER_FRAMES
        if finished:
            self.step += 1
            self._step_frames = 0
        return action


# ------------------------------------------------------------
# (다) 등록과 준비 (L7 과 같다)
# ------------------------------------------------------------
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
task = PickEnvTask(name="pick_env")
world.add_task(task)
world.reset()

params = task.get_params()
robot_name = params["robot_name"]["value"]
cube_name = params["cube_name"]["value"]
robot = world.scene.get_object(robot_name)
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]
controller = StepController("step_controller", gripper=robot.gripper, arm_indices=arm)

# ============================================================
# 3. 메인 루프 (L7 과 같다. 출력에 TCP 위치와 관절 각도를 더했다)
# ============================================================
reset_needed = False
last_step = 0
while simulation_app.is_running():
    world.step(render=True)

    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        if reset_needed:
            world.reset()
            controller.reset()
            last_step = 0
            reset_needed = False
            print("reset 완료: 처음부터 다시 시작")
        observations = world.get_observations()
        current_arm = observations[robot_name]["joint_positions"][arm]
        action = controller.forward(current_arm)
        if action is not None:
            robot.apply_action(action)

        if controller.step != last_step:  # 단계가 끝났을 때 한 번 출력
            finger = observations[robot_name]["joint_positions"][robot.get_dof_index("finger_joint")]
            x, y, z = observations[cube_name]["position"]
            tcp, _ = ik_solver.compute_forward_kinematics(TCP_FRAME, current_arm)  # 지금 관절값 -> TCP 위치 (FK)
            print(f"시각 {world.current_time:5.2f} s | {STEPS[last_step][0]} 끝"
                  f" | TCP x {tcp[0]:.2f} y {tcp[1]:5.2f} z {tcp[2]:.3f} m"
                  f" -> joint_1 ~ 6 {np.round(np.degrees(current_arm)).astype(int)} deg"
                  f" | 그리퍼 {finger:.2f} rad | 큐브 x {x:5.2f} y {y:5.2f} z {z:.3f} m")
            last_step = controller.step
            if controller.is_done():
                print("모든 단계 완료")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
