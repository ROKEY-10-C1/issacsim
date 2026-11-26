"""[L9 예제] 타겟 따라가기: 빨간 상자를 마우스로 옮기면 그리퍼가 따라온다

- 장면(Task): 바닥, 로봇(M0609 + RG2, 그리퍼 열림), 빨간 타겟(VisualCuboid, 한 변 4 cm)
- 타겟 시작: 홈 자세의 TCP 위치, 그리퍼가 아래를 향하는 자세
- 컨트롤러: 매 프레임 타겟 위치·자세로 IK 를 풀어 팔 관절 6개 명령을 돌려준다
    · IK 는 매 프레임 지금 자세 근처에서 푼다. 거기서 쓸 답이 없을 때만 "홈 형태를 타겟 쪽으로 돌린 자세"에서 한 번 더 푼다
    · 쓸 답이 없으면 마지막 명령을 그대로 보낸다 (그 자리에서 기다림)
    · 팔 형태가 홈 자세(joint_3, joint_5 가 +90 deg)와 다르게 뒤집히는 답은 쓰지 않는다
    · 더 잘 따라오게: 중력 보상 토크(팔 처짐 제거) + 속도 피드포워드(움직일 때 늦게 따라오는 것 줄이기)
- 메인 루프: Stop -> Play 하면 처음부터 다시 시작. 따라감 / 기다림이 바뀔 때 한 줄 출력

실행: isaac_python L9_target_follow.py
      Play 를 누른 뒤 Stage 에서 /World/Target 을 골라 이동(W) / 회전(E) 해 본다
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
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.rotations import rot_matrix_to_quat
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

# 팔 형태: 홈 자세(joint_3, joint_5 = 90 deg)와 같은 형태인지 판단할 관절과 허용 범위 (deg)
#   joint_3 (팔꿈치) 가 0 아래로 가면 팔꿈치가 반대로 꺾이고,
#   joint_5 (손목) 가 0 아래나 180 위로 가면 손목이 뒤집힌다 (182 deg 는 -178 deg 와 같은 뒤집힌 손목)
#   양 끝 2 deg 는 뒤집히기 직전이라 뺀다
FORM_RANGES_DEG = {"joint_3": (2.0, 178.0), "joint_5": (2.0, 178.0)}

TARGET_SIZE = 0.04  # 타겟 한 변 (m)

# 더 잘 따라오게 하는 값
RENDER_DT = 1.0 / 60.0                # 한 프레임 시간 (s). 컨트롤러는 프레임마다 한 번 불린다
MAX_TRACK_STEP_DEG = 3.0              # 한 프레임에 이보다 크게 바뀌면 "따라가기" 가 아니라 "자세 전환" -> 속도 피드포워드를 끈다

# IK 계산기: 장면의 로봇과 상관없이 계산만 한다 (L8 과 같다)
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,
    urdf_path=URDF_PATH,
)

# 타겟 시작 위치·자세 = 홈 자세의 TCP (FK 로 계산). 홈 자세에서는 그리퍼가 아래를 향한다
HOME_TCP_POS, home_tcp_rot = ik_solver.compute_forward_kinematics(TCP_FRAME, np.radians(HOME_DEG))
HOME_TCP_ORIENT = rot_matrix_to_quat(home_tcp_rot)  # 회전 행렬 -> 쿼터니언 [w, x, y, z]


# ------------------------------------------------------------
# (가) Task: 로봇(팔 + 그리퍼)과 빨간 타겟이 놓인 환경
# ------------------------------------------------------------
class TargetFollowTask(BaseTask):
    """장면(바닥 + M0609 + RG2 그리퍼 + 빨간 타겟)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
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

        # 타겟: 물리 없이 보이기만 하는 상자. 마우스로 옮기거나 돌릴 수 있다
        #   처음 넣은 위치·자세가 기본 상태가 되어, world.reset() 때 여기로 돌아온다
        self._target = scene.add(
            VisualCuboid(
                prim_path="/World/Target",
                name="target",
                position=HOME_TCP_POS,                # 홈 자세의 TCP 위치
                orientation=HOME_TCP_ORIENT,          # 홈 자세의 TCP 자세 (그리퍼 아래 방향)
                scale=np.array([TARGET_SIZE] * 3),
                size=1.0,
                color=np.array([1.0, 0, 0]),          # 빨간색
            )
        )
        # 같은 로봇을 가리키는 Articulation(여러 로봇용 view): 중력 보상 토크를 읽는 데만 쓴다
        self._robot_view = scene.add(Articulation(prim_paths_expr="/World/m0609_rg2_d455", name="m0609_view"))

        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._target.name] = self._target

    # 언제: 매 world.reset() 끝에 (처음 시작할 때, 그리고 Stop -> Play 때)
    # 할 일: 초기 자세(팔 홈 + 그리퍼 열림)로 되돌린다
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        home[self._robot.get_dof_index("finger_joint")] = self._robot.gripper.joint_opened_positions[0]
        zero = np.zeros(self._robot.num_dof)
        self._robot.set_joints_default_state(positions=home, velocities=zero)  # 다음 reset 이 되돌릴 자세
        self._robot.set_joint_positions(home)                                   # 지금 자세를 홈으로
        self._robot.set_joint_velocities(zero)                                  # 남은 속도 없애기
        self._robot.apply_action(ArticulationAction(joint_positions=home))      # 구동 목표도 홈으로

    # 돌려줄 값: 로봇 관절 각도와 중력 보상 토크, 타겟 위치·자세
    def get_observations(self):
        position, orientation = self._target.get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
                # 지금 자세에서 중력을 버티는 데 필요한 관절 토크 (N·m). 관절 12개, 로봇 1대라 [0]
                "gravity_efforts": self._robot_view.get_generalized_gravity_forces()[0],
            },
            self._target.name: {"position": position, "orientation": orientation},
        }

    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},
            "target_name": {"value": self._target.name},
        }


# ------------------------------------------------------------
# (나) Controller: 매 프레임 타겟으로 IK 를 풀어 팔 명령을 만든다
# ------------------------------------------------------------
class TargetFollowController(BaseController):
    """타겟 위치·자세 -> IK -> 팔 관절 6개 명령. 쓸 답이 없으면 마지막 명령을 그대로 보낸다"""

    def __init__(self, name, ik_solver, arm_indices, dt):
        super().__init__(name)
        self._ik = ik_solver
        self._arm_indices = arm_indices
        self._dt = dt  # 프레임 시간 (s): 속도 피드포워드 = 이번 명령 변화 / dt
        # 팔 형태 판단에 쓸 관절이 팔 6개 중 몇 번째인지와 허용 범위 (rad) -> {2: (lo, hi), 4: (lo, hi)}
        self._form = {ARM_JOINTS.index(name): np.radians(limits) for name, limits in FORM_RANGES_DEG.items()}
        self.reset()

    # 언제: 처음 만들 때, 그리고 메인 루프가 world.reset() 할 때
    # 할 일: 기억하는 상태를 처음 값으로
    def reset(self):
        self._last_command = np.radians(HOME_DEG)  # 마지막으로 보낸 팔 명령 (rad). 처음은 홈 자세
        self.following = True                      # 따라가는 중인지 (쓸 답이 있었는지). 메인 루프가 읽는다
        self.reason = ""                           # 기다릴 때 그 이유

    def _keeps_form(self, joints):
        """팔 형태가 홈 자세와 같은지: joint_3, joint_5 가 모두 홈과 같은 범위 (0 ~ 180 deg) 안에 있는지"""
        return all(lo < joints[i] < hi for i, (lo, hi) in self._form.items())

    def _solve(self, target_position, target_orientation, warm_start):
        """IK 한 번. 쓸 답이면 (관절값, ""), 아니면 (None, 이유)"""
        joints, success = self._ik.compute_inverse_kinematics(
            TCP_FRAME, target_position, target_orientation, warm_start=warm_start)
        if not success:
            return None, "IK 해 없음 (팔이 닿지 않는 위치나 자세)"
        if not self._keeps_form(joints):
            return None, "팔 형태가 뒤집히는 답 (joint_3 / joint_5 가 0 ~ 180 deg 를 벗어남)"
        return joints, ""

    def _home_form_seed(self, target_position, current_arm):
        """홈 형태(joint_3, joint_5 = 90 deg)를 joint_1 만 타겟 쪽으로 돌린 자세 (rad)

        joint_1 은 같은 방향이 360 deg 마다 반복되므로, 지금 joint_1 에 가장 가까운 값을 고른다.
        """
        heading = np.arctan2(target_position[1], target_position[0])
        candidates = heading + 2 * np.pi * np.array([-1, 0, 1])
        joint_1 = candidates[np.argmin(np.abs(candidates - current_arm[0]))]
        seed = np.radians(HOME_DEG)
        seed[0] = seed[5] = joint_1
        return seed

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 타겟 위치 [x, y, z], 타겟 자세 [w, x, y, z], 팔 관절 6개의 지금 각도 (rad),
    #          팔 관절 6개의 중력 보상 토크 (N·m)
    def forward(self, target_position, target_orientation, current_arm, gravity_efforts):
        # 1) 지금 자세 근처에서 푼다 -> 팔이 크게 바뀌지 않는다
        joints, reason = self._solve(target_position, target_orientation, current_arm)
        # 2) 쓸 답이 없으면: 홈 형태를 타겟 쪽으로 돌린 자세에서 한 번 더 (예: 타겟을 로봇 뒤로 한 번에 옮겼을 때)
        if joints is None:
            joints, _ = self._solve(target_position, target_orientation,
                                    self._home_form_seed(target_position, current_arm))

        velocity = np.zeros(len(self._arm_indices))
        if joints is None:
            self.following, self.reason = False, reason   # 기다림: 마지막 명령 그대로, 속도 0
        else:
            self.following, self.reason = True, ""
            step = joints - self._last_command
            # 속도 피드포워드: 타겟이 움직이는 만큼 관절도 이 속도로 움직이라고 알려 준다
            #   (위치 목표만 주면 관절 댐핑이 움직임을 막아 한 박자 늦게 따라온다)
            #   한 프레임에 크게 바뀐 경우(자세 전환)에는 속도를 주지 않는다
            if np.max(np.abs(step)) < np.radians(MAX_TRACK_STEP_DEG):
                velocity = step / self._dt
            self._last_command = joints            # 쓸 답일 때만 마지막 명령을 바꾼다

        return ArticulationAction(
            joint_positions=self._last_command,    # 쓸 답이 없으면 마지막 명령 그대로 (그 자리에서 기다림)
            joint_velocities=velocity,             # 속도 피드포워드 (rad/s)
            joint_efforts=gravity_efforts,         # 중력 보상: 팔 무게만큼 관절 토크를 더해 처짐을 없앤다
            joint_indices=self._arm_indices,
        )


# ------------------------------------------------------------
# (다) 등록과 준비
# ------------------------------------------------------------
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDER_DT)
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
task = TargetFollowTask(name="target_follow")
world.add_task(task)
world.reset()

params = task.get_params()
robot_name = params["robot_name"]["value"]
target_name = params["target_name"]["value"]
robot = world.scene.get_object(robot_name)
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]
controller = TargetFollowController("target_follow_controller", ik_solver=ik_solver, arm_indices=arm, dt=RENDER_DT)

x, y, z = HOME_TCP_POS
print(f"타겟 시작 | 홈 자세 TCP x {x:.3f} y {y:.3f} z {z:.3f} m | Play 후 /World/Target 을 옮겨 보세요")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
was_following = None  # 바로 전 프레임의 따라감 / 기다림 (바뀌면 출력)
while simulation_app.is_running():
    world.step(render=True)

    # 1) Stop 을 감지해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        # 2) Stop 다음 첫 Play 프레임: 장면(로봇·타겟)과 컨트롤러를 함께 처음으로
        if reset_needed:
            world.reset()
            controller.reset()
            was_following = None
            reset_needed = False
            print("reset 완료: 처음부터 다시 시작")
        # 3) 관측 (Task)
        observations = world.get_observations()
        current_arm = observations[robot_name]["joint_positions"][arm]
        target_position = observations[target_name]["position"]
        target_orientation = observations[target_name]["orientation"]
        # 4) 계산 (Controller)
        gravity_efforts = observations[robot_name]["gravity_efforts"][arm]
        action = controller.forward(target_position, target_orientation, current_arm, gravity_efforts)
        # 5) 명령
        robot.apply_action(action)

        # 따라감 / 기다림이 바뀔 때 한 줄 출력
        if controller.following != was_following:
            x, y, z = target_position
            state = "따라감" if controller.following else f"기다림 | {controller.reason}"
            print(f"시각 {world.current_time:6.2f} s | {state} | 타겟 x {x:.3f} y {y:6.3f} z {z:.3f} m"
                  f" | joint_1 ~ 6 {np.round(np.degrees(current_arm)).astype(int)} deg")
            was_following = controller.following

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
