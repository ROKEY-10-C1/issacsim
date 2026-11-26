"""M0609 손목 카메라와 ROS 2 Action Graph를 사용하는 색상 분류 시뮬레이션.

PC A (Isaac Sim): source ~/isaacsim/setup_ros_env.sh
                  ROS_DOMAIN_ID=50 ~/isaacsim/python.sh pick_place_color_repectory.py
PC B (검출기): ROS_DOMAIN_ID=50, /rgb (Image) 구독, /color_id (Int32) 발행.
"""

import os
from pathlib import Path
import random

os.environ.setdefault("ROS_DOMAIN_ID", "101")

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
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

from pxr import Gf, Usd, UsdGeom

ROBOT_PRIM = "/World/m0609_rg2_d455"
CAMERA_PRIM = ROBOT_PRIM + "/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"
ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
ABOVE_DEG = [0, 12, 83, 0, 85, 0]   # 큐브 위 자세: 그리퍼가 아래를 보고, 그리퍼 끝이 바닥에서 약 0.2 m
PICK_DEG = [0, 18, 101, 0, 61, 0]   # 큐브를 집는 자세: 그리퍼 끝이 큐브 높이
CUBE_POSITION = np.array([0.45, 0.0, 0.02575])  # 로봇 정면 (m)
CUBE_COLORS = {1: (0.0, 0.0, 1.0), 2: (0.0, 1.0, 0.0)}
COLOR_NAMES = {1: "파랑", 2: "초록"}
PLACE_TARGETS = {
    1: (-45, (0.3235, -0.31883, 0.02575)),
    2: (45, (0.3235, 0.31883, 0.02575)),
}
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
CAMERA_SETTLE_FRAMES = 15
IMAGE_INTERVAL_FRAMES = 6
CUBE_SIZE = 0.0515                               # 큐브 한 변 (m)
TILE_SIZE = 0.13
TILE_THICKNESS = 0.001
RENDERING_DT = 1.0 / 60.0
ARM_SPEED_DEG_PER_SECOND = 25.0
TOLERANCE_DEG = 1.0  # 팔 관절이 모두 목표와 이만큼 안으로 들어오면 "도착"
GRIPPER_FRAMES = 60  # 그리퍼 단계는 이만큼 기다린 뒤 다음 단계로 (60 프레임 = 1 초)

# 단계 목록: (이름, 종류, 값)
#   종류 "arm"     : 값 = 팔 관절 목표 (deg). 도착하면 다음 단계
#   종류 "gripper" : 값 = "open" 또는 "close". GRIPPER_FRAMES 가 지나면 다음 단계
STEPS = [
    ("큐브 위로", "arm", ABOVE_DEG),
    ("색상 감지", "detect", None),
    ("내려가기", "arm", PICK_DEG),
    ("그리퍼 닫기", "gripper", "close"),
    ("들어 올리기", "arm", ABOVE_DEG),
    ("옆으로 돌기", "place", ABOVE_DEG),
    ("내려놓기", "place", PICK_DEG),
    ("그리퍼 열기", "gripper", "open"),
    ("올라가기", "place", ABOVE_DEG),

]


# ------------------------------------------------------------
# (가) Task: 로봇(팔 + 그리퍼)과 큐브가 놓인 환경
# ------------------------------------------------------------
class PickEnvTask(BaseTask):
    """장면(바닥 + M0609 + RG2 그리퍼 + 큐브)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
    # 할 일: 장면에 물체 추가
    def set_up_scene(self, scene):
        super().set_up_scene(scene)                   # 부모 것을 먼저 (self.scene 을 저장한다)
        scene.add_default_ground_plane()              # 바닥

        for color_id, (_, (x, y, _)) in PLACE_TARGETS.items():
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/PlaceTile{color_id}",
                    name=f"place_tile_{color_id}",
                    position=np.array([x, y, 0.0]),
                    scale=np.array([TILE_SIZE, TILE_SIZE, TILE_THICKNESS]),
                    size=1.0,
                    color=np.array(CUBE_COLORS[color_id]),
                )
            )

        # 로봇 USD 참조: 파일의 기본 prim(로봇)을 장면에 넣는다
        add_reference_to_stage(
            usd_path=str(ROBOT_USD),                  # 참조할 USD 파일 경로
            prim_path=ROBOT_PRIM,        # 장면에 만들 prim 경로 (GUI 의 Add Reference 처럼 파일 이름)
        )

        # 그리퍼 객체: finger_joint 하나로 열고 닫는다
        gripper = ParallelGripper(
            end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim (rigid body 여야 함)
            joint_prim_names=["finger_joint"],        # 구동 관절 이름 (RG2 는 하나)
            joint_opened_positions=np.array([0.0]),   # 열림 각도 (rad)
            joint_closed_positions=np.array([1.0]),   # 닫힘 각도 (rad). 큐브를 잡으면 그 전에 멈춘다
            use_mimic_joints=True,                    # 나머지 손가락은 USD 의 mimic 관절이 따라 움직임
        )

        # 로봇 객체: 팔과 그리퍼를 한 객체로 (SingleArticulation 에 gripper 가 붙은 것)
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM,    # 로봇 prim 경로 (위에서 USD 를 참조해 넣은 곳)
                name="m0609",                         # 장면에서 찾을 때 쓰는 이름 (world.scene.get_object)
                end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim (ParallelGripper 와 같게)
                gripper=gripper,                      # 위에서 만든 그리퍼 객체 -> robot.gripper 로 꺼냄
            )
        )

        # 큐브: 중력과 충돌이 있는 상자 (그리퍼로 집을 물체)
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",              # Stage 에 만들 prim 경로
                name="cube",                          # 장면에서 찾을 때 쓰는 이름
                position=CUBE_POSITION,               # 큐브 중심 위치 [x, y, z] (m)
                scale=np.array([CUBE_SIZE] * 3),      # x, y, z 방향 크기 배율 [0.0515, 0.0515, 0.0515]
                size=1.0,                             # 기본 한 변 1 m. 실제 크기 = size x scale = 0.0515 m
                color=np.array(CUBE_COLORS[1]),
            )
        )

        # 이 Task 에 속한 물체로 등록
        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._cube.name] = self._cube

    # 언제: 매 world.reset() 끝에 (Stop -> Play 때도)
    # 할 일: 시작 자세 맞추기 (팔은 홈 자세, 그리퍼는 열림으로 순간이동)
    def post_reset(self):
        color_id = random.choice(tuple(CUBE_COLORS))
        self._cube.get_applied_visual_material().set_color(np.array(CUBE_COLORS[color_id]))
        home = np.zeros(self._robot.num_dof)          # 관절 12개 값을 담을 배열 (팔 6 + 그리퍼 6)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle  # 관절 이름으로 몇 번째인지 찾아 홈 각도를 넣는다
        self._robot.set_joint_positions(home)         # 팔을 홈 자세로 순간이동

        # 그리퍼 열기: 그리퍼 객체가 finger_joint 를 열림 각도로 순간이동시킨다
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions,  # 그리퍼를 만들 때 넣은 열림 각도 [0.0]
        )
        print(f"[큐브] 랜덤 색상 {COLOR_NAMES[color_id]}({color_id}) | 시작 위치 {tuple(map(float, CUBE_POSITION))} m")

    # 언제: world.get_observations() 를 부를 때
    # 돌려줄 값: {물체 이름: {값 이름: 값}}
    def get_observations(self):
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),  # 관절 12개의 지금 각도 (rad)
            },
            self._cube.name: {
                "position": self._cube.get_world_pose()[0],           # 큐브 중심 위치 [x, y, z] (m)
            },
        }

    # 언제: 메인 루프가 물체 이름을 알아낼 때 (task.get_params())
    # 돌려줄 값: {파라미터 이름: {"value": 값}}
    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},  # "m0609"
            "cube_name": {"value": self._cube.name},    # "cube"
        }


# ------------------------------------------------------------
# (나) Controller: 단계 목록(STEPS)을 하나씩 진행한다 (상태 머신)
#      지금 단계 번호를 기억하고, 단계가 끝나면 다음 번호로 넘어간다
# ------------------------------------------------------------
class StepController(BaseController):
    """STEPS 의 단계를 차례로 진행한다. 팔 단계는 관절 목표를, 그리퍼 단계는 그리퍼 명령을 돌려준다"""

    def __init__(self, name, gripper, arm_indices):
        super().__init__(name)                        # 컨트롤러 이름
        self._gripper = gripper          # 그리퍼 객체 (forward("open" / "close") 로 명령을 만들어 준다)
        self._arm_indices = arm_indices  # 명령을 보낼 관절이 관절 배열의 몇 번째인지 (팔 6개)
        self.reset()

    # 언제: 처음 만들 때, 그리고 메인 루프가 world.reset() 할 때
    # 할 일: 기억하는 상태를 처음 값으로
    def reset(self):
        self.step = 0          # 지금 단계 번호 (STEPS 의 몇 번째)
        self._step_frames = 0  # 지금 단계에서 지난 프레임 수
        self._place_angle = None
        self._arm_start = None
        self.accepting_color = False

    def needs_color(self):
        return not self.is_done() and STEPS[self.step][1] == "detect"

    def select_color(self, color_id):
        if not self.needs_color() or not self.accepting_color or color_id not in PLACE_TARGETS:
            return False
        self._place_angle, target = PLACE_TARGETS[color_id]
        print(f"[ROS] /color_id={color_id} ({COLOR_NAMES[color_id]}) 수신 → Place {color_id}, {self._place_angle:+d}°, 목표 {target} m")
        self.accepting_color = False
        self.step += 1
        return True

    # 모든 단계를 끝냈는지
    def is_done(self):
        return self.step >= len(STEPS)

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 팔 관절 6개의 지금 각도 (rad) / 돌려줄 값: ArticulationAction (다 끝났으면 None)
    def forward(self, current_arm):
        if self.is_done() or self.needs_color():
            return None
        name, kind, value = STEPS[self.step]          # 지금 단계의 (이름, 종류, 값)
        self._step_frames += 1

        # 1) 이번 단계의 명령과 "끝났는지"
        if kind == "place":
            value = [self._place_angle, *value[1:]]
        if kind in ("arm", "place"):
            current_deg = np.degrees(current_arm)
            if self._arm_start is None:
                self._arm_start = current_deg
            travel = np.asarray(value) - self._arm_start
            max_travel = np.max(np.abs(travel))
            allowed_travel = self._step_frames * RENDERING_DT * ARM_SPEED_DEG_PER_SECOND
            fraction = allowed_travel / max_travel if max_travel > allowed_travel else 1.0
            action = ArticulationAction(
                joint_positions=np.radians(self._arm_start + fraction * travel),
                joint_indices=self._arm_indices,      # 6개 값을 관절 배열의 몇 번째에 넣을지 (그리퍼는 그대로)
            )
            finished = fraction == 1.0 and np.max(np.abs(current_deg - value)) < TOLERANCE_DEG
        else:
            # 그리퍼 객체가 finger_joint 자리에만 열림/닫힘 각도를 넣은 명령을 만들어 준다
            action = self._gripper.forward(
                action=value,                         # "open" 또는 "close"
            )
            finished = self._step_frames >= GRIPPER_FRAMES
        # 2) 끝났으면 다음 단계로
        if finished:
            self.step += 1
            self._step_frames = 0
            self._arm_start = None
        return action


def aim_camera(camera_prim, point):
    camera = UsdGeom.Xformable(camera_prim)
    parent_to_world = camera.ComputeParentToWorldTransform(Usd.TimeCode.Default())
    # D455 렌즈의 장착 위치에서 현재 큐브 중심을 바라본다.
    eye = parent_to_world.Transform(Gf.Vec3d(0, 0.0115, 0))
    world_to_camera = Gf.Matrix4d().SetLookAt(
        eye, Gf.Vec3d(*map(float, point)), Gf.Vec3d(0, 1, 0)
    )
    camera.MakeMatrixXform().Set(world_to_camera.GetInverse() * parent_to_world.GetInverse())


ROS_GRAPH_PATH = "/World/ColorSortingROS"


def create_ros_graph(render_product_path):
    import omni.graph.core as og

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": ROS_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ColorQoS", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                ("RGBPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ColorSubscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("ReceivedMessages", "omni.graph.action.Counter"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("ColorQoS.inputs:createProfile", "Custom"),
                ("ColorQoS.inputs:depth", 1),
                ("ColorQoS.inputs:reliability", "bestEffort"),
                ("RGBPublisher.inputs:topicName", "/rgb"),
                ("RGBPublisher.inputs:type", "rgb"),
                ("RGBPublisher.inputs:frameId", "d455_color_optical_frame"),
                ("RGBPublisher.inputs:renderProductPath", render_product_path),
                ("RGBPublisher.inputs:frameSkipCount", IMAGE_INTERVAL_FRAMES - 1),
                ("RGBPublisher.inputs:queueSize", 1),
                ("RGBPublisher.inputs:useSystemTime", True),
                ("RGBPublisher.inputs:enabled", False),
                ("ColorSubscriber.inputs:topicName", "/color_id"),
                ("ColorSubscriber.inputs:messagePackage", "std_msgs"),
                ("ColorSubscriber.inputs:messageSubfolder", "msg"),
                ("ColorSubscriber.inputs:messageName", "Int32"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "RGBPublisher.inputs:execIn"),
                ("Tick.outputs:tick", "ColorSubscriber.inputs:execIn"),
                ("Context.outputs:context", "RGBPublisher.inputs:context"),
                ("Context.outputs:context", "ColorSubscriber.inputs:context"),
                ("ColorQoS.outputs:qosProfile", "ColorSubscriber.inputs:qosProfile"),
                ("ColorSubscriber.outputs:execOut", "ReceivedMessages.inputs:execIn"),
            ],
        },
    )


def run():
    import omni.graph.core as og
    import omni.replicator.core as rep

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("omni.graph.action_nodes")

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDERING_DT)
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

    camera_prim = get_current_stage().GetPrimAtPath(CAMERA_PRIM)
    if not camera_prim.IsA(UsdGeom.Camera):
        raise RuntimeError(f"D455 RGB 카메라를 찾을 수 없습니다. {CAMERA_PRIM}")

    render_product = rep.create.render_product(CAMERA_PRIM, (CAMERA_WIDTH, CAMERA_HEIGHT), force_new=True)
    render_product.hydra_texture.set_updates_enabled(False)
    create_ros_graph(render_product.path)
    rgb_enabled = og.Controller.attribute(f"{ROS_GRAPH_PATH}/RGBPublisher.inputs:enabled")
    received_count = og.Controller.attribute(f"{ROS_GRAPH_PATH}/ReceivedMessages.outputs:count")
    previous_count = received_count.get()

    def set_detection_active(active):
        controller.accepting_color = active
        rgb_enabled.set(active)
        render_product.hydra_texture.set_updates_enabled(active)

    print(f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']} | /rgb → PC B | /color_id ← PC B | {ROS_GRAPH_PATH}")
    reset_needed = False
    detection_frames = 0
    first_step_logged = False

    while simulation_app.is_running():
        world.step(render=True)
        count = received_count.get()
        # Counter increments on execOut even when consecutive messages carry the same ID.
        received_color = count != previous_count
        previous_count = count

        if world.is_stopped():
            reset_needed = True
        if not world.is_playing():
            if controller.accepting_color:
                set_detection_active(False)
            continue

        if reset_needed:
            world.reset()
            controller.reset()
            previous_count = received_count.get()
            received_color = False
            detection_frames = 0
            first_step_logged = False
            reset_needed = False
            print("리셋 완료. 시작 위치에 큐브를 배치했습니다.")

        if not first_step_logged:
            print(f"{world.current_time:5.2f} s | [1/{len(STEPS)}] {STEPS[0][0]} 시작")
            first_step_logged = True
        last_step = controller.step
        if received_color and controller.accepting_color:
            color = og.Controller.attribute(f"{ROS_GRAPH_PATH}/ColorSubscriber.outputs:data").get()
            if controller.select_color(color):
                set_detection_active(False)

        observations = world.get_observations()
        cube_position = observations[cube_name]["position"]
        if controller.needs_color():
            if detection_frames == 0:
                aim_camera(camera_prim, cube_position)
            detection_frames += 1
            if detection_frames >= CAMERA_SETTLE_FRAMES and not controller.accepting_color:
                set_detection_active(True)
                print("[ROS] /rgb 발행 중. /color_id 1 또는 2 대기")
        else:
            current_arm = observations[robot_name]["joint_positions"][arm]
            action = controller.forward(current_arm)
            if action is not None:
                robot.apply_action(action)

        for completed_step in range(last_step, controller.step):
            print(f"{world.current_time:5.2f} s | [{completed_step + 1}/{len(STEPS)}] {STEPS[completed_step][0]} 완료")
            next_step = completed_step + 1
            if next_step < len(STEPS):
                print(f"{world.current_time:5.2f} s | [{next_step + 1}/{len(STEPS)}] {STEPS[next_step][0]} 시작")
        if controller.is_done() and last_step < controller.step:
            print(f"분류 완료. 큐브 중심 위치 {cube_position.round(5)} m")


try:
    run()
finally:
    simulation_app.close()
