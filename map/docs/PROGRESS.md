# CN7 휠 조립 셀 — 작업 기록 (2026-11-25)

목표: 금요일(11/27)까지 **실제 PhysX 물리로 동작하는** 자동차 휠 체결 맵 완성.
참고 배치도(`/home/rokey/project/image.png`)와 같은 구성입니다. 트럭은 이번 범위에서 제외합니다.

## 1. 현재 맵 상태

| 오버뷰 (40 s) | 조립 셀 (45 s) |
|---|---|
| ![overview](images/overview_40s.png) | ![assembly](images/assembly_45s.png) |

| 탑뷰 (40 s, 조명 부족) | 휠 디테일 (60 s) | 허브 디테일 (20 s) |
|---|---|---|
| ![top](images/topview_40s.png) | ![wheel](images/wheeldetail_60s.png) | ![hub](images/hubdetail_20s.png) |

배치도와 대응하는 요소:

| 배치도 | 맵 |
|---|---|
| 타이어 공급 라인 좌/우 | `factory/conveyor` (분기 S자 컨베이어 + 축별 이송) |
| 로봇팔 (타이어 장착) | 두산 H2515 × 2 (`LeftMount`, `RightMount`) |
| 로봇팔 (스크류 체결) | 두산 M1509 × 2 (`LeftFasten`, `RightFasten`) |
| 로봇팔 레인 좌/우 | `factory/robot_rails` |
| AMR (자동 이송) | KUKA KMP1500급 AMR + 리프트 |
| 대기구역 / 출고구역 | `factory/vehicle_buffers` |

이미지 다시 찍기: `~/isaacsim/python.sh map/wheel_cell/capture.py map/scene/wheel_cell.usda map/docs/images Overview@40 ...`

## 2. 장면 분할 구조 (USD 레이어)

빌드 결과를 한 파일(18 MB)이 아닌 **루트 1개 + 영역별 서브레이어 14개**로 만듭니다.
Isaac Sim에서 `map/scene/wheel_cell.usda`를 열면 Layer 패널에서 영역별로 켜고 끄거나 편집할 수 있습니다.

```
map/scene/
├── wheel_cell.usda              # 루트: 스테이지 메타데이터 + 서브레이어 목록
├── overrides/*.usda             # 손으로 편집하는 레이어 (빌드가 덮어쓰지 않음, 가장 강함)
├── concrete.png
├── shared/asset_library.usdc    # 로봇 링크·휠·차체 공용 메시 (17 MB)
├── shared/materials.usdc        # /Looks 재질
├── environment/lighting_cameras.usdc
├── factory/
│   ├── building.usdc  floor.usdc  conveyor.usdc  robot_rails.usdc
│   ├── safety.usdc    vehicle_buffers.usdc  truck.usdc
└── orders/CN7_001.usdc … CN7_004.usdc   # 차량별 공정(AMR·로봇·휠 키프레임)
```

- 분할 코드: `map/wheel_cell/split_scene.py`. `build.py`가 빌드 마지막에 자동으로 호출합니다.
- 검증: 분할 전 단일 파일과 비교해 프림 5,100개, 속성 81,009개, 관계와 타임샘플이 모두 일치했습니다. 차이는 텍스처 경로 1개뿐인데, 복사한 같은 파일을 가리킵니다.
- 이제 `launch.py`, `check_physics.py`가 기본으로 `map/scene/wheel_cell.usda`를 엽니다.
- 나눠서 작업하는 방법은 [`map/scene/README.md`](../scene/README.md)를 보세요.
- 용어는 [`GLOSSARY.md`](GLOSSARY.md)로 통일합니다.
- 공정 시나리오와 현재 구현의 차이는 [`PROCESS_SCENARIO.md`](PROCESS_SCENARIO.md)를 보세요.

## 3. 변경 사항

| 파일 | 내용 |
|---|---|
| `visuals.py` | `isaacpjt` 원본 에셋이 없을 때 `assets/tire_example.usda`와 `assets/d455_tool.usda`를 대신 씁니다. |
| `assets/tire_example.usda`, `assets/d455_tool.usda` | 기존 빌드 결과에서 복원한 메시입니다. 다시 빌드한 결과가 원본과 일치했습니다(오차 0). |
| `physics.py` 기립 지그 | 두 턱을 **랙·피니언 조인트로 동기화**했습니다(SCHUNK PHL 방식). |
| `physics.py` 스핀들 | 인계 때 휠을 **삽입 축 중심에 맞춰** 고정합니다. |
| `physics.py` 로봇 | 타이어 장착 로봇이 시계 대신 **공정 상태에 맞춰** 움직입니다. 대기 자세에서 기다렸다가 삽입이 끝나면 안전 자세로 빠집니다. |
| `conveyor.py` 지그 대기 | 인계 후 지그를 세운 채로 축 쪽으로 물려서 낮추고, 턱을 닫습니다. |
| `split_scene.py`, `build.py` | 장면을 레이어로 분할합니다. |
| `capture.py` | 문서용 스크린샷을 렌더링합니다. |
| 디버그 | `CELL_DEBUG=1`, `CELL_DEBUG_TIP=<FL..>`, `CELL_DEBUG_JAW=<FL..>`를 켤 때만 진단 로그를 출력합니다. |

## 4. 에러 기록 (발생 순서)

모든 물리 결과는 `./map/wheel_cell/check_physics.sh --seed 42` 기준입니다.

### E1. 빌드 실패: 에셋 파일 없음 — ✅ 해결
```
FileNotFoundError: Set --asset-root to isaacpjt. Missing:
  ~/cobot3_ws/isaacpjt/M0609/robot/m0609_rg2_d455_edit.usd
  ~/cobot3_ws/isaacpjt/Factory/CN7/tire_example/tire_example.usd
```
- **원인:** 원래 빌드하던 PC의 에셋이 이 PC에는 없습니다(`Factory/` 폴더 자체가 없음).
- **조치:** 기존 `wheel_cell.usdc`에 베이크된 메시를 복원해 `assets/`에 넣고, 원본이 없으면 이 파일을 쓰게 했습니다.

### E2. 4개 스테이션 모두 `grip_slip` (기립 중 46 mm 미끄러짐) — ✅ 해결
```
FL/FR/RL/RR: result=grip_slip, grip_slip_mm≈46, tip_handoff=false
```
- **원인:** 패드 마찰 문제가 아니었습니다. 두 턱 드라이브가 650 N 한계에서 서로 밀고 있어서 **턱 쌍의 중심을 잡아주는 힘이 없었습니다.** 지그를 기울이면 휠과 턱이 한 덩어리로 턱 축을 따라 −40 mm에서 +42 mm까지 흘렀습니다(디버그 로그 `d=(0,-40.4,0)`).
- **조치:** `PhysxPhysicsRackAndPinionJoint`로 두 턱을 피니언 하나에 묶었습니다.
- **결과:** 미끄러짐이 **46 mm에서 0.5 mm**로 줄었고, 4개 스테이션 모두 기립과 인계에 성공했습니다.

### E3. 타이어 장착 로봇이 휠에 충돌 (`equipment_contact`) — ✅ 해결
```
CONTACTDBG RightMount/Link_6/GuideGripper  ↔  FR/Wheel/Web_32   (침투 1.0 mm)
```
- **원인:** 로봇 키프레임은 리플레이용이라 "로봇이 휠을 들고 허브까지 간다"는 동작이에요. 라이브에서는 셔틀 프레스가 휠을 밀어 넣습니다. 그래서 정해진 시간에 움직인 로봇 그리퍼가 공급 위치에 있던 휠에 먼저 박혔습니다.
- **조치:** 타이어 장착 로봇은 휠 바깥 대기 자세에서 기다립니다. 해당 축 삽입이 끝나면 관절 공간에서 2초 동안 안전 자세로 빠져나옵니다(`Robot.advance`).

### E4. 로봇이 대기 중인 기립 지그를 밀어 삽입 경로를 막음 — ✅ 해결
```
INSERTDBG FL ... /World/Live/FL/TipJawA/RubberPad   (삽입 내내 접촉)
JAWDBG aligned 26.75→28.75s  tool x 1.82 → 1.41
```
- **원인:** 지그가 수평 자세로 x 1.82에서 대기했습니다. AMR이 도킹하자 타이어 장착 로봇 레일 캐리지가 x 1.89로 들어오며 지그를 밀었고, 턱 빔이 허브와 휠 사이에 끼었습니다.
- **조치:** 지그를 세운 채로 축 쪽(x −0.07)으로 물리고, 0.2 m 낮춘 뒤 턱을 닫습니다.

### E5. 스터드 걸림 (`jammed`, 허브 30 mm 앞) — ✅ 대부분 해결
```
CENTERDBG FL gap 29.9 mm  center_mm 1.118  → Stud_0..4 접촉
```
- **원인:** 인계 때 스핀들이 휠의 실제 위치를 그대로 고정했습니다. 인계 기준이 6 mm 오차까지 허용하기 때문이에요. 그래서 휠이 허브 축에서 1.1 mm 어긋났고, 스터드와 구멍 사이 여유 0.7 mm를 넘어 스터드 끝에서 걸렸습니다.
- **조치:** 스핀들이 휠을 삽입 축 중심에 고정합니다(실제 스핀들의 파일럿 센터링과 같은 방식).
- **결과:** 중심 오차가 **1.1 mm에서 0.28 mm**로 줄었습니다. FL은 스터드를 통과해 **장착면 3.9 mm 앞**까지 들어갔고, RL은 11.9 mm 앞에서 멈췄습니다.

### E6. 시도했다가 되돌린 것: 지그를 더 낮게 대기 — ↩️ 되돌림
- 지그를 레일 안쪽(y 1.35)으로 넣고 0.28 m 낮추면, 셔틀 크래들(포크와 롤러)이 지그 턱 패드에 걸려 셔틀이 움직이지 못했습니다.
```
SHUTTLEDBG FL/Shuttle/Cradle/Visual/Fork ↔ FL/TipJawB/RubberPad
```
- E4의 대기 위치로 되돌렸습니다.

## 5. 현재 결과 (seed 42, 마지막 실행)

| 스테이션 | 결과 | 기립 미끄러짐 | 인계 | 남은 거리 | 원인 |
|---|---|---|---|---|---|
| FL | `jammed` | 0.58 mm | ✅ | **3.9 mm** | 8초 시간 초과. 프레스가 차체를 약 2.6 mm 밀어 허브가 함께 움직임 |
| RL | `jammed` | 0.49 mm | ✅ | 11.9 mm | 파일럿/스터드 접촉에서 정체 |
| FR | `interference` | 0.48 mm | ✅ | 411 mm | 오른쪽 타이어 장착 로봇 Link_5가 FR 지그 빔에 닿아 안전 정지 |
| RR | `interference` | 0.44 mm | ✅ | 398 mm | 위와 같은 정지로 오른쪽 전체가 멈춤 |

- AMR 도킹 정상: 32.9 s, 위치 오차 2 mm 이하, 속도 한계 안.
- 기본 검사(`check_physics.sh`)의 종료 코드는 0입니다. `--require-seated`는 아직 통과하지 못합니다.

## 6. 남은 문제 (다음 작업)

1. **오른쪽 로봇과 지그 간섭:** RightMount Link_5가 세워 둔 FR 지그 빔 윗부분(z 약 1.0)에 1~2 mm 닿습니다. 로봇이 대기 자세로 가는 경로를 조정하거나, 지그 빔 길이와 대기 위치를 다시 잡아야 합니다. 크래들 간섭(E6)도 함께 피해야 합니다.
2. **마지막 수 mm 안착:** 프레스 힘(145 N)이 차체를 밀어 허브가 같이 움직입니다. 도킹 클램프 강성을 올리거나, 삽입 시간 제한(8초)과 속도를 조정해야 합니다.
3. **볼트 체결 로봇:** 지금은 라이브에서 대기만 합니다(`sample ≤ 54 s`). 너트 체결은 키프레임 시연에만 있습니다.
4. **탑뷰와 세부 카메라 조명:** 탑뷰는 거의 검게, 세부 화면은 어둡게 렌더링됩니다.
5. 디버그 로그 코드는 환경변수로만 켜지므로 그대로 두었습니다. 정리할 때 제거하면 됩니다.

## 7. 실행 방법

```bash
cd ~/project
./map/wheel_cell/build.sh                         # 빌드 + 레이어 분할
./map/wheel_cell/run.sh --assembly --seed 42      # 라이브 물리 (창에서 Play)
./map/wheel_cell/run.sh --replay                  # 키프레임 시연
CELL_DEBUG=1 ./map/wheel_cell/check_physics.sh --seed 42   # 헤드리스 검증 + 진단 로그
```
