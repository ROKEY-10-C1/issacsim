# scene — 분할 작업용 장면

Isaac Sim에서 **`wheel_cell.usda`**를 여세요. 이 루트 파일이 아래 레이어를 모두 불러옵니다.
레이어 하나만 따로 열면 공용 메시(`/AssetLibrary`)와 재질 참조가 풀리지 않으니, 항상 루트로 여세요.

```
scene/
├── wheel_cell.usda            # 루트 (빌드가 다시 씀)
├── overrides/                 # ✏️ 손으로 편집하는 레이어 — 빌드가 절대 덮어쓰지 않음
│   ├── environment.usda       #   조명·카메라
│   ├── factory.usda           #   건물·바닥·타이어 공급 라인·로봇팔 레인·펜스·대기구역·출고구역
│   ├── orders.usda            #   차체·AMR·로봇·공급 휠
│   └── shared.usda            #   재질·공용 메시
├── shared/                    # 🔁 생성됨: asset_library.usdc, materials.usdc
├── environment/               # 🔁 생성됨: lighting_cameras.usdc
├── factory/                   # 🔁 생성됨: building, floor, conveyor, robot_rails, safety, vehicle_buffers, truck
├── orders/                    # 🔁 생성됨: CN7_001 … CN7_004
├── concrete.png
└── production.sqlite3         # 시연 이벤트 로그
```

## 레이어 순서 (위가 강함)

`overrides/*` → `shared/*` → `environment/*` → `factory/*` → `orders/*`

override 레이어에 쓴 값이 생성된 값을 이깁니다. 빌드를 다시 해도 override는 그대로 남아요.
(확인함: `overrides/factory.usda`에서 `TruckCab`을 숨긴 뒤 다시 빌드해도 숨김이 유지됨)

## 나눠서 작업하는 방법

1. 담당 영역을 정합니다(예: A=factory, B=orders/로봇, C=environment).
2. Isaac Sim **Layer** 패널에서 자기 `overrides/<영역>.usda`를 **Authoring Layer**(편집 대상)로 지정합니다.
3. 그 상태에서 옮기거나 숨기거나 속성을 바꾸면 그 override 파일에만 저장됩니다.
4. 각자 자기 override 파일만 커밋하면 git 충돌이 나지 않습니다.

| 하고 싶은 일 | 어디서 |
|---|---|
| 배치 조정, 숨기기, 재질·조명 바꾸기 | `overrides/<영역>.usda` |
| 형상·공정 로직·물리 값 바꾸기 | 코드 `map/wheel_cell/*.py` 수정 후 `./map/wheel_cell/build.sh` |
| 새 에셋 추가 | `overrides/<영역>.usda`에 reference로 추가 |

⚠️ `shared/`, `environment/`, `factory/`, `orders/` 아래의 `.usdc`나 `wheel_cell.usda`를 직접 고치면, 다음 빌드 때 사라집니다.

## 실행

```bash
cd ~/project
./map/wheel_cell/build.sh                      # 생성 레이어 다시 만들기 (overrides 유지)
./map/wheel_cell/run.sh --replay               # 키프레임 시연
./map/wheel_cell/run.sh --assembly --seed 42   # 라이브 물리 (Play)
```
