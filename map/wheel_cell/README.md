# CN7 AMR 휠 조립 셀

HMGICS 셀 생산 방식을 참고한 Isaac Sim 5.1 물리 환경입니다. 라이브 장면의 센서와 설비 명령은 시뮬레이션 상태를 읽는 시험용 제어기에서 실행합니다. 정렬 실패, 파지 미끄러짐, 끼임은 공정 결과로 기록합니다.

## 실행

```bash
cd "$(git rev-parse --show-toplevel)"
./map/wheel_cell/run.sh --wheel-detail --seed 42
```

**CN7 | Live PhysX** 창에서 **Play**를 누릅니다. `--assembly`는 셀 전체, `--hub-detail`은 허브, `--top`은 상부 카메라를 엽니다. `--seed`를 생략하면 휠의 초기 회전각을 새로 뽑습니다. `--jam`은 왼쪽 앞 삽입 지그에 6 mm 횡오차를 줍니다. 모따기와 컴플라이언스가 흡수할 수 있는 범위를 넘는 값입니다.

- `output/wheel_cell.usdc`는 라이브 실행에서 참조하는 모델과 별도의 키프레임 시연을 담습니다. `run.sh --replay` 또는 Isaac Sim의 기본 타임라인에서 시연을 재생할 수 있습니다.
- `output/wheel_cell_physics.usdc`는 강체, 재료, 관절과 초기 자세를 포함합니다. Isaac Sim에서 직접 열면 초기 상태의 물리 실험과 자체 롤러 구동을 볼 수 있습니다. AMR 주행·파지·삽입 명령은 위 라이브 실행에서 작동합니다.
- 기본 렌더링은 1080p RTX이며 `--quality cinematic`은 1440p 패스 트레이싱입니다. 실행기의 기본 VRAM 감시 기준은 8 GiB입니다. USD를 옮길 때는 `concrete.png`를 같은 폴더에 둡니다.

## 적용한 데이터

| 대상 | 물리에 반영한 값과 근거 |
|---|---|
| 아반떼 차체 | CN7 페이스리프트 G1.6 IVT 16인치를 기준으로 전장 4,710 mm, 전폭 1,825 mm, 축거 2,720 mm, 전후 윤거 1,585 / 1,596 mm를 적용했습니다. 카탈로그 공차중량 1,265 kg에서 아래 휠 네 개를 뺀 약 1,201.5 kg을 초기 차체 질량으로 계산했습니다. [현대차 카탈로그](https://www.hyundai.com/contents/repn-car/catalog/the-new-avante-catalog.pdf) |
| 휠 | 205/55 R16, 6.5J × 16, 이론 외경 631.9 mm를 적용했습니다. Continental Elantra 적용 품번 `15570360000`의 8.0 kg 타이어와 Konig 동급 주조 16인치 림의 17.38 lb를 합쳐 약 15.9 kg으로 모델링했습니다. 림 무게는 비교품 값입니다. 관성은 타이어와 림의 환형 단면에서 추정해 USD에 명시했습니다. [현대차 매뉴얼](https://ownersmanual.hyundai.com/full_webhelp/CN7/2025/en_GN/idf379ca53e75.html) · [Continental 제품 가이드](https://www.continental-tires.com/content/dam/conti-tires-cms/continental/market-content/ca/download-pdfs/product-guide/CT2024-TireGuide_v6b.pdf.coredownload.pdf) · [Konig Lockout](https://konigwheels.com/products/konig-lockout) |
| 허브 | PCD 5×114.3 mm, 센터 보어 Ø67.10 mm, M12×1.5는 설계 기준입니다. 파일럿 Ø66.94 mm, 돌출 12 mm, 휠 오프셋 +46 mm, 스터드 돌출 30 mm, 홀 Ø13.4 mm는 실측 전 가정입니다. 이 치수의 보어–파일럿 반경 여유 0.08 mm를 충돌 형상에 그대로 적용했습니다. |
| 컨베이어 | Interroll EC5000 35 W, 49:1 형식의 Ø50 mm, 튜브 두께 1.5 mm, 정격 2.42 N·m 및 최고 반송 속도 0.37 m/s를 적용했습니다. 롤러는 튜브 질량·관성을 가진 강체로 회전합니다. 속도 제어 이득과 롤러 외피 마찰은 추정값입니다. 가이드 라이너의 동마찰은 UHMW-PE 자료 범위 안에서 0.22를 적용했습니다. [Interroll 데이터시트](https://www.interroll.com/fileadmin/user_upload/Downloads__PDF_/Drives___Controls/DC_Platform/EC5000_Catalog_PDF/EN/EC5000_50mm_IP66_EN.pdf) · [TIVAR 1000 EC 데이터시트](https://www.mcam.com/mam/38052/Nylaplas-PE-TIVAR%C2%AE%201000%20EC%20UHMW-PE_en_US.pdf) |
| 기립 지그 | SCHUNK PHL-W 40-100의 스트로크 100 mm/턱과 닫힘력 1,300 N을 참고해 양쪽 턱에 650 N 제한을 적용하고, 제품처럼 두 턱을 기계적으로 동기화했습니다(축소좌표 mimic 조인트). 패드 마찰은 HKI 자료의 0.3–0.4 범위를 참고한 정마찰 0.35와 추정 동마찰 0.28을 적용했습니다. 타이어와 패드는 계속 충돌하며, 고정 조인트로 잡아두는 동작을 제거했습니다. 지그 구조물의 질량·관성·드라이브 이득은 모델 외형을 근거로 한 추정입니다. [SCHUNK 그리퍼](https://schunk.com/us/en/gripping-systems/parallel-gripper/phl-w/phl-w-40-100/p/000000000000308172) · [SCHUNK 패드](https://schunk.com/sg/en/gripping-systems/accessories/hki/c/PGR_572) |
| AMR·승강 | KUKA KMP 1500 omniMove의 차대 711 kg, 리프트 144 kg, 정격 적재 1,500 kg, 직진 1 m/s, 횡주행 0.56 m/s, 승강 0.05 m/s를 기준으로 설정했습니다. 차체는 고무 지지 패드와 위치결정 블록 위에 놓여 하중을 전달합니다. 구동력은 네 접지점에 나누어 가하며, 각 바퀴의 측정 수직 하중 × 0.4를 넘지 못합니다. 구름 저항 0.02를 적용했습니다. 메카넘 롤러별 미끄러짐과 배터리·모터 전달계는 생략했습니다. [KUKA 사양서](https://www.kuka.com/-/media/kuka-downloads/files/87f2706ce77c4318877932fb36f6002d/kuka_kmp1500_en.pdf?hash=90820FB7DA3F2999B45EFC28EF98A626&rev=c434eb6522c4445e8b65c5e1fdde221c) |
| 두산 로봇·툴 | 장착 H2515 2대와 체결 M1509 2대에 URDF의 질량·무게중심·전체 관성 텐서·관절 한계·속도·토크 제한과 공식 충돌 메시를 적용했습니다. 복수 충돌 메시를 링크별 볼록 형상으로 조리합니다. D455의 116 g과 공칭 치수를 사용했습니다. 로봇 서보 이득과 개별 공구의 하중 데이터는 현장 보정이 필요합니다. [Doosan URDF](https://github.com/DoosanRobotics/doosan-robot2/tree/243af5d1a071f030818bbcd6e9e5ee23d9edacae/dsr_description2) · [RealSense 사양서](https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf) |

실제 패드–타이어 마찰, 차량 무게중심과 현가계는 교정이 필요합니다. 휠 너트의 현대차 공칭 체결 범위는 107–127 N·m입니다. 너트 체결·체결 토크 기록·출차·불량 배출 주문은 키프레임 시연에 포함되며, 나사산 축력이나 공구 반력은 물리 검증 범위 밖입니다.

## 물리 모델의 범위

- **스핀들 인계**: 실행 중에 조인트를 만들어 붙이지 않습니다. 5조 확장 척(`chuck.py`)이 스포크 사이로 들어가 림 안쪽을 마찰로 잡습니다. 척은 허브 면에 중심을 둔 RCC(횡 60 kN/m, 기울기 8 kN·m/rad)에 떠 있습니다. 카탈로그 무게로만 균형을 잡습니다.
- **맞춤**: 파일럿·스터드·보어 모따기를 충돌 형상에 넣었습니다. 보어와 볼트 구멍은 다각형 변의 중앙이 공칭 치수에 오도록 만들었습니다.
- **타이어·패드**: 타이어와 척 패드는 탄성 접촉입니다. 강성과 감쇠는 문헌 범위의 추정값입니다.
- **편차와 센서**: 시드마다 휠 질량 ±1.5 %, 정적 불균형, 마찰 ±10 %가 달라집니다. 제어기는 공칭값과 잡음이 있는 각도만 사용합니다.
- **인공 감쇠·슬립**: PhysX 기본 각감쇠와 절전 기능을 껐습니다. 시간 간격은 1/480 s입니다.
- **로봇**: 녹화된 팔 경로는 실제 바퀴 위치를 모르므로 구역 인터록으로 주차 자세를 유지합니다. 너트 체결은 물리 시험에 포함되지 않습니다.

한계: 480 Hz와 960 Hz는 안착 여부가 같지만, 척의 축방향 슬립은 아직 수렴하지 않았습니다(시드 42에서 0.5 mm와 약 5 mm). 슬립 값을 설계 근거로 쓰지 마십시오. 240 Hz에서는 삽입 결과 자체가 달라집니다. 척·RCC·탄성 값은 실측이 아니라 추정입니다.

## 검증 및 생성

```bash
./map/wheel_cell/build.sh
./map/wheel_cell/check_physics.sh --seed 42
./map/wheel_cell/check_physics.sh --supply-probe left
./map/wheel_cell/check_physics.sh --supply-probe right
./map/wheel_cell/check_physics.sh --tip-drive-off
./map/wheel_cell/check_physics.sh --insertion-probe nominal
./map/wheel_cell/check_physics.sh --insertion-probe skew
./map/wheel_cell/check_physics.sh --export-setup "$PWD/map/wheel_cell/output/wheel_cell_physics.usdc"
```

기본 검사는 기준 질량·관성, 중복 바디, 차체 하중 이송, USD와 PhysX 자세 일치, 휠 흘림과 안착 상태를 확인합니다. 공정 성공을 별도로 검사하려면 `--require-seated`를 사용합니다. `--supply-probe`는 트럭 하역 위치의 휠 하나가 선택한 가지로 이동하는지 검사합니다. 라이브 데모는 스테이션 앞에서 휠 네 개를 공급하는 조건입니다. 자동으로 다수 휠을 투입하고 분기를 변경하는 운영 제어는 별도 구현이 필요합니다.

`--insertion-probe`는 고정 시험대에서 휠과 허브만 평가합니다. `nominal`은 장착면 접촉을, `skew`는 6 mm 횡오차에서 스터드가 삽입을 막는지를 확인합니다. 시드 1–9와 42에서 네 바퀴가 모두 장착면에 닿는 것을 확인했습니다.

다시 빌드할 때 `ISAAC_SIM_ROOT`와 `--asset-root`, `--tool-usd`로 로컬 설치 및 자산 경로를 지정할 수 있습니다. 링크 라이브러리는 `import_doosan.py`로 공식 COLLADA 시각·충돌 메시를 변환해 다시 생성합니다. [Doosan 자산 라이선스](assets/LICENSE-Doosan.txt)

현재 실행 환경에서 검증한 물리는 CPU PhysX입니다. RTX 화면과 VRAM 사용량은 별도 장비 측정이 필요합니다.
