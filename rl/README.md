# 강화학습팀

아직 비어 있습니다. 강화학습팀 코드는 이 폴더에 둡니다.

- 시뮬레이터: Isaac Lab
- 환경: Ubuntu 22.04 + ROS 2 Humble (고전제어팀과 같게 맞춤)

고전제어팀은 Gazebo를 쓰므로 시뮬레이터가 다릅니다. 결과를 비교하려면 로봇 모델과 과제 조건을 같게 맞춰야 합니다.

## 고전제어팀과 같이 쓰는 것

| 항목 | 같이 쓰는가 | 방법 |
|---|---|---|
| 로봇 모델 (링크, 관절, 한계, 두 팔의 위치) | 예 | 아래 방법으로 URDF 파일을 뽑아 Isaac Lab에 불러옵니다 |
| 과제 조건 (테이블, 큐브, 목표, 성공 판정) | 예 | 아래 수치대로 Isaac Lab 장면을 직접 만듭니다 |
| Gazebo 월드, 컨트롤러 설정, launch 파일 | 아니요 | Gazebo 전용입니다 |
| MoveIt 설정 (`classic_control/dualarm_moveit_config`) | 아니요 | 고전제어 전용입니다 |

## 로봇 모델을 URDF로 뽑기

로봇 모델은 [common/dualarm_description/urdf](../common/dualarm_description/urdf/)에 xacro 형식으로 있습니다. ROS 2 Humble이 설치된 PC에서 저장소 최상위 폴더에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dualarm_description
source install/setup.bash
xacro common/dualarm_description/urdf/dual_panda.urdf.xacro > dual_panda.urdf
```

뽑은 파일을 Isaac Lab에 불러올 때 알아둘 점:

- **메시 파일은 이 저장소에 없습니다.** URDF 안의 메시 경로는 `package://moveit_resources_panda_description/meshes/...` 형태입니다. 메시는 apt 패키지 `ros-humble-moveit-resources-panda-description`을 설치하면 `/opt/ros/humble/share/moveit_resources_panda_description/meshes`에 생깁니다. Isaac Lab이 `package://` 경로를 찾지 못하면 URDF 안의 그 부분을 실제 폴더 경로로 바꿉니다.
- **Gazebo 전용 내용이 섞여 있습니다.** `<gazebo>`와 `<ros2_control>` 태그 6개는 Gazebo에서만 쓰는 것이라 Isaac Lab에서는 필요 없습니다.
- **두 번째 손가락은 첫 번째를 따라 움직입니다.** `<side>_panda_finger_joint2`가 `<side>_panda_finger_joint1`의 mimic 관절입니다.
- **두 팔의 위치**: 왼팔 받침은 `(0, +0.3, 0)`, 오른팔 받침은 `(0, -0.3, 0)`입니다.

`common/`의 로봇 모델이 바뀌면 URDF를 다시 뽑아야 합니다.

## 과제 조건

좌표 기준은 로봇의 `world` 링크이고 단위는 미터입니다. 테이블과 큐브의 원본은 Gazebo 월드 파일 [pick_place.sdf](../common/dualarm_description/worlds/pick_place.sdf)이고, 시작 위치와 성공 판정의 원본은 고전제어 구현의 [scene.py](../classic_control/damsssss/pick_place_damsssss/pick_place_damsssss/scene.py)와 [trials.py](../classic_control/damsssss/pick_place_damsssss/pick_place_damsssss/trials.py)입니다. 아래는 거기서 옮겨 적은 값이고, 서로 다르면 원본이 맞습니다.

| 항목 | 값 |
|---|---|
| 테이블 | 크기 `0.6 × 1.0 × 0.2`, 중심 `(0.5, 0, 0.1)`, 윗면 높이 `0.2`, 마찰 계수 1.0 |
| 큐브 | 한 변 `0.05`, 질량 0.1 kg, 마찰 계수 1.5, 테이블에 놓였을 때 중심 높이 `0.225` |
| Case 1 시작 위치 | `(0.5, -0.3)` (오른팔 기준. 왼팔은 `(0.5, +0.3)`) |
| Case 2 시작 위치 | Case 1 위치를 중심으로 한 30 × 30 cm 정사각형 안 무작위. 목표 10번 중 8번 |
| 내려놓을 목표 | `(0.5, 0.0)` |
| 성공 판정 | 큐브 중심이 목표에서 수평 3 cm 이내, 높이 오차 1 cm 이내, 기울기 5도 이내 |

## 이 폴더를 쓸 때

- 저장소 최상위는 고전제어팀의 colcon 워크스페이스입니다. 이 폴더에는 빈 `COLCON_IGNORE` 파일을 넣어 두어서 `colcon build`가 이 폴더를 건너뜁니다. 지우지 마세요.
- 학습된 모델, 로그처럼 용량이 큰 파일은 git에 올리지 않습니다.
- 결과가 나오면 [저장소 README](../README.md#결과)의 표에 적습니다.
