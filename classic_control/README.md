# 고전제어팀

MoveIt 2로 경로를 계획해 큐브를 집어 옮깁니다. 같은 과제를 두 사람이 각자 구현하고, 같은 조건에서 성공률을 비교합니다.

- 로봇: 공개 Panda 양팔 모델 (HW팀 스펙이 나오면 교체)
- 환경: Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress + MoveIt 2
- 비교 조건: [저장소 README](../README.md#비교-조건)

## 폴더

| 폴더 | 내용 |
|---|---|
| [dualarm_moveit_config/](dualarm_moveit_config/) | MoveIt 설정 (SRDF, 기구학, 컨트롤러 연결). 두 사람 공용 |
| [scripts/](scripts/) | ROS 2 Humble 설치 스크립트. 두 사람 공용 |
| [damsssss/](damsssss/) | damsssss의 구현 (`pick_place_damsssss` 패키지) |
| [son/](son/) | son의 구현 (아직 비어 있음) |

## dualarm_moveit_config — MoveIt 설정

MoveIt이 이 로봇의 경로를 계획하고 실행하는 데 필요한 설정을 모은 ROS 패키지입니다. 코드는 없고 설정 파일과 launch 파일만 있습니다. 로봇 모델 자체는 [common/dualarm_description](../common/dualarm_description/)에서 가져옵니다.

| 파일 | 내용 |
|---|---|
| [config/dual_panda.srdf](dualarm_moveit_config/config/dual_panda.srdf) | 계획 그룹(`left_panda_arm`, `right_panda_arm`, `left_hand`, `right_hand`), 기본 자세(`ready`, `open`, `close`), 충돌 검사에서 뺄 링크 쌍 |
| [config/kinematics.yaml](dualarm_moveit_config/config/kinematics.yaml) | 역기구학 계산 방법 (양팔 모두 KDL) |
| [config/joint_limits.yaml](dualarm_moveit_config/config/joint_limits.yaml) | 관절별 최대 속도와 가속도, 기본 속도 비율(0.3) |
| [config/moveit_controllers.yaml](dualarm_moveit_config/config/moveit_controllers.yaml) | MoveIt이 경로를 보낼 컨트롤러 4개와 실행 시간 여유 |
| [launch/move_group.launch.py](dualarm_moveit_config/launch/move_group.launch.py) | MoveIt(`move_group`)을 켭니다. `rviz:=true`를 붙이면 RViz도 함께 켭니다 |

Gazebo가 먼저 켜져 있어야 합니다.

```bash
ros2 launch dualarm_description gazebo.launch.py          # 터미널 1
ros2 launch dualarm_moveit_config move_group.launch.py    # 터미널 2
```

자기 코드에서 MoveIt에 요청할 때 쓰는 이름은 다음과 같습니다.

- 계획 그룹: `left_panda_arm`, `right_panda_arm`
- 팔 끝 링크: `left_panda_link8`, `right_panda_link8`
- 컨트롤러: `left_arm_controller`, `right_arm_controller`, `left_hand_controller`, `right_hand_controller`

컨트롤러 이름은 [common/dualarm_description/config/ros2_controllers.yaml](../common/dualarm_description/config/ros2_controllers.yaml)과 같아야 합니다. 로봇 모델이 바뀌면 SRDF를 MoveIt Setup Assistant로 다시 만듭니다.

## scripts — 설치 스크립트

[scripts/install_ros2_humble.sh](scripts/install_ros2_humble.sh) 하나가 있습니다. Ubuntu 22.04 전용이고, 다른 버전에서는 아무것도 하지 않고 멈춥니다.

- ROS 2 apt 저장소가 없으면 추가합니다. 이미 있으면 건드리지 않습니다.
- ROS 2 Humble, MoveIt 2, Gazebo Fortress 연동(`ros_gz`, `gz_ros2_control`), `ros2_control`, Panda 모델 메시, `colcon`을 apt로 설치합니다.

자기 구현에 apt 패키지가 더 필요하면 이 스크립트의 설치 목록에 추가합니다.

## 규칙

- 각자 자기 폴더 안에서 자유롭게 작업합니다.
- ROS 패키지 이름은 사람마다 달라야 합니다. 같은 이름의 패키지가 둘 있으면 `colcon build`가 실패합니다. `pick_place_<이름>` 형식으로 짓습니다.
- `dualarm_moveit_config/`와 [common/](../common/)은 여러 사람이 함께 쓰므로, 바꿀 때는 PR로 확인을 받습니다.

## 설치와 빌드

저장소 최상위 폴더에서 실행합니다. 설치는 한 번만 하면 됩니다.

```bash
sudo bash classic_control/scripts/install_ros2_humble.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

실행 방법은 각자 폴더의 README에 있습니다.

## 결과

| 구현 | Case 1 | Case 2 | 비고 |
|---|---|---|---|
| [damsssss](damsssss/README.md) | 9/10 | 9/10 | 오른팔, GUI 없이 실행, 2026-11-19 |
| [son](son/README.md) | — | — | |
