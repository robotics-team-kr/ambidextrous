# damsssss의 pick & place 구현

물체 위치를 받아서 → 한 팔로 집어서 → 지정 위치에 내려놓고, 성공률을 측정합니다.

- 로봇: 공개 Panda 양팔 모델 (HW팀 스펙이 나오면 교체)
- 환경: Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress + MoveIt 2

## 비교 조건

물체, Case 1·2, 성공 판정은 다른 구현(son, 강화학습팀)과 공통이며 [저장소 README](../../README.md#비교-조건)에 있습니다.

코드에서 쓰는 수치는 [scene.py](pick_place_damsssss/pick_place_damsssss/scene.py) 한 곳에 있습니다.

## 현재 상태

Ubuntu 22.04 + ROS 2 Humble에서 GUI 없이(`headless:=true`) 끝까지 실행해 확인했습니다 (2026-11-19).

| 항목 | 결과 |
|---|---|
| 빌드, 단위 테스트 | 패키지 3개 빌드, 테스트 56개 통과 |
| Gazebo 스폰, 컨트롤러 | 5개 모두 `active` |
| Case 1 (고정 위치, 오른팔) | 9/10 성공 → PASS |
| Case 2 (무작위 위치, 오른팔, seed 0) | 9/10 성공 → PASS |

알려진 점:
- 실패 2건은 모두 `grasp` 단계에서 큐브를 놓친 경우입니다 (손가락이 끝까지 닫힘).
- 놓인 위치가 목표에서 y 방향으로 약 1.5 cm 일정하게 벗어납니다. 허용 오차(3 cm) 안이지만 원인은 아직 확인하지 않았습니다.
- Gazebo GUI, RViz, 왼팔(`arm:=left`)은 아직 실행해 보지 않았습니다.

## 설치 (한 번만)

저장소 최상위 폴더에서 실행합니다.

```bash
sudo bash classic_control/scripts/install_ros2_humble.sh
```

Ubuntu 22.04(jammy) 전용입니다. ROS 2 apt 저장소가 없으면 추가하고, 이 워크스페이스가 쓰는 패키지(Humble, MoveIt 2, Gazebo Fortress 연동, ros2_control)를 설치합니다.

## 빌드

저장소 최상위 폴더가 colcon 워크스페이스입니다. `common/`과 `classic_control/`의 패키지를 함께 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ambidextrous        # 저장소를 받은 위치
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select pick_place_damsssss && colcon test-result --verbose
```

## 실행 — 단계별로 확인

| 단계 | 명령 | 확인할 것 |
|---|---|---|
| 1. Gazebo에 로봇 띄우기 | `ros2 launch dualarm_description gazebo.launch.py` | 양팔이 ready 자세로 서 있고, `ros2 control list_controllers`에 5개가 `active` |
| 2. MoveIt | `ros2 launch pick_place_damsssss sim.launch.py rviz:=true` | RViz에 MotionPlanning 디스플레이 추가 후 `right_panda_arm` 계획·실행 |
| 3–5. 한 번 집어 옮기기 | `ros2 run pick_place_damsssss pick_place` | 로그에 `SUCCESS` |
| 6. Case 1 | `ros2 run pick_place_damsssss evaluate --ros-args -p case:=1 -p trials:=10` | 요약 줄과 `results/case1_*.csv` |
| 6. Case 2 | `ros2 run pick_place_damsssss evaluate --ros-args -p case:=2 -p trials:=10` | `8/10` 이상이면 `PASS` |

3–6단계는 `sim.launch.py`가 켜진 상태에서 다른 터미널로 실행합니다. GUI 없이 돌리려면 `headless:=true`.

`evaluate` 옵션: `arm`(right/left), `seed`(무작위 위치 재현용, 기본 0), `random_yaw`(큐브 방향도 무작위, 기본 false), `output_dir`.

큐브 위치 토픽 확인: `ros2 topic echo /model/cube/pose --once`

`/joint_states`에서 두 번째 손가락은 `<side>_panda_finger_joint2_mimic`이라는 이름으로 나옵니다 (Humble의 gz_ros2_control 동작).

## 구조

```
common/
  dualarm_description/     로봇 모델(xacro), Gazebo 월드, ros2_control 컨트롤러   ← 1단계
classic_control/
  dualarm_moveit_config/   SRDF, 기구학, 컨트롤러 연결, move_group launch          ← 2단계
  damsssss/pick_place_damsssss/
    object_pose.py         물체 위치 받기 (카메라로 바꿀 때 여기만 교체)
    grasp.py               잡는 자세 계산 (위에서 아래로 집기)                     ← 4단계
    gripper.py             그리퍼 열기/닫기                                        ← 3단계
    moveit_client.py       MoveIt 호출 (자유 이동 / 직선 이동)
    pick_place.py          접근 → 하강 → 집기 → 들기 → 옮기기 → 놓기              ← 5단계
    trials.py, evaluate.py 시행 위치, 성공 판정, 성공률 측정                       ← 6단계
    scene.py               테이블·큐브·목표 위치 등 과제 수치
    pick_place_node.py     한 번 집어 옮기는 실행 프로그램
    app.py                 두 실행 프로그램이 함께 쓰는 준비 작업 (노드, 부품 조립)
    sim.py                 시뮬레이터에서 큐브를 시작 위치로 옮기기
    geometry.py, ros_helpers.py  자세 계산, ROS 요청을 보내고 기다리는 도우미
```

## 안 될 때 만질 곳

| 증상 | 조절할 것 |
|---|---|
| 큐브가 미끄러지거나 튕김 | [panda_arm.xacro](../../common/dualarm_description/urdf/panda_arm.xacro) 맨 위 `finger_effort`(쥐는 힘), `finger_mu`(마찰) / 월드 파일의 큐브 `mu` |
| 팔이 명령을 늦게 따라감 | launch 인자 `position_gain:=0.3` (기본 0.1) |
| 실행 중 시간 초과로 중단 | [moveit_controllers.yaml](../dualarm_moveit_config/config/moveit_controllers.yaml)의 `allowed_execution_duration_scaling` |
| 계획 실패가 잦음 | [moveit_client.py](pick_place_damsssss/pick_place_damsssss/moveit_client.py)의 `allowed_planning_time`, `num_planning_attempts` |
| 움직임이 너무 빠름/느림 | [moveit_client.py](pick_place_damsssss/pick_place_damsssss/moveit_client.py) 맨 위 `FREE_SPEED`, `LINEAR_SPEED` |

실패한 시행은 CSV의 `failed_stage` 열에 어느 단계에서 멈췄는지 남습니다.

## 설계 메모

- **위치 받기 분리**: 큐브 위치는 [object_pose.py](pick_place_damsssss/pick_place_damsssss/object_pose.py)의 `SimObjectPoseSource`에서만 받습니다. 지금은 Gazebo의 실제 큐브 자세이고, 카메라를 쓸 때는 같은 메서드 두 개(`latest`, `wait_until_still`)를 가진 클래스를 만들어 [app.py](pick_place_damsssss/pick_place_damsssss/app.py)에서 바꿔 끼우면 됩니다.
- **큐브는 MoveIt 충돌 장면에 넣지 않았습니다.** 테이블만 넣었습니다. 접근·들기·옮기기를 큐브 위쪽에서 직선으로 하기 때문에 지금은 필요 없지만, 장애물이 늘면 추가해야 합니다.
- **쓰지 않는 팔**은 ready 자세로 가만히 있고, MoveIt이 팔끼리의 충돌을 검사합니다.
- **성공 판정**은 시퀀스 완료 여부와 별개로 시뮬레이터의 최종 큐브 위치로 합니다.

## HW팀 모델로 교체할 때

1. [dual_panda.urdf.xacro](../../common/dualarm_description/urdf/dual_panda.urdf.xacro)와 컨트롤러 YAML을 새 로봇으로 교체
2. [dual_panda.srdf](../dualarm_moveit_config/config/dual_panda.srdf)를 MoveIt Setup Assistant로 다시 생성
3. [scene.py](pick_place_damsssss/pick_place_damsssss/scene.py)의 그룹·링크·관절 이름과 [grasp.py](pick_place_damsssss/pick_place_damsssss/grasp.py)의 그리퍼 치수(`TCP_OFFSET` 등) 수정
4. `colcon test`로 이름·수치 불일치 확인
