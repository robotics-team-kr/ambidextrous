# ambidextrous — 양팔로봇 팀

양팔 로봇으로 물체를 집어 지정 위치에 내려놓는 과제를 고전제어와 강화학습 두 방식으로 풀고, 같은 조건에서 성공률을 비교합니다.

## 폴더

| 폴더 | 담당 | 내용 |
|---|---|---|
| [common/](common/) | 공용 | 로봇 모델, Gazebo 월드, 컨트롤러 설정 ([dualarm_description](common/dualarm_description/)). 강화학습팀은 이 중 로봇 모델만 씀 |
| [classic_control/](classic_control/) | 고전제어팀 | MoveIt 2 기반 pick & place, 성공률 측정. 사람별 구현이 [damsssss/](classic_control/damsssss/), [son/](classic_control/son/)에 있음 |
| [rl/](rl/) | 강화학습팀 | Isaac Lab 사용. 아직 비어 있음 |

각 팀은 자기 폴더 안에서 자유롭게 작업합니다. `common/`과 아래 비교 조건은 두 팀의 결과에 모두 영향을 주므로, 바꿀 때는 PR로 상대 팀 확인을 받습니다.

## 비교 조건

| 항목 | 값 |
|---|---|
| 로봇 | 공개 Panda 양팔 모델 (HW팀 스펙이 나오면 교체) |
| 물체 | 5 cm 큐브 |
| 위치 정보 | 시뮬레이터가 알려줌 (카메라 없음) |
| Case 1 | 고정 위치 `(0.5, -0.3)` |
| Case 2 | 고정 위치 중심 30×30 cm 안 무작위, 목표 8/10 |
| 성공 판정 | 큐브가 목표 `(0.5, 0.0)`에서 3 cm 이내, 테이블 위에 평평하게 놓임 |

## 결과

| 방식 | Case 1 | Case 2 | 비고 |
|---|---|---|---|
| 고전제어 (damsssss) | 9/10 | 9/10 | 오른팔, GUI 없이 실행, 2026-11-19. 자세한 내용은 [classic_control/damsssss/README.md](classic_control/damsssss/README.md) |
| 고전제어 (son) | — | — | |
| 강화학습 | — | — | |

## 빌드

저장소 최상위 폴더가 colcon 워크스페이스입니다. `common/`과 `classic_control/`의 ROS 패키지를 빌드하며, 환경은 Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress입니다. 강화학습팀은 Isaac Lab을 쓰므로 이 빌드 대상이 아닙니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

설치와 실행 방법은 각 팀 폴더의 README에 있습니다.
