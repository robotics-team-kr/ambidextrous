# son의 pick & place 구현

아직 비어 있습니다. son의 구현은 이 폴더에 둡니다.

- ROS 패키지 이름은 `pick_place_son`처럼 다른 사람 것과 겹치지 않게 짓습니다. 같은 이름의 패키지가 둘 있으면 `colcon build`가 실패합니다.
- 로봇 모델과 Gazebo 월드는 [common/dualarm_description](../../common/dualarm_description/)을, MoveIt 설정은 [dualarm_moveit_config](../dualarm_moveit_config/)를 함께 씁니다.
- 비교 조건(물체, Case 1·2, 성공 판정)은 [저장소 README](../../README.md#비교-조건)를 따릅니다.
- 결과가 나오면 [고전제어팀 README](../README.md#결과)와 [저장소 README](../../README.md#결과)의 표에 적습니다.
- 참고할 구현: [damsssss/](../damsssss/)
