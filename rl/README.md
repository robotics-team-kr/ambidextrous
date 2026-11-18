# 강화학습팀

아직 비어 있습니다. 강화학습팀 코드는 이 폴더에 둡니다.

- 로봇 모델과 Gazebo 월드는 [common/dualarm_description](../common/dualarm_description/)을 함께 씁니다.
- 비교 조건(물체, Case 1·2, 성공 판정)은 [저장소 README](../README.md#비교-조건)를 따릅니다.
- 저장소 최상위에서 `colcon build`를 하면 이 폴더의 ROS 패키지도 함께 빌드됩니다. ROS 패키지가 아닌 Python 프로젝트를 둘 때는 그 폴더에 빈 `COLCON_IGNORE` 파일을 넣어 빌드 대상에서 빼세요.
- 학습된 모델, 로그처럼 용량이 큰 파일은 git에 올리지 않습니다.
