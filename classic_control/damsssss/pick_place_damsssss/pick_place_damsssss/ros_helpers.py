"""ROS 를 쓸 때 반복되는 작은 도우미 함수들.

이 패키지의 프로그램은 스레드 두 개로 돕니다.
  - 배경 스레드: ROS 메시지를 계속 받습니다 (app.py 에서 시작).
  - 메인 스레드: "요청을 보내고 -> 끝날 때까지 기다린다"를 위에서 아래로 순서대로 실행합니다.
아래 wait_for 가 그 "기다린다"입니다.
"""
import time

from geometry_msgs.msg import Pose as PoseMsg

from .geometry import Pose


class RosTimeout(TimeoutError):
    """ROS 쪽에서 제때 답이 오지 않았을 때."""


def wait_for(future, timeout, what):
    """요청의 답(future)이 올 때까지 기다렸다가 답을 돌려줍니다."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise RosTimeout(f'timed out after {timeout:.0f} s waiting for {what}')
        time.sleep(0.005)
    return future.result()


def call_action(client, goal, timeout, what):
    """액션(시간이 걸리는 동작)을 요청하고 끝날 때까지 기다립니다. (상태, 결과)를 돌려줍니다."""
    handle = wait_for(client.send_goal_async(goal), 10.0, f'{what} goal acceptance')
    if not handle.accepted:
        raise RuntimeError(f'{what}: goal rejected by the action server')
    answer = wait_for(handle.get_result_async(), timeout, f'{what} result')
    return answer.status, answer.result


def to_pose_msg(pose):
    """우리 Pose -> ROS 메시지."""
    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = [float(v) for v in pose.position]
    (msg.orientation.x, msg.orientation.y,
     msg.orientation.z, msg.orientation.w) = [float(v) for v in pose.orientation]
    return msg


def from_pose_msg(msg):
    """ROS 메시지 -> 우리 Pose."""
    return Pose(
        (msg.position.x, msg.position.y, msg.position.z),
        (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    )
