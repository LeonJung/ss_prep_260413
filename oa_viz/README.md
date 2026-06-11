# oa_viz

**OpenArm A2 leader/follower 두 로봇의 URDF 모델을 RViz에 띄워, 실물 관절각과
모델 포즈를 비교하는 도구.** 모델(영점/축/부호)과 실물의 불일치를 눈으로 잡는다.

## 동작
- `oa_fd_cpp`의 `/oa/{leader,follower}_{left,right}/joint_state` 를
  `joint_state_bridge` 가 URDF 관절 이름(`openarmx_{left,right}_joint1..7`)으로
  합쳐 robot_state_publisher 에 공급.
- leader 로봇 x=0, follower 로봇 x=1.5 에 나란히 표시 (`world` 고정 프레임).
- URDF/메시는 패키지에 번들 (openarmx_description 설치 불필요).
  bimanual, hand 제외 (원본 xacro 의 hand 조인트 버그 회피).

## 사용 (제어 PC)
```bash
colcon build --packages-select oa_viz && source install/setup.bash

# 1) 실물 상태 퍼블리셔 — oa_fd_node 를 FREEDRIVE 로 (모터 enable + 중력보상만)
ros2 launch oa_fd_cpp oa_fd.launch.py            # 시작 모드가 FREEDRIVE
# (한 팔만 보려면 arms:=left role:=leader 등)

# 2) RViz 비교
ros2 launch oa_viz oa_viz.launch.py              # 두 로봇
ros2 launch oa_viz oa_viz.launch.py robots:=leader
```

## 비교 포인트 (불일치 잡는 법)
- 실물 팔을 천천히 움직이며 RViz 모델이 **같은 방향/같은 평면**으로 움직이는지.
  - 반대 방향 → 그 관절 부호(dir) 문제
  - 다른 평면 → 그 관절 축 정의 문제 (URDF axis)
  - 똑같이 움직이는데 자세가 통째로 다름 → 영점(zero) 오프셋
- joint_state 가 안 들어오면 모델이 아예 안 뜸 (bridge 는 실제 상태만 표시).
