# study_for_v1 — v1.0을 향한 공부·계획 노트

> **모드:** 이 문서가 살아있는 동안은 **공부·계획만** 한다. 운영자가 "개발하라"고 명시하기 전까진
> 코드 변경 없음. 공부한 핵심은 여기에 빠짐없이 기록.
> 현재 코드 = v0.3 (백업 `~/backup_ws/openarmx_bilateral_v0.3_69368a7`).

---

## §1. 제어 방식 비교 — openarmx_bilateral(우리) vs enactic openarm_teleop

출처(enactic): `~/ext_ref/openarm_teleop/` (Enactic, Inc. 2025, Apache-2.0) —
`control/openarm_bilateral_control.cpp`(main), `src/controller/control.cpp`(bilateral_step),
`config/{leader,follower}.yaml`, `src/periodic_timer_thread.hpp`, `src/robot_state.hpp`.

### 핵심 한 줄
**제어 법칙은 사실상 동일(대칭 P-P bilateral = SPBT). 근본 차이는 "아키텍처".**
enactic = 단일 프로세스 + 공유메모리 커플링 + 500Hz tight loop / 우리 = ros2_control 분산 노드 +
DDS relay 커플링 + 150Hz.

### 비교표
| 항목 | **openarmx_bilateral (우리, v0.3)** | **enactic openarm_teleop** |
|---|---|---|
| **제어 법칙** | 대칭 P-P: τ=Kp(q_peer−q)+Kd(q̇_peer−q̇)+τ_ff | **동일** (대칭 P-P / SPBT) |
| **MIT 명령 생성** | HW 드라이버가 pos=peer, kp/kd, effort=FF를 패킹 | control이 직접 `MITParam{Kp,Kd, pos=peer, vel=peer, eff=FF}` |
| **크로스커플링 경로** | **DDS** (relay_node: leader `/joint_states`→follower cmd 토픽, +CM) | **공유메모리** (AdminThread가 `robot_state` references를 peer 응답으로 set) — **in-process, ~0 지연** |
| **프레임워크** | ros2_control (분산 노드 + DDS 토픽) | 독립 C++ 단일 프로세스 (raw CAN, rclcpp 제어루프 밖) |
| **프로세스/스레드** | 노드 다수(relay·grav·fric·2×CM·HW) + 브링업 2개 | 단일 프로세스, 3스레드: LeaderArmThread / FollowerArmThread / AdminThread(커플링) |
| **제어 rate** | **150Hz 균일** (파이프라인 패치 후; 원래 75/86, USB2CAN 상한) | **500Hz/arm** (PeriodicTimerThread, Ts=1/500) |
| **피드포워드(FF)** | 중력 + 마찰(Coulomb) + 속도FF + posture(J3) | 중력 + 마찰(풀모델). coriolis는 계산하나 **bilateral엔 미적용**(unilateral만 ×0.1) |
| **마찰 모델** | `Fc·tanh(0.1·k·ω)` (Coulomb 위주, Fv≈0, 대충값) | `Fo + Fv·ω + Fc·tanh(k·ω)` (오프셋+점성+Coulomb, 식별값) |
| **중력보상** | 별도 노드 `gravity_comp_node`(KDL) → effort 토픽 | control 내부 `Dynamics::GetGravity` (leader/follower 각각 dynamics 객체) |
| **모터 / CAN 라이브러리** | **Robstride** / openarmx-can (namespace openarmx) | **DM(Damiao)** / openarm_can (namespace openarm) |
| **MIT KP 범위** | RS04/RS03(J1-4) [0,5000], RS00(J5-7,그리퍼) [0,500] | DM 전부 [0,500] (KD [0,5]) |
| **CAN I/O 방식** | ros2_control read/write (파이프라인 패치: read=recv만, write=send만) | `bilateral_step` 내 `mit_control_all` + `sleep 200us` + `recv_all(220us)` |
| **그리퍼** | 별도 처리(위치홀드 + 우리 마찰 패치 2줄) | **동일 MIT 루프의 한 관절**로 취급(Kp/Kd/eff 동일 파이프) |
| **채택 게인(현재)** | kp=35,20,15,8,8,3,1,1 / kd=3.5,5,2.5,0.8,0.8,0.3,0.1,0.1 | kp=240,240,240,240,24,31,25,16 / kd=3×4,0.2×4 (leader=follower) |
| **구성 범위** | bimanual (arm:=left/right/both) | 단일 arm 쌍(leader↔follower), 한 PC가 양쪽 CAN 소유 |
| **BANNED(둘 다 안 씀)** | DOB/RTOB/energy tank/4ch/wave 없음 | 없음 (순수 SPBT + FF) |

### §1.1 관찰·시사점 (v1.0 관점)
1. **법칙은 베낄 게 없다 — 이미 같다.** 둘 다 "MIT kp로 peer 위치를 당기고, 중력+마찰을 FF로 상쇄"하는
   대칭 P-P. 우리가 못 따라가는 건 알고리즘이 아니라 **구현 아키텍처**.
2. **가장 큰 구조 차이 = 커플링 경로.** enactic은 peer 위치가 **공유메모리로 ~0지연** 도달. 우리는
   **DDS relay 홉**(leader js→relay→follower cmd)을 거침. 우리 relay는 200Hz/지터 0.05ms로 깨끗하지만
   여전히 직렬화+토픽+CM 지연이 있는 층. → **in-process 단일-CM bilateral 컨트롤러**(update()에서 커플링
   직접 계산)가 enactic 구조에 대응. transparency의 "지연" 축 후보.
3. **rate: enactic 500Hz vs 우리 150Hz.** 단 이건 알고리즘이 아니라 **USB2CAN full-speed 상한** 탓
   (§LATENCY_INVESTIGATION). enactic의 DM+CAN이 우리 full-speed 동글과 다르면 500이 가능. 우리 150은
   소프트웨어 상한까지 짜낸 값. rate 더는 HS-USB/PCIe(HW) 필요.
4. **마찰 모델이 우리보다 정교(Fv 점성 + Fo 오프셋 + 식별값).** 우리는 Coulomb 대충값. **자유공간
   가벼움(transparency)** 개선 여지 = 여기. friction 재식별 + Fv/Fo 도입이 저비용 카드.
5. **coriolis**: enactic도 bilateral엔 안 씀(계산만). → 우리도 굳이 필요 없음(unilateral ×0.1 참고).
6. **gripper를 1급 관절로 통합** = enactic이 더 깔끔. 우리는 드라이버 하드코딩 우회 중.

### §1.2 열린 질문 (다음 공부 주제)
- Q1. enactic AdminThread(커플링)의 실제 rate와 leader/follower 스레드와의 동기화 방식? (지연·지터 영향)
- Q2. enactic이 실제로 500Hz를 CAN에서 달성하나? 그들의 USB/CAN 하드웨어는? (우리 150 상한과 비교)
- Q3. enactic 마찰 식별 절차(Fc/k/Fv/Fo)는 어떻게? 우리 friction_id에 Fv/Fo 확장 가능?
- Q4. "in-process 단일-CM bilateral 컨트롤러"로 갈 때 ros2_control 안에서 공유메모리 커플링을 어떻게
      구현? (양팔 단일 CM + 커스텀 controller, DDS relay 제거)
- Q5. v1.0의 정의/목표는? (rate? transparency 지표? 아키텍처 전환? 마찰 정교화?) — 계획 수립 필요.

---

## §2. (예정) …
> 다음 공부 주제가 생기면 여기에 계속 추가.
