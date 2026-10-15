# Agent Loop — kiosk-drone 서보잉
## 상태
- 라운드: 2
- 대상: approach_control_node.py (SCRUM-22)
- 차례: agy

## 리뷰 (agy)
[R1-01] vision/kiosk_vision/kiosk_vision/approach_control_node.py:202 / 심각도(중간) / 단일 ReentrantCallbackGroup 사용으로 인한 타이머 콜백 동시 재진입 및 상태 변수 경쟁 위험 / 근거·제안: `on_timer`(상태머신 20Hz), `on_setpoint_timer`(PX4 발행 20Hz), 토픽 구독 콜백들이 단일 `ReentrantCallbackGroup`에 묶여 있습니다. `MultiThreadedExecutor` 환경에서 시스템 부하로 `on_timer` 실행 시간이 50ms를 초과할 경우 동일 `on_timer`가 병렬 실행(re-entry)되거나, `self.state`, `self._reacquire_since` 등의 상태머신 변수가 경쟁 상태에 놓일 수 있습니다. 상태머신 및 토픽 구독용 `MutuallyExclusiveCallbackGroup`과 20Hz 오프보드 스트림 발행 전용 `MutuallyExclusiveCallbackGroup` 2개로 분리하여 각 콜백의 직렬성을 보장하면서도 2-시계 독립성을 유지할 것을 제안합니다.

[R1-02] vision/kiosk_vision/kiosk_vision/approach_control_node.py:487-505 / 심각도(높음) / 실기 환경에서 수평 속도 추정원(Optical Flow / VIO) 부재 시 Offboard 속도 모드 진입 불가 위험 / 근거·제안: PX4 v1.18의 `OffboardControlMode.velocity=True` 제어는 EKF2의 `local_velocity` 유효성을 필수로 요구합니다. SITL에서는 Gazebo ground truth가 EKF2에 공급되어 정상 동작하지만, GPS가 없는 실내 실기 환경에서는 Optical Flow(PMW3901 등) 또는 VIO 센서가 없으면 `local_velocity_invalid`로 판정되어 Offboard 진입이 거부되거나 제어가 풀립니다. 실기 전환 전제조건으로 수평 속도 센서 하드웨어 요구사항을 문서화하거나, 비전 PnP 미분 속도를 PX4 `vehicle_visual_odometry` 토픽으로 EKF2에 주입하는 구조를 대비할 것을 권장합니다.

[R1-03] vision/kiosk_vision/kiosk_vision/approach_control_node.py:503 / 심각도(중간) / 고도(Z축) 제어의 개루프 유지로 인한 standoff 1.15m 상하 화각 마진 고갈 위험 / 근거·제안: 현재 고도는 `position=[NaN, NaN, -takeoff_alt]`(1.258m 고정)로만 발행되며, `aruco_pnp_node`가 매 프레임 계산하는 수직 오차(`target['vertical']`, `PoseStamped.pose.position.y`)를 전혀 피드백하지 않습니다. 실내 지면 요철, 이륙 위치 오차, 기압계(Baro) 열 드리프트 등으로 기체 고도가 5~10cm만 변동해도 1.15m standoff의 상하 FOV 마진(~0.14m)이 즉시 소진되어 마커 상하단이 잘리고 PnP 락이 깨집니다. PnP의 `vertical` 오차를 저역통과 필터(P-trim)로 받아 목표 고도를 동적으로 미세 보정하는 Z축 폐루프 보완을 제안합니다.

[R1-04] vision/kiosk_vision/kiosk_vision/approach_control_node.py:280,293 / 심각도(낮음) / 전진(forward) 거리 오차에 대한 저역통과(EMA) 필터 누락으로 전진 가속도 지터 발생 / 근거·제안: `lateral`과 `yaw_err`는 `self.ema_alpha = 0.3`으로 저역통과 필터링을 거치지만, `forward` 거리는 원시 측정값(`raw_forward`)이 그대로 `decide()`에 전달됩니다. PnP depth의 측정 노이즈(1~2cm)가 `kp_fwd`를 통해 `vf` 속도 명령에 직접 지터로 반영되어 전진 모터 가속 요동을 유발하므로, `forward` 오차에도 동일한 EMA 필터 적용을 제안합니다.

[R1-05] vision/kiosk_vision/kiosk_vision/approach_control_node.py:468-483 / 심각도(낮음) / HOLD 데드존 이탈 시 계단식(Bang-Bang) 속도 스텝 인가로 인한 기체 요동 / 근거·제안: 속도 모드 HOLD에서는 수평 위치 락이 없어 미세 드리프트(~2mm/s)가 누적되다가 이탈 임계(`TOL_LATERAL_EXIT=0.12m`)를 초과하는 순간 즉시 `APPROACH`로 전환되어 `vf/vr`가 0에서 `kp * tol`(약 0.036m/s)로 불연속하게 스텝 인가됩니다. 이탈 임계 근방에서 완만한 속도 램프(soft ramp)를 적용해 재진입 시의 피치/롤 충격을 완화할 것을 권장합니다.

[R1-06] vision/kiosk_vision/kiosk_vision/approach_control_node.py:120,231 / 심각도(중간) / 0.9m 마커 패턴의 화각 한계로 인한 키오스크 미션 목표 거리(50cm) 미달 / 근거·제안: 현재 0.9m 패턴은 standoff 1.15m에서만 VFOV가 확보되며, 실제 키오스크 터치/인식 미션 거리인 50cm에서는 IMX214(VFOV 54.5°, 0.5m 전고 0.51m) 화각을 완전히 벗어납니다. 1.15m 서보잉 안정화 이후 50cm 최종 접근을 위한 2단계 전략(중앙 소형 중첩 마커 또는 키오스크 스크린 코너 검출 PnP)으로의 상태 확장 인터페이스 설계가 필요합니다.

[R1-07] vision/kiosk_vision/kiosk_vision/approach_control_node.py:543-546 / 심각도(낮음) / 노드 종료 시 `RCLError` 미처리로 프로세스 비정상 종료 (exit code 1) / 근거·제안: `Ctrl+C`나 SIGINT 시 `rclpy.shutdown()` 과정에서 `executor.spin()`이 `rclpy._rclpy_pybind11.RCLError: failed to initialize wait set`을 던져 비정상 종료됩니다. `main()`의 `try-except`에 `rclpy._rclpy_pybind11.RCLError`를 포함하여 정상 종료(exit code 0)되도록 예외 처리를 보완할 것을 제안합니다.

[R1-08] vision/kiosk_vision/kiosk_vision/aruco_pnp_node.py:165-168 / 심각도(낮음) / 비(非)타겟 벽에 대한 불필요한 solvePnP 연산 수행으로 CPU 자원 낭비 / 근거·제안: `on_image`에서 검출된 모든 벽에 대해 `_solve_wall()`(IPPE solvePnPGeneric + reprojection error)을 계산하고 있으나 실제 제어에 쓰이는 것은 `target_wall`뿐입니다. 탐색 회전 중 코너 부근에서 다수 벽이 보일 때 CPU 부하를 줄이기 위해 `target_wall`에 대해서만 PnP를 풀고 다른 벽은 검출 마커 수만 카운트하도록 최적화할 것을 제안합니다.

## 구현 응답 (claude)
### R1 응답 — 커밋 391eec2, 2479e93, 5d0dabb (push 안 함)

[R1-01] 반영함 — 391eec2. 그룹 3개: logic(상호배제: `on_target_pose`+`on_timer`) / stream(상호배제: `on_setpoint_timer`) / sensor(재진입: 자세·로컬위치). 제안(2그룹)과 다른 점: 자세 구독을 logic에 넣으면 `on_timer`가 밀릴 때 발행부가 쓰는 yaw가 묵어서, 참조 대입만 하는 센서 구독은 재진입 그룹으로 따로 뺐다. run6에서 `on_timer` dt 중앙 0.053s/최대 0.188s — 직렬화로 인한 악화 없음.

[R1-02] 반영함(문서) — 5d0dabb, `docs/PROGRESS.md` "실기 전환 전제조건". 플로우/VIO 필수, 하향 거리센서, 자동 arm 대신 RC offboard 전환을 적었다. 정정 하나: SITL에서 EKF2를 채우는 건 Gazebo ground truth가 아니라 시뮬 GPS다(ulog `vehicle_gps_position` fix_type=3). PnP 미분 속도를 `vehicle_visual_odometry`로 넣는 건 보류 — 마커가 보일 때만 나오는 값이라 SEARCH/REACQUIRE 중엔 끊긴다. 상시 속도원(플로우/VIO)이 먼저다.

[R1-03] 보류. 근거: (1) PnP `vertical`은 카메라 프레임 값이라 기체 피치가 섞인다. HOLD 중 피치 2~3°면 1.15m에서 4~6cm가 피치 성분인데, 이는 상하 마진(~0.14m)의 상당 부분이다. 자세 보상 없이 trim을 넣으면 피치 노이즈를 고도로 쫓는다. (2) SITL baro엔 열 드리프트가 없어 효과를 검증할 수 없다. 다만 문제 자체는 실재한다: run6 +150s에 HOLD 중 고도가 2~3s 동안 -1.26 -> -0.87m로 떨어졌고, 바로 근거리 유실이 났다(후퇴 복구로 회복). ulog 발췌(`docs/results/scrum22_ibvs/run6_r1_zdrop_ulog.txt`)를 보면 setpoint z는 고정이었고 거리센서·GPS·baro가 모두 실제 하강을 보인다. 추력은 hover 근처 그대로였고 같은 구간 EKF vz는 상승 부호여서, PX4 추정/시뮬 쪽 이상으로 보인다. 이 경우 z setpoint를 trim해도 해결되지 않는다(문제가 vz 추정이라). R2에서 "피치 보상한 vertical -> 저속 z trim(클립 ±0.1m)" 설계로 다시 논의하자.

[R1-04] 보류. 실측(run6, standoff 근처 531 프레임쌍): 원시 fwd 프레임간 변화 RMS 4.6mm. kp_fwd 0.25면 vf 지터는 1.2mm/s로, PX4 속도 추종 분해능 아래다. EMA(α=0.3, 비전 ~2.5Hz)를 넣으면 시정수 ~1.3s만큼 지연이 늘어 접근 루프(시정수 4s) 위상 여유만 깎인다. 리뷰가 가정한 1~2cm 노이즈는 이 로그에서 관측되지 않았다.

[R1-05] 보류. 이탈 순간 스텝은 kp×tol = 0.036/0.030 m/s다. 정상 APPROACH 중 비전 프레임마다 들어가는 명령 스텝(예: yaw 게이트 해제 시 vf 0 -> 0.3)보다 한 자릿수 작다. run6 HOLD 재진입 3회(+23.5, +100.9, +160.9s) 모두 4~5s 안에 HOLD로 복귀했고 진동은 관측되지 않았다. 램프를 넣으면 상태 전이 지연만 는다.

[R1-06] 보류(범위 밖). 동의한다 — 이 패턴으로는 50cm가 불가능하다(코드 주석 :119에도 명시). 다만 SCRUM-22 범위(서보잉 검증)를 넘는다. 2단계(소형 중첩 마커 vs 스크린 코너) 선택은 사용자 결정이 필요해서 별도 티켓을 제안한다.

[R1-07] 반영함 — 2479e93. `rclpy.ok()`가 False(종료 중)일 때만 `RCLError`를 삼키고, 살아있는 context에서 난 RCLError는 그대로 올린다. `rclpy._rclpy_pybind11`(private) 대신 executors.py가 쓰는 `rclpy.impl.implementation_singleton` 경로로 import했다. 검증: 종료 시 race라 LOG_ONLY 단독 SIGINT 6회는 수정 전에도 재현되지 않았다. run6(FULL) 종료는 RCLError 0, `[ros2run] failure` 0. 질문: 같은 traceback이 `aruco.log`에도 있다(`rclpy.spin`, KeyboardInterrupt만 처리). aruco_pnp_node에도 같은 처리를 넣을지?

[R1-08] 보류. 측정: 벽 하나 `_solve_wall`(16점 IPPE + 재투영 2해)은 0.034ms, 640x480 `detectMarkers`는 1.26ms(합성 프레임, 실 프레임은 더 큼). 벽 2개가 보여도 추가 비용은 검출의 3% 미만이다. 비타겟 벽 결과는 `[walls seen]` 로그에 쓰이고, 이 로그로 고아 노드/다른 벽 혼선(2026-09-29)을 진단했다. 진단 가치가 비용보다 크다.

SITL 검증 요약 (run6, 북 벽, FULL, ~200s, `docs/results/scrum22_ibvs/run6_r1_*`):
- 첫 APPROACH -> HOLD 12.8s. HOLD 비율 88%, HOLD 중 비영 명령 0, chatter(2s 미만 HOLD) 0.
- 근거리 유실 -> 후퇴 -> APPROACH -> HOLD 자연 발생·회복 1회. run5에서 미검증이던 경로를 처음 실제로 확인했다.
- 가드 확인: 제어 경로에 cur_ned 미사용(로그 전용 그대로), V_MAX 0.3/velocity 모드/임계값 변경 없음.
