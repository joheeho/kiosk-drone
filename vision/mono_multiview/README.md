# mono_multiview — 단안 다중뷰 검증 스크립트

이동 구간(3m~0.5m) 위치계산부를 카메라 두 프레임만으로 구현하는 검증 스크립트 모음. 아직 1회성 스크립트 단계이며, `kiosk_vision/`의 `aruco_pnp_node.py`처럼 계속 도는 ROS2 노드로는 이식되지 않았다.

## 파일

`mono_multiview_pose_estimate.py`가 실제로 쓰는 메인 스크립트다. 나머지 셋은 검증·튜닝용 보조 도구.

| 파일 | 역할 |
|---|---|
| **`mono_multiview_pose_estimate.py`** | **메인.** Essential Matrix/Homography로 자세 복원 → 스케일 보정 → 둘 중 자동 선택까지 전체 파이프라인 실행 |
| `mono_multiview_capture_and_match.py` | 검증용. 프레임 캡처 → ORB 특징점 → 매칭 → RANSAC만. `goto`, `wait_until_stable`, `capture_frame`, `detect_and_match` 등 위 메인 스크립트가 가져다 쓰는 공용 함수도 여기 있음 |
| `mono_multiview_position_sweep.py` | 튜닝용. 여러 거리를 훑으며 ORB 키포인트 수만 측정해서 좋은 접근 위치를 찾을 때 씀 |
| `mono_multiview_distance_sweep.py` | 검증용. 여러 거리 구간을 한 비행으로 훑으며 메인 스크립트의 파이프라인을 반복 실행 |

## 실행

repo 루트 `README.md`의 "시뮬 실행"으로 SITL을 띄운 뒤, "다른 터미널에서 비행"과 같은 방식으로 파일명만 바꿔서 실행하면 된다:

```bash
cd ~/kiosk-drone
source venv/bin/activate
python vision/mono_multiview/<파일명>.py
```

캡처 프레임·매칭 시각화는 repo 루트 `outputs/`에 저장된다.
