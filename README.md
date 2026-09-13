# kiosk-drone

시각장애인을 위한 AI 드론 에이전트 기반 키오스크 접근 지원 시스템

한양대학교 ERICA 산학 캡스톤디자인 · 주식회사 닷

드론이 실내에서 키오스크를 찾아 접근하고, 화면을 인식해 시각장애인의 주문을 돕는다.

---

## 시스템 구조

```
【 위치 계산부 】  카메라로 거리·각도를 숫자로 만든다
        ↓        "거리 87cm, 각도 12도"
【   판단부   】  그래서 뭘 할지 고른다        ← AI 에이전트
        ↓        "정렬해"
【   제어부   】  고른 대로 실제로 움직인다
```

| 층 | 하는 일 | 주기 | 위치 | 디렉터리 |
|---|---|---|---|---|
| 위치 계산부 | 거리·각도 산출 | 5~10Hz | 온보드 | `vision/` |
| 판단부 | 다음 행동 결정 | 몇 초에 한 번 | 온보드/서버 | `agent/` |
| 제어부 | offboard 실행 | 30Hz | 온보드 | `control/` |
| 화면 이해부 | 버튼·글자 읽기 | 1Hz 미만 | 서버 | `screen/` |

**핵심 설계 원칙** — 세 층은 함수로 분리한다. 인식 방식(depth ↔ 단안)이나 판단 방식(규칙 ↔ LLM)을 바꿀 때 해당 함수 내부만 교체하면 되도록.

```python
def get_pose():        # 위치 계산 — 방식 교체 지점
    return distance, angle

def decide(state):     # 판단 — 규칙 ↔ LLM 교체 지점
    return action

def control_loop():    # 제어 — 안 바뀜
    d, a = get_pose()
    execute(decide(d, a))
```

---

## 일정

총 12주 (중간고사 2주, 기말고사 2주 제외)

| 마일스톤 | 마감 | 목표 |
|---|---|---|
| **M1** 시뮬 비전 이동 | 10/3 (4주) | ArUco 검출 → PnP → 규칙 기반 판단부 → 제어 루프 연결 |
| **M2** depth 전환 | 10/17 (2주) | depth 기반 접근 구간, 구간 전환 임계거리 실측 |
| **M3** LLM 자율비행 | 12/26 (6주) | LLM 패스 플래닝, 키오스크 앞 자율 정지 |

---

## 두 갈래 연구 주제

같은 코드베이스를 공유하되 주제는 나뉜다. 라벨로 구분.

| 주제 | 라벨 | 담당 |
|---|---|---|
| LLM 기반 패스 플래닝 | `topic-llm-planning` | 박준우, 조희호 , 손혜성|
| 단안 다중뷰 비전 이동 | `topic-mono-vision` | 김다인, 안정현 |



## 디렉터리

```
vision/     위치 계산부 — ArUco, PnP, depth
control/    제어부 — offboard, 비행 스크립트
agent/      판단부 — 규칙 기반 / LLM
screen/     화면 이해부 — OCR, 버튼 검출
sim/        Gazebo 월드, 모델, 마커 생성
docs/       실험 기록, 측정 데이터
tools/      유틸리티 스크립트
```
각자 작업 위치:

단안 다중뷰 → vision/, 브랜치 vision/mono-multiview
LLM 패스 플래닝 → agent/, 브랜치 agent/llm-path-planning
---

## 시작하기

### 환경

- Ubuntu 24.04 (WSL2 가능)
- ROS2 Jazzy
- PX4 SITL + Gazebo 8.14
- Python 3.12

### 설치

```bash
git clone https://github.com/<본인계정>/kiosk-drone.git
cd kiosk-drone
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 시뮬 실행

```bash
# PX4 SITL — make 쓰지 말 것 (메모리 터짐)
cd ~/kiosk_drone_ws && venv/bin/python fly_to_marker_and_capture.py

# 다른 터미널에서 비행
cd ~/kiosk-drone
venv/bin/python control/fly_to_marker_and_capture.py
```

---

## 기여하기

포크 → 브랜치 → PR. [CONTRIBUTING.md](CONTRIBUTING.md) 참조.

작게 자주 올릴 것. 완성 안 됐어도 Draft PR로 올려 방향을 확인받는 게 낫다.
