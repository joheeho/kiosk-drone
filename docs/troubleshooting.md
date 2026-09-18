# 트러블슈팅

## gz sim 사라짐 → Waiting for Gazebo world 무한대기

**증상**: `./build/px4_sitl_default/bin/px4` 실행 시 PX4가 `Waiting for Gazebo world`에서
멈춰서 진행되지 않는다.

**원인**: ROS2 Jazzy(`ros_gz_bridge` 설치)가 자체 vendor 경로를 `GZ_CONFIG_PATH` 맨
앞에 꽂아 넣는다. 그 결과 시스템 gz sim 설정(`/usr/share/gz/sim8.yaml`)이 가려져서
PX4가 실행하려는 gz sim 버전을 찾지 못한다. `ros_gz_bridge` 설치가 방아쇠이며,
설치 전까지는 문제없이 동작한다.

**해결**:
```bash
export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH
```
시스템 gz 경로를 맨 앞에 붙여 vendor 경로보다 먼저 찾게 한다.

매번 치기 번거로우므로 `.bashrc`의 ROS2 `source` 라인 다음에 영구적으로 추가해둘 것:
```bash
source /opt/ros/jazzy/setup.bash
export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH
```

`sim/run_kiosk.sh`도 실행 시마다 이 값을 앞에 붙이도록 되어 있다.

## ArUco 벽 안 보임 → PX4_GZ_WORLD=kiosk 지정

**증상**: SITL은 정상 기동되는데 ArUco 마커 벽(`sim/kiosk.sdf`)이 스폰되지 않는다.

**원인**: `PX4_GZ_WORLD`를 지정하지 않으면 PX4 기본 월드가 뜨기 때문에 `kiosk` 월드가
로드되지 않는다.

**해결**: `PX4_GZ_WORLD=kiosk`를 명시할 것. `make`가 아니라 빌드된 바이너리를 직접
실행할 것 (sim/README.md 참고 — ninja 컴파일 잡의 메모리 문제).
```bash
PX4_GZ_WORLD=kiosk PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4
```
