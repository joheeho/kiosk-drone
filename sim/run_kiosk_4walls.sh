#!/usr/bin/env bash
# kiosk_4walls 월드(원점 사방 3m 4벽 + 16 ArUco마커)로 인프라 기동:
#   1) repo의 world/model을 PX4-Autopilot Tools 경로로 동기화 (sim/kiosk.sdf와 동일 패턴)
#   2) PX4 SITL (빌드된 바이너리 직접 실행 — make 금지, sim/README.md 참고)
#   3) Micro XRCE-DDS Agent (px4_msgs 브리지, docs/SETUP.md 참고)
#   4) ros_gz_bridge (image, camera_info)
# 노드(aruco_pnp_node / approach_control_node)는 이 스크립트가 아니라 별도로
# `ros2 run kiosk_vision ...`으로 띄운다 (bringup_level 단계별로 재시작하기 위함).
set -euo pipefail

export GZ_CONFIG_PATH=/usr/share/gz:${GZ_CONFIG_PATH:-}

REPO_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR=~/PX4-Autopilot
PX4_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
PX4_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
LOG_DIR=~/kiosk_drone_ws/logs
mkdir -p "${LOG_DIR}"

echo "== 1) world/model 동기화 =="
cp "${REPO_SIM_DIR}/worlds/kiosk_4walls.sdf" "${PX4_WORLDS}/kiosk_4walls.sdf"
for wall in kiosk_wall kiosk_wall_east kiosk_wall_south kiosk_wall_west; do
    mkdir -p "${PX4_MODELS}/${wall}"
    cp "${REPO_SIM_DIR}/${wall}/model.sdf" "${REPO_SIM_DIR}/${wall}/model.config" "${PX4_MODELS}/${wall}/"
    cp "${REPO_SIM_DIR}/${wall}/${wall}_marker.png" "${PX4_MODELS}/${wall}/"
done
echo "동기화 완료: ${PX4_WORLDS}/kiosk_4walls.sdf, ${PX4_MODELS}/kiosk_wall{,_east,_south,_west}/"

if ! command -v gz >/dev/null 2>&1; then
    echo "경고: gz 명령을 찾을 수 없음. gz sim(Gazebo) 설치 및 PATH를 확인할 것." >&2
elif ! gz sim --versions >/dev/null 2>&1; then
    echo "경고: gz sim이 인식되지 않음. GZ_CONFIG_PATH=${GZ_CONFIG_PATH} 로도 sim8.yaml을 찾지 못하고 있음." >&2
fi

echo "== 2) PX4 SITL (world=kiosk_4walls) =="
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
if [ ! -x "${PX4_BIN}" ]; then
    echo "먼저 빌드하세요: cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth" >&2
    exit 1
fi
cd "${PX4_DIR}"
setsid bash -c "GZ_CONFIG_PATH='${GZ_CONFIG_PATH}' PX4_GZ_WORLD=kiosk_4walls PX4_SIM_MODEL=gz_x500_depth script -qec '${PX4_BIN}' '${LOG_DIR}/px4_sitl_4walls.log'" \
    < /dev/null > /dev/null 2>&1 &
disown
echo "PX4 SITL 백그라운드 기동, 로그: ${LOG_DIR}/px4_sitl_4walls.log"

echo "== 3) Micro XRCE-DDS Agent =="
if ! pgrep -f "MicroXRCEAgent udp4 -p 8888" > /dev/null; then
    setsid bash -c "MicroXRCEAgent udp4 -p 8888 > '${LOG_DIR}/agent.log' 2>&1" \
        < /dev/null > /dev/null 2>&1 &
    disown
    echo "Agent 백그라운드 기동, 로그: ${LOG_DIR}/agent.log"
else
    echo "Agent 이미 실행 중 — 재사용"
fi

echo "== 완료 =="
echo "gz sim / px4 부팅에는 수십 초 걸릴 수 있음. 확인:"
echo "  tail -f ${LOG_DIR}/px4_sitl_4walls.log"
echo "  gz topic -l | grep -i camera   # 카메라 토픽 이름 확인 후 ros_gz_bridge 기동"
echo "  source /opt/ros/jazzy/setup.bash && source ~/kiosk_drone_ws/install/setup.bash && ros2 topic list | grep fmu"
