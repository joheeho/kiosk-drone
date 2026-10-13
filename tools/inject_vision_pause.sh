#!/usr/bin/env bash
# 비전 유실 주입: aruco_pnp_node를 SIGSTOP해 비전 명령 hold/decay, REACQUIRE 경로를 실제로 태운다.
# 사용: run_full_auto.sh 기동 직후 `tools/inject_vision_pause.sh <출력dir> [모드]`
#   approach      : 접근 중(fwd_err>0.6, vf>0) 2s 정지 — cmd hold->decay 확인 (기본)
#   hold <s>...   : HOLD 15s 유지 후 <s>초 정지, 다시 HOLD 복귀 15s 후 다음 값 — REACQUIRE(후퇴) 확인
# 기록: <출력dir>/setpoint.csv(TrajectorySetpoint), pause_log(시작/끝 epoch, 길이)
set +u; source /opt/ros/jazzy/setup.bash; source ~/kiosk_drone_ws/install/setup.bash; set -u
OUT=${1:?출력 디렉터리}; MODE=${2:-approach}; shift 2 2>/dev/null || shift $#
L=~/kiosk_drone_ws/logs/approach_control.log
mkdir -p "$OUT"; : > "$OUT/pause_log"
until grep -q "Arm command sent" $L 2>/dev/null; do sleep 1; done
ros2 topic echo --csv /fmu/in/trajectory_setpoint > "$OUT/setpoint.csv" 2>/dev/null &
EP=$!
pause() {  # pause <초>
    local pid; pid=$(pgrep -f "[k]iosk_vision/lib/kiosk_vision/aruco_pnp_node" | tail -1)
    local t0; t0=$(date +%s.%N); kill -STOP "$pid"; sleep "$1"; kill -CONT "$pid"
    echo "$t0 $(date +%s.%N) $1" >> "$OUT/pause_log"
}
wait_hold_s() {  # 마지막 로그 상태가 HOLD로 연속 <초> 유지될 때까지
    local need=$1 since=""
    while :; do
        if tail -n 1 $L | grep -q "\]\[HOLD\]"; then
            since=${since:-$(date +%s)}; [ $(( $(date +%s) - since )) -ge "$need" ] && return
        else since=""; fi
        sleep 0.5
    done
}
if [ "$MODE" = approach ]; then
    until tail -n 3 $L | grep -qE "\[APPROACH\] fwd_err=\+(0\.[6-9]|1\.)[0-9]* .*vf=\+0\.[1-3]"; do sleep 0.2; done
    pause 2.0; sleep 30
else
    for s in "$@"; do wait_hold_s 15; pause "$s"; done
    wait_hold_s 15
fi
kill $EP; echo "inject done mode=$MODE"
