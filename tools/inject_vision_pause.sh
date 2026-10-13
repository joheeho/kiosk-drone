#!/usr/bin/env bash
# 접근 중(fwd_err>0.6, vf>0) aruco_pnp_node를 2s SIGSTOP -> 비전 명령 hold/decay 경로 확인
# 사용: run_full_auto.sh 기동 직후 `tools/inject_vision_pause.sh <출력dir>` (setpoint.csv, pause_start/end 기록)
set +u; source /opt/ros/jazzy/setup.bash; source ~/kiosk_drone_ws/install/setup.bash; set -u
L=~/kiosk_drone_ws/logs/approach_control.log; OUT=${1:?출력 디렉터리}
until grep -q "Arm command sent" $L 2>/dev/null; do sleep 1; done
ros2 topic echo --csv /fmu/in/trajectory_setpoint > $OUT/setpoint.csv 2>/dev/null &
EP=$!
until tail -n 3 $L | grep -qE "\[APPROACH\] fwd_err=\+(0\.[6-9]|1\.)[0-9]* .*vf=\+0\.[1-3]"; do sleep 0.2; done
PID=$(pgrep -f "[k]iosk_vision/lib/kiosk_vision/aruco_pnp_node" | tail -1)
date +%s.%N > $OUT/pause_start; kill -STOP $PID
sleep 2.0
kill -CONT $PID; date +%s.%N > $OUT/pause_end
sleep 30; kill $EP
echo "inject done pid=$PID"
