#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HOLD -> APPROACH 이탈 직후 20Hz setpoint 명령 궤적 (SCRUM-22 R2 이탈 램프 검증).
사용: hold_exit_trace.py <approach_control.log> <setpoint.csv>
setpoint.csv: `ros2 topic echo --csv /fmu/in/trajectory_setpoint` (timestamp[us], pos3, vel3, acc3, jerk3, yaw, yawspeed)"""
import math
import re
import sys

STATE = re.compile(r'\[(\d+\.\d+)\].*\]\[(\w+)\] ')


def main(ctrl_log, sp_csv):
    exits, prev = [], None
    for m in map(STATE.search, open(ctrl_log)):
        if not m:
            continue
        if prev == 'HOLD' and m.group(2) == 'APPROACH':
            exits.append(float(m.group(1)))
        prev = m.group(2)
    sp = []
    for line in open(sp_csv):
        f = line.strip().split(',')
        if len(f) < 15:
            continue
        sp.append((int(f[0]) * 1e-6, math.hypot(float(f[4]), float(f[5])), abs(float(f[14]))))
    print(f'HOLD -> APPROACH 이탈 {len(exits)}회 (로그 샘플 0.5s 단위라 전이 시각은 최대 0.5s 늦음)')
    for te in exits:
        # 로그 throttle(0.5s) 때문에 실제 이탈은 te 이전일 수 있다 -> 직전 1s부터 첫 비영 명령을 찾는다
        win = [s for s in sp if te - 1.0 <= s[0] <= te + 2.5]
        k0 = next((i for i, s in enumerate(win) if s[1] > 1e-6 or s[2] > 1e-6), None)
        if k0 is None:
            print(f'\n  t={te:.1f}: 이탈 구간 setpoint 비영 명령 없음')
            continue
        t0 = win[k0][0]
        seg = [s for s in win[k0:] if s[0] - t0 <= 1.5]
        steps = [b[1] - a[1] for a, b in zip(seg, seg[1:])]
        print(f'\n  이탈 #{exits.index(te) + 1}: 첫 비영 명령 |v_xy| {seg[0][1]:.4f} m/s, '
              f'최대 틱간 증가 {max(steps, default=0):.4f} m/s')
        print('    t-t0[s]  |v_xy|[m/s]  |yawspeed|[deg/s]')
        last = -1.0
        for t, v, w in seg:
            if t - t0 - last >= 0.1 - 1e-3:
                last = t - t0
                print(f'    {t - t0:6.2f}   {v:.4f}       {math.degrees(w):.2f}')


if __name__ == '__main__':
    main(*sys.argv[1:3])
