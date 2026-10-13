#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCRUM-22 IBVS 속도 서보 검증 로그 요약: 상태 전이, 수렴 경과, HOLD 중 명령/오차,
고도, 비전 프레임 간격. 사용: analyze_ibvs_log.py <approach_control.log> <aruco.log>"""
import re
import statistics as S
import sys

ROW = re.compile(
    r'\[(\d+\.\d+)\].*\]\[(\w+)\] (?:fwd_err=([-+\d.]+) lat_err=([-+\d.]+) yaw_err=([-+\d.]+)deg )?'
    r'cmd_frd=\(vf=([-+\d.]+),vr=([-+\d.]+),yaw_rate=([-+\d.]+)deg/s\).*?'
    r'cur_ned\(log\)=\(([-\d.]+),([-\d.]+),([-\d.]+)\) dt=([\d.]+)s')


def main(ctrl_log, aruco_log):
    rows = [m.groups() for m in map(ROW.search, open(ctrl_log)) if m]
    trans, prev = [], None
    for r in rows:
        if r[1] != prev:
            trans.append((float(r[0]), r[1]))
            prev = r[1]
    t_app = next(t for t, s in trans if s == 'APPROACH')
    t_hold = next(t for t, s in trans if s == 'HOLD')

    print('## 상태 전이 (첫 APPROACH 기준 s)')
    for t, s in trans:
        print(f'  {t - t_app:+7.1f}  {s}')

    print('\n## 수렴 경과 (~1.5s 간격)')
    print('  t[s]  state     fwd_err  lat_err  yaw_err | vf     vr     yaw_rate | dt')
    last = -1e9
    for r in rows:
        t = float(r[0])
        if t_app <= t <= t_hold + 0.1 and (t - last >= 1.5 or r[1] == 'HOLD'):
            last = t
            print(f'  {t - t_app:4.1f}  {r[1]:8s} {float(r[2]):+.3f}   {float(r[3]):+.3f}   {float(r[4]):+5.1f}° '
                  f'| {float(r[5]):+.3f} {float(r[6]):+.3f} {float(r[7]):+5.1f}°/s | {r[11]}')
            if r[1] == 'HOLD':
                break
    print(f'  첫 APPROACH -> 첫 HOLD: {t_hold - t_app:.1f}s')

    hold = [r for r in rows if r[1] == 'HOLD']
    nz = [r for r in hold if any(abs(float(x)) > 1e-9 for x in r[5:8])]
    post = [r for r in rows if float(r[0]) >= t_hold]
    dur = float(post[-1][0]) - t_hold
    hold_s = sum(1 for r in post if r[1] == 'HOLD') / len(post) * dur
    print('\n## HOLD')
    print(f'  첫 HOLD 이후 {dur:.0f}s 중 HOLD 비율 {hold_s / dur * 100:.0f}% (로그 샘플 기준)')
    print(f'  HOLD 샘플 {len(hold)}개, 비영 명령 {len(nz)}개')
    for i, name in ((2, 'fwd_err'), (3, 'lat_err')):
        v = [float(r[i]) for r in hold]
        print(f'  {name}: {min(v):+.3f} .. {max(v):+.3f} m')
    v = [float(r[4]) for r in hold]
    print(f'  yaw_err: {min(v):+.1f} .. {max(v):+.1f} deg')
    z = [float(r[10]) for r in post]
    print(f'  고도 z: {min(z):.3f} .. {max(z):.3f} (목표 -1.258)')
    n = [float(r[8]) for r in post]
    e = [float(r[9]) for r in post]
    print(f'  cur_ned(로그 전용) 범위: n {max(n) - min(n):.3f}m, e {max(e) - min(e):.3f}m')
    flips = sum(1 for a, b in zip(trans, trans[1:]) if a[1] == 'HOLD' and b[0] - a[0] < 2.0)
    print(f'  2s 미만 HOLD 체류(chatter) 횟수: {flips}')
    dts = [float(r[11]) for r in rows]
    print(f'  on_timer dt: 중앙 {S.median(dts):.3f}s, 최대 {max(dts):.3f}s')

    # 첫 HOLD 이후 타겟 벽 검출 마커 수 분포 — ≤2개면 마커 게이트(3)에 걸려 pose가 끊긴다.
    mk = []
    for l in open(aruco_log):
        m = re.search(r'\[(\d+\.\d+)\].*walls seen\].*?\((\d)mk', l)
        if m and float(m.group(1)) >= t_hold:
            mk.append(int(m.group(2)))
    low = sum(1 for n in mk if n <= 2)
    print('\n## 첫 HOLD 이후 검출 마커 수')
    print(f'  프레임 {len(mk)}개: ' + ', '.join(f'{k}mk {mk.count(k)}' for k in sorted(set(mk)))
          + f' -> ≤2mk {low}개 ({low / max(len(mk), 1) * 100:.1f}%)')
    lost = sum(1 for _, s in trans if s == 'REACQUIRE')
    print(f'  REACQUIRE 진입 {lost}회')

    ts = [float(re.search(r'\[(\d+\.\d+)\]', l).group(1)) for l in open(aruco_log) if '[PnP target=' in l]
    iv = sorted(b - a for a, b in zip(ts, ts[1:]) if a >= t_app - 1)
    print('\n## 비전 PnP 간격 (APPROACH 이후)')
    print(f'  n={len(iv)} 중앙 {S.median(iv):.2f}s p90 {iv[int(.9 * len(iv))]:.2f}s 최대 {iv[-1]:.2f}s; '
          + ', '.join(f'>{th}s {sum(i > th for i in iv)}' for th in (0.6, 0.8, 1.4, 3.0)))


if __name__ == '__main__':
    main(*sys.argv[1:3])
