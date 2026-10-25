"""
Ethernet link check for the switch port QA.

The robot carries the cable from the Jetson's enP7p1s0f0 between the switch
ports. A port works when, after insertion, the link comes up at full speed and
stays up; after extraction the link must drop again. Everything is read from
/sys/class/net/<iface>/, so no root or packets are needed.

As a module:   wait_link('enP7p1s0f0', up=True)  -> result dict
From a shell:  python -m dashboard.check_link              # current state
               python -m dashboard.check_link --wait-up    # after insertion
               python -m dashboard.check_link --wait-down  # after extraction
Exit code 0 = pass, 1 = fail, 2 = interface missing.
"""

import argparse
import json
import sys
import time
from pathlib import Path

IFACE = 'enP7p1s0f0'


def _read(path, cast=str):
    try:
        return cast(path.read_text().strip())
    except (OSError, ValueError):      # carrier/speed raise EINVAL while the interface is down
        return None


def read_link(iface=IFACE):
    """Current link state of iface."""
    p = Path('/sys/class/net') / iface
    if not p.exists():
        raise FileNotFoundError(f'no network interface {iface}')
    carrier = _read(p / 'carrier', int) == 1
    speed = _read(p / 'speed', int) if carrier else None
    return {'iface': iface, 'up': carrier,
            'oper': _read(p / 'operstate'),
            'speed': speed if speed and speed > 0 else None,
            'duplex': _read(p / 'duplex') if carrier else None,
            'carrier_changes': _read(p / 'carrier_changes', int) or 0}


def wait_link(iface=IFACE, up=True, timeout=6.0, stable=1.0, min_speed=1000, poll=0.02):
    """Wait until the link is up (or down) and has held for `stable` seconds.

    Returns {'ok', 'up', 'reason', 't_link', 'speed', 'duplex', 'flaps', ...}:
    t_link is seconds from the call until the link reached the wanted state,
    flaps counts carrier changes beyond the one expected transition.
    """
    t0 = time.monotonic()
    first = read_link(iface)
    since = t0 if first['up'] == up else None
    s = first
    while True:
        now = time.monotonic()
        if since is not None and now - since >= stable:
            break
        if now - t0 > timeout + stable:
            break
        time.sleep(poll)
        s = read_link(iface)
        if s['up'] == up:
            since = since or time.monotonic()
        else:
            since = None
    changes = s['carrier_changes'] - first['carrier_changes']
    res = {'iface': iface, 'want': 'up' if up else 'down', 'up': s['up'],
           'speed': s['speed'], 'duplex': s['duplex'],
           't_link': None if since is None else round(since - t0, 2),
           'flaps': max(0, changes - (first['up'] != up)),
           'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
    if since is None:
        res['ok'] = False
        res['reason'] = (f'no link after {timeout:.0f} s' if up else f'link still up after {timeout:.0f} s')
    elif time.monotonic() - since < stable * 0.99:
        res['ok'] = False
        res['reason'] = f'link {res["want"]} but not stable for {stable:.1f} s ({res["flaps"]} flaps)'
    elif up and min_speed and (s['speed'] or 0) < min_speed:
        res['ok'] = False
        res['reason'] = f'link at {s["speed"]} Mb/s, expected {min_speed}: check the crimp and seating'
    elif up and s['duplex'] != 'full':
        res['ok'] = False
        res['reason'] = f'{s["duplex"]} duplex'
    else:
        res['ok'] = True
        res['reason'] = (f'link up {s["speed"]} Mb/s {s["duplex"]} after {res["t_link"]:.2f} s' if up
                         else f'link down after {res["t_link"]:.2f} s')
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', default=IFACE)
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--wait-up', action='store_true', help='wait for a stable link (after insertion)')
    g.add_argument('--wait-down', action='store_true', help='wait for the link to drop (after extraction)')
    ap.add_argument('--timeout', type=float, default=6.0, help='seconds to wait for the state change')
    ap.add_argument('--stable', type=float, default=1.0, help='seconds the state must hold')
    ap.add_argument('--min-speed', type=int, default=1000, help='Mb/s needed to pass (0: any)')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    try:
        if a.wait_up or a.wait_down:
            res = wait_link(a.iface, up=a.wait_up, timeout=a.timeout, stable=a.stable, min_speed=a.min_speed)
        else:
            res = read_link(a.iface)
            res['ok'] = res['up']
            res['reason'] = f'link up {res["speed"]} Mb/s {res["duplex"]}' if res['up'] else 'no link'
    except FileNotFoundError as e:
        print(json.dumps({'ok': False, 'reason': str(e)}) if a.json else f'ERROR {e}')
        sys.exit(2)
    print(json.dumps(res) if a.json else f'{"PASS" if res["ok"] else "FAIL"} {a.iface}: {res["reason"]}')
    sys.exit(0 if res['ok'] else 1)


if __name__ == '__main__':
    main()
