"""CAD guidance for insertion tasks: register -> predict -> insert -> anchor -> predict ...

    python -m cadguide ensure   [obj ...] [--refresh] [--compact]   # THE tool: targets in base mm
    python -m cadguide status   [obj]
    python -m cadguide register obj [--survey data/ports.yaml]     # cameras only, no motion
    python -m cadguide predict  obj FEATURE [--json]               # e.g. port:5, hole:c12
    python -m cadguide anchor   obj FEATURE X Y Z [--run RUN] [--note TEXT]   # seated, metres, tip TCP
    python -m cadguide reset    obj [--anchors]                    # drop fit (and anchors)

obj: ls108g | breadboard. The table plane fixes z / roll / pitch; anchors solve the
in-plane x / y / yaw and the pitch: 6 DOF from the cameras at ~1 mm becomes 3 DOF + pitch
from seated insertions at ~0.2 mm. See .claude/skills/cad-guidance/SKILL.md.
"""
import argparse
import json
import sys
import time

import numpy as np

from cadguide import fit as F
from cadguide import state
from cadguide.model import ROOT, CadModel, yaw_of

# a new anchor this far from its prediction means the object moved (taped, ~1 mm creep)
MOVE_TOL = {'switch': 1.0e-3, 'breadboard': 0.6e-3}
# prior sigmas by registration method
SIGMA = {'cloud': dict(sigma_t=2.0e-3, sigma_yaw=np.radians(1.0), sigma_scale=[0.015, 0.015]),
         'survey': dict(sigma_t=1.0e-3, sigma_yaw=np.radians(0.3), sigma_scale=[0.04, 0.04]),
         'touch': dict(sigma_t=0.7e-3, sigma_yaw=np.radians(1.0), sigma_scale=[0.04, 0.04])}


def used(s):
    return [a for a in s['anchors'] if a.get('used', True)]


def solve(model, s):
    """Fit of the registration + the used anchors (None if not registered)."""
    return F.Fit(model, s['registration'], used(s)) if s.get('registration') else None


def refit(model, s):
    fit = solve(model, s)
    if fit is None:
        s['fit'] = None
        return None
    s['fit'] = {'T': fit.T, 'scale': fit.scale, 'stamp': state.now(), 'anchors': len(used(s)),
                'epochs': fit.n_ep, 'residual_mm': [round(r * 1000, 3) for r in fit.resid],
                'yaw_deg': round(np.degrees(yaw_of(fit.T)), 3),
                'pitch_mm': [round(2.54 * fit.scale[0], 4), round(2.54 * fit.scale[1], 4)]
                if model.kind == 'breadboard' else round(model.spec['pitch'] * fit.scale[0], 4)}
    return fit


def predict(model, s, fid, fit=None):
    fit = fit or solve(model, s)
    if fit is None:
        raise SystemExit(f'{model.name}: not registered - run: python -m cadguide register {model.name}')
    f = model.feature(fid)
    seat = model.feature(fid.split('/')[0] + '/seat')
    lat, along = fit.sigma(fid)
    n = len(used(s))
    return {'object': model.name, 'feature': fid,
            'entry': model.place(fit.T, fit.scale, f.p).round(6).tolist(),
            'seat': model.place(fit.T, fit.scale, seat.p).round(6).tolist(),
            'axis': (fit.T[:3, :3] @ f.axis).round(6).tolist(),
            'yaw_deg': round(np.degrees(yaw_of(fit.T)), 3),
            'sigma_mm': {'lateral': round(lat * 1000, 3), 'along': round(along * 1000, 3)},
            'search_budget_mm': round(max(0.3, 3 * lat * 1000), 2),
            'basis': f"{n} anchor(s) in {fit.n_ep} epoch(s) on the {s['registration'].get('kind')} registration"}


def cmd_status(args):
    for name in ([args.obj] if args.obj else state.OBJECTS):
        s = state.load(name)
        r, f = s.get('registration'), s.get('fit')
        print(f'== {name}')
        if not r:
            print('   not registered')
            continue
        print(f"   registration {r['stamp']}  {r['method']}")
        for k in ('image', 'residual_mm', 'rms_mm', 'pitch_mm', 'yaw_deg', 'rect_size_error_mm', 'depth_vs_image_mm'):
            if k in r:
                print(f'     {k}: {r[k]}')
        for a in s['anchors']:
            print(f"   anchor {a['stamp']} {a['feature']:<12} epoch {a.get('epoch', 0)} "
                  f"{'used' if a.get('used', True) else 'dropped'}  pred-err {a.get('pred_err_mm')} mm  {a.get('note', '')}")
        if f:
            print(f"   fit {f['stamp']}: {f['anchors']} anchors / {f.get('epochs', 1)} epochs, residual {f['residual_mm']} mm, "
                  f"yaw {f['yaw_deg']} deg, pitch {f['pitch_mm']} mm")


def cmd_register(args):
    from cadguide import register as R
    model = CadModel(args.obj)
    out = ROOT / 'results' / f"{time.strftime('%Y%m%d-%H%M%S')}_cad_register"
    if args.touch:
        prev = state.current(state.load(args.obj))
        T, scale, info = R.from_touch(model, *args.touch, yaw=yaw_of(prev[0]) if prev else None)
        kind = 'touch'
    elif args.survey:
        T, scale, info = R.from_survey(model, args.survey)
        kind = 'survey'
    else:
        T, scale, info = R.from_cloud(model, out)
        kind = 'cloud'
        info['overlay'] = str(out / f'{model.name}_overlay.png')
    s = state.load(args.obj)
    old = state.current(s)
    ok = gate(model, kind, info)
    if not ok and s.get('registration') and not args.force:
        print(f"NOT SAVED: the gate failed and {args.obj} already has a {s['registration'].get('kind')} registration "
              f"from {s['registration']['stamp']} (use --force to replace it anyway)")
        return
    reg = {'T': T, 'scale': scale, 'stamp': state.now(), 'kind': kind, **SIGMA[kind], **info}
    f = s.get('fit')
    if f and f['anchors'] >= 2:
        # the pitch learned from seated anchors survives a re-registration (the part did not stretch)
        reg['scale'], reg['sigma_scale'] = f['scale'], [0.003, 0.003]
        info['pitch_from_anchors'] = f['pitch_mm']
    # a new registration starts a new pose history: older anchors no longer apply
    for a in used(s):
        a['used'], a['dropped'] = False, f"superseded by the {kind} registration {reg['stamp']}"
    s['registration'] = reg
    refit(model, s)
    if old is not None:
        d = (T[:3, 3] - old[0][:3, 3]) * 1000
        info['moved_vs_previous_mm'] = [round(float(v), 2) for v in d]
    state.save(args.obj, s)
    print(json.dumps({'object': args.obj, 'yaw_deg': info.get('yaw_deg'),
                      'origin_mm': (T[:3, 3] * 1000).round(2).tolist(), **info}))


def gate(model, kind, info):
    """Print PASS/WARN lines the skill reads."""
    if kind == 'touch':
        print('GATE PASS: touch registration (contact-measured face / top / left end)')
        return True
    if kind == 'survey':
        ok = info['rms_mm'] < 1.0
        print(f"GATE {'PASS' if ok else 'WARN'}: survey fit rms {info['rms_mm']} mm (< 1.0)")
        return ok
    # the image fit decides: depth is often occluded (cables, the arm) or missing on white plastic,
    # so the depth rectangle only seeds it
    img = info['image']
    ok = img['edge_inliers'] > 0.55 and img['edge_cost_px'] < 2.5
    print(f"GATE {'PASS' if ok else 'WARN'}: image edge inliers {img['edge_inliers']} (> 0.55), "
          f"edge cost {img['edge_cost_px']} px (< 2.5); depth rectangle size error {info['rect_size_error_mm']} mm "
          f"(large = occluded), image moved it {info['depth_vs_image_mm']} mm. Look at {info.get('overlay')}")
    return ok


def cmd_predict(args):
    model = CadModel(args.obj)
    s = state.load(args.obj)
    p = predict(model, s, args.feature)
    print(json.dumps(p) if args.json else json.dumps(p, indent=1))


def cmd_anchor(args):
    model = CadModel(args.obj)
    s = state.load(args.obj)
    if not s.get('registration'):
        raise SystemExit(f'{args.obj}: register first')
    meas = np.array([args.x, args.y, args.z])
    fid = args.feature if '/' in args.feature else args.feature + '/seat'
    p = predict(model, s, fid)
    # only the error across the insertion axis is evidence (depth depends on the seat force)
    d, ax = meas - np.array(p['seat']), np.array(p['axis'])
    lat_v = d - ax * (d @ ax)
    lat = float(np.linalg.norm(lat_v))
    a = {'feature': fid, 'base': meas.tolist(), 'stamp': state.now(), 'run': args.run, 'note': args.note,
         'pred_err_mm': round(lat * 1000, 3), 'pred_err_vec_mm': (lat_v * 1000).round(3).tolist(), 'used': True}
    prev = used(s)
    # moved = off by more than the creep tolerance AND more than the prediction itself is
    # uncertain (far from the anchors yaw / pitch are not known yet: that is not a move)
    tol = max(MOVE_TOL[model.kind], 3 * p['sigma_mm']['lateral'] / 1000)
    moved = bool(prev) and lat > tol
    # the part slid (taped, ~1 mm per seat): new epoch = new pose, same rigid part and pitch
    a['epoch'] = (prev[-1].get('epoch', 0) + int(moved)) if prev else 0
    s['anchors'].append(a)
    refit(model, s)
    state.save(args.obj, s)
    f = s['fit']
    print(json.dumps({'anchor': a, 'moved': moved, 'fit': {k: f[k] for k in
                      ('anchors', 'epochs', 'residual_mm', 'yaw_deg', 'pitch_mm')}}, indent=1))
    limit = 0.3 if model.kind == 'switch' else 0.2
    if prev and not moved:
        print(f"GATE {'PASS' if lat * 1000 < max(limit, 3 * p['sigma_mm']['lateral']) else 'WARN'}: "
              f"anchor {lat * 1000:.2f} mm lateral from prediction (sigma {p['sigma_mm']['lateral']} mm)")
    if moved:
        print(f'GATE WARN: part moved ({lat * 1000:.2f} mm > {tol * 1000:.2f} mm): new epoch '
              f"{a['epoch']} (pitch kept, pose re-anchored); a second far anchor pins its yaw")
    if max(f['residual_mm']) > 3 * F.SIGMA_MEAS * 1000:
        print(f"GATE WARN: fit residual {max(f['residual_mm'])} mm: an anchor disagrees with the rigid model "
              '(bad seat, regrip, or a turn): check, or reset --anchors')


def brief(model, s, max_age_min):
    """Everything an insertion step needs, in base mm, from one call."""
    fit = solve(model, s)
    reg = s['registration']
    measured = reg.get('measured', reg['stamp'])          # survey: when the photos were taken
    age = (time.time() - time.mktime(time.strptime(measured, '%Y-%m-%dT%H:%M:%S'))) / 60
    last = max([a['stamp'] for a in used(s)] + [measured])
    fresh = (time.time() - time.mktime(time.strptime(last, '%Y-%m-%dT%H:%M:%S'))) / 60
    mm = lambda v: [round(float(x) * 1000, 2) for x in v]  # noqa: E731
    out = {'source': reg['kind'], 'registered_min_ago': round(age), 'last_evidence_min_ago': round(fresh),
           'stale': fresh > max_age_min, 'anchors': len(used(s)), 'epochs': fit.n_ep,
           'yaw_deg': round(np.degrees(yaw_of(fit.T)), 3), 'fit_residual_mm': s['fit']['residual_mm'] if s.get('fit') else []}
    R = fit.T[:3, :3]
    if model.kind == 'switch':
        out['pitch_mm'] = round(model.spec['pitch'] * fit.scale[0], 3)
        out['axis'] = (R @ [0, 1, 0]).round(5).tolist()
        out['ports'] = {}
        for k in range(1, 9):
            f = f'port:{k}'
            lat = fit.sigma(f)[0]
            out['ports'][k] = {'entry': mm(model.place(fit.T, fit.scale, model.feature(f).p)),
                               'sigma': round(lat * 1000, 2), 'budget': round(max(0.3, 3 * lat * 1000), 2)}
    else:
        sp = model.spec
        a1 = model.feature('hole:a1').p
        out['pitch_mm'] = [round(2.54 * fit.scale[0], 4), round(2.54 * fit.scale[1], 4)]
        out['axis'] = (R @ [0, 0, -1]).round(5).tolist()
        out['grid'] = {'a1': mm(model.place(fit.T, fit.scale, a1)),
                       'col_step': mm(R @ [0.00254 * fit.scale[0], 0, 0]),
                       'row_step': mm(R @ [0, -0.00254 * fit.scale[1], 0]),
                       'channel_extra': mm(R @ [0.00508 * fit.scale[0], 0, 0]),
                       'rule': f'terminal (col c in a..j, row r in 1..{sp["rows"]}) = a1 + k*col_step + '
                               '(r-1)*row_step, k = index of c, plus channel_extra for f..j; '
                               'rails: python -m cadguide predict breadboard hole:L+1'}
        out['sigma'] = {h: round(fit.sigma(f'hole:{h}')[0] * 1000, 2) for h in ('a1', f'j{sp["rows"]}')}
    # next anchor: the feature farthest from the used ones pins yaw / pitch best
    fs = [f for f in model.features() if model.kind == 'switch' or f[5:6].isalpha() and f[5:6] in 'aj']
    have = [model.feature(a['feature']).p for a in used(s) if a.get('epoch', 0) == fit.n_ep - 1]
    if have:
        far = max(fs, key=lambda f: min(np.linalg.norm(model.feature(f).p - h) for h in have))
        out['next_anchor'] = f'{far} (farthest from this epoch\'s anchors: pins yaw and pitch)'
    else:
        out['next_anchor'] = 'any (the first insertion sets the in-plane offset)'
    return out


def cmd_ensure(args):
    """Register what is missing or stale (cameras only), then print the brief for each object."""
    from cadguide import register as R
    res = {}
    for name in args.obj or state.OBJECTS:
        if name not in state.OBJECTS:
            raise SystemExit(f'unknown object {name!r}: {state.OBJECTS}')
        model, s = CadModel(name), state.load(name)
        reg = s.get('registration')
        need = reg is None or args.refresh
        if need:
            out = ROOT / 'results' / f"{time.strftime('%Y%m%d-%H%M%S')}_cad_register"
            try:
                T, scale, info = R.from_cloud(model, out)
                info['overlay'] = str(out / f'{model.name}_overlay.png')
                if gate(model, 'cloud', info) or reg is None:
                    for a in used(s):
                        a['used'], a['dropped'] = False, 'superseded by a new cloud registration'
                    s['registration'] = {'T': T, 'scale': scale, 'stamp': state.now(), 'kind': 'cloud',
                                         **SIGMA['cloud'], **info}
                    refit(model, s)
                    state.save(name, s)
            except R.RegistrationFailed as e:
                print(f'GATE WARN: {name}: {e}')
        if not s.get('registration'):
            res[name] = {'status': 'unregistered', 'do': (
                'python -m porttest.port_survey (moves the gripper cam), then '
                'python -m cadguide register ls108g --survey data/ports.yaml') if name == 'ls108g'
                else 'clear the view of the fixed camera and run python -m cadguide register breadboard'}
            continue
        b = brief(model, s, args.max_age_min)
        b['status'] = 'stale' if b['stale'] else 'ok'
        res[name] = b
    print(json.dumps(res, separators=(',', ':')) if args.compact else json.dumps(res, indent=1))


def cmd_reset(args):
    s = state.load(args.obj)
    s['fit'] = None
    if args.anchors:
        s['anchors'] = []
    model = CadModel(args.obj)
    if s.get('registration'):
        refit(model, s)
    state.save(args.obj, s)
    print(f'{args.obj}: reset ({"anchors cleared" if args.anchors else "anchors kept"})')


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m cadguide', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('status')
    p.add_argument('obj', nargs='?', choices=state.OBJECTS)
    p.set_defaults(fn=cmd_status)
    p = sub.add_parser('register')
    p.add_argument('obj', choices=state.OBJECTS)
    p.add_argument('--survey', help='port survey yaml (switch): precise registration')
    p.add_argument('--force', action='store_true', help='save even if the gate fails')
    p.add_argument('--touch', nargs=3, type=float, metavar=('FACE_Y', 'TOP_Z', 'LEFT_X'),
                   help='switch: plug-tip touch references [m] (porttest.rj45_b touch_refs)')
    p.set_defaults(fn=cmd_register)
    p = sub.add_parser('predict')
    p.add_argument('obj', choices=state.OBJECTS)
    p.add_argument('feature')
    p.add_argument('--json', action='store_true')
    p.set_defaults(fn=cmd_predict)
    p = sub.add_parser('anchor')
    p.add_argument('obj', choices=state.OBJECTS)
    p.add_argument('feature', help='port:4 (= port:4/seat) or hole:a1')
    for k in 'xyz':
        p.add_argument(k, type=float, help=f'seated tip {k} in base [m]')
    p.add_argument('--run', default='')
    p.add_argument('--note', default='')
    p.set_defaults(fn=cmd_anchor)
    p = sub.add_parser('ensure', help='register if needed, then the targets of each object in base mm')
    p.add_argument('obj', nargs='*', help='ls108g, breadboard (default: both)')
    p.add_argument('--refresh', action='store_true', help='re-register from the fixed camera')
    p.add_argument('--max-age-min', type=float, default=60, help='older evidence is flagged stale')
    p.add_argument('--compact', action='store_true')
    p.set_defaults(fn=cmd_ensure)
    p = sub.add_parser('reset')
    p.add_argument('obj', choices=state.OBJECTS)
    p.add_argument('--anchors', action='store_true')
    p.set_defaults(fn=cmd_reset)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == '__main__':
    sys.exit(main())
