"""Python access for insertion scripts (same state as `python -m cadguide`).

    from cadguide.api import target, anchor
    t = target('ls108g', 'port:5')        # {'entry': [x,y,z] m, 'seat', 'axis', 'sigma_mm', 'search_budget_mm', ...}
    ...insert, link up...
    anchor('ls108g', 'port:5', tcp_xyz)   # seated tip position in base [m]
"""
import contextlib
import io
import json

from cadguide import __main__ as cli
from cadguide import state
from cadguide.model import CadModel


def target(obj, feature):
    return cli.predict(CadModel(obj), state.load(obj), feature)


def anchor(obj, feature, xyz, run='', note=''):
    """Record a seated insertion; returns the CLI's JSON result plus its GATE lines."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cli.main(['anchor', obj, feature, *map(str, xyz), '--run', run, '--note', note])
    out = buf.getvalue()
    gates = [line for line in out.splitlines() if line.startswith('GATE')]
    return {**json.loads(out[:out.index('\nGATE')] if gates else out), 'gates': gates}


def brief(objs=None):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cli.main(['ensure', '--compact', *(objs or [])])
    return json.loads(buf.getvalue().splitlines()[-1])
