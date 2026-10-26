"""Minimal reader for ACIS binary (.sab) files, enough to pull geometry out of a CAD export.

No open-source tool tessellates ACIS, so the builders read exact numbers (vertices, edge
end points, planes) from the B-rep and rebuild the part from them.

    ents = read('breadboard.sab')     # [(type name, [(tag, value), ...]), ...]

Entity pointers (tag 0x0C) are indices into `ents`. Positions (0x13) and vectors (0x14)
are in file units (metres for SpaceClaim exports).
"""
import struct

POS, VEC, PTR = 0x13, 0x14, 0x0C


def read(path):
    data = open(path, 'rb').read()
    if not data.startswith(b'ACIS BinaryFile'):
        raise ValueError(f'{path}: not an ACIS binary file')

    def tok(i):
        t = data[i]
        i += 1
        if t == 0x04:
            return t, struct.unpack_from('<i', data, i)[0], i + 4
        if t == 0x06:
            return t, struct.unpack_from('<d', data, i)[0], i + 8
        if t in (0x07, 0x0D, 0x0E, 0x12):
            n = data[i]
            return t, data[i + 1:i + 1 + n].decode('latin1'), i + 1 + n
        if t == 0x08:
            n = struct.unpack_from('<H', data, i)[0]
            return t, data[i + 2:i + 2 + n].decode('latin1'), i + 2 + n
        if t == 0x09:
            n = struct.unpack_from('<I', data, i)[0]
            return t, data[i + 4:i + 4 + n].decode('latin1'), i + 4 + n
        if t in (PTR, 0x15):
            return t, struct.unpack_from('<i', data, i)[0], i + 4
        if t in (POS, VEC):
            return t, struct.unpack_from('<3d', data, i), i + 24
        if t in (0x0A, 0x0B, 0x0F, 0x10, 0x11):
            return t, None, i
        raise ValueError(f'{path}: unknown tag {t:#x} at byte {i - 1}')

    i = 15 + 16                       # magic + 4 header ints
    for _ in range(6):                # product, version, date, units, resabs, resnor
        _, _, i = tok(i)
    ents, name = [], []
    while i < len(data):
        t, v, i = tok(i)
        if t == 0x0E:                 # sub-type prefix of the next entity name
            name.append(v)
            continue
        if t == 0x0D:
            name.append(v)
            if v == 'End-of-ACIS-data':
                break
            cur = []
            ents.append(('-'.join(name), cur))
            name = []
            continue
        if t != 0x11:
            cur.append((t, v))
    return ents


def points(ents):
    """All B-rep vertex positions, (N, 3) in file units."""
    return [[v for t, v in toks if t == POS][0] for n, toks in ents if n == 'point']


def edge_segments(ents):
    """(start, end) of every edge: ACIS stores the edge's end points after its curve."""
    out = []
    for n, toks in ents:
        if n == 'edge':
            p = [v for t, v in toks if t == POS]
            if len(p) >= 2:
                out.append((p[-2], p[-1]))
    return out
