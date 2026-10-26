"""Tiny binary glTF (.glb) writer: named, single-colour meshes, flat shaded.

Coordinates are written as given (metres, z up, our robot convention). glTF says y up, but
the dashboard's three.js scenes are z up, so it loads the file without a rotation.
"""
import json
import struct

import numpy as np


def _flat(V, F):
    """Un-index the triangles so each face gets its own normal (CAD look, crisp edges)."""
    P = V[F.reshape(-1)].astype(np.float32)
    t = P.reshape(-1, 3, 3)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-20)
    return P, np.repeat(n, 3, axis=0).astype(np.float32)


def write_glb(path, parts, extras=None):
    """parts: [{name, V (N,3) metres, F (M,3) int, color (r,g,b) 0-1, metal, rough}]."""
    blob, views, accs, meshes, mats, nodes = bytearray(), [], [], [], [], []

    def add(arr, target, comp, typ, minmax=False):
        while len(blob) % 4:
            blob.append(0)
        views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': arr.nbytes, 'target': target})
        blob.extend(arr.tobytes())
        a = {'bufferView': len(views) - 1, 'componentType': comp, 'count': len(arr), 'type': typ}
        if minmax:
            a['min'], a['max'] = arr.min(0).tolist(), arr.max(0).tolist()
        accs.append(a)
        return len(accs) - 1

    for p in parts:
        P, N = _flat(np.asarray(p['V'], float), np.asarray(p['F'], np.int64))
        mats.append({'name': p['name'], 'pbrMetallicRoughness': {
            'baseColorFactor': [*p['color'], 1.0], 'metallicFactor': p.get('metal', 0.0),
            'roughnessFactor': p.get('rough', 0.6)}})
        prim = {'attributes': {'POSITION': add(P, 34962, 5126, 'VEC3', True),
                               'NORMAL': add(N, 34962, 5126, 'VEC3')}, 'material': len(mats) - 1}
        meshes.append({'name': p['name'], 'primitives': [prim]})
        nodes.append({'name': p['name'], 'mesh': len(meshes) - 1, 'extras': p.get('extras', {})})

    while len(blob) % 4:
        blob.append(0)
    doc = {'asset': {'version': '2.0', 'generator': 'order2cobot assets/models/cad'},
           'scene': 0, 'scenes': [{'nodes': list(range(len(nodes))), 'extras': extras or {}}],
           'nodes': nodes, 'meshes': meshes, 'materials': mats, 'accessors': accs,
           'bufferViews': views, 'buffers': [{'byteLength': len(blob)}]}
    js = json.dumps(doc, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob)))
        f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        f.write(struct.pack('<II', len(blob), 0x004E4942) + bytes(blob))


def mesh_of(m):
    """(V, F) of a manifold3d Manifold."""
    g = m.to_mesh()
    return np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts)
