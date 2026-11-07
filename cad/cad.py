"""極簡 CAD 工具：多邊形（可含孔）擠出成水密實體、方塊、圓、STL 輸出與檢查。
座標單位 mm。所有實體各自水密，彼此可重疊（切片軟體會自動合併）。
"""
import math, struct
import numpy as np

SEG = 2.0  # 圓弧取樣角度（度）

# ---------------- 2D 工具 ----------------
def area2(pts):
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]; x2, y2 = pts[(i + 1) % len(pts)]
        s += x1 * y2 - x2 * y1
    return s / 2

def circle(cx, cy, r, n=None):
    n = n or max(16, int(360 / SEG / 2) if r > 10 else 24)
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]

def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

def rect_c(cx, cy, w, h):
    return rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)

def arc(r, a0, a1, cx=0.0, cy=0.0):
    n = max(2, int(math.ceil(abs(a1 - a0) / SEG)))
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]

def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def _in_tri(p, a, b, c, eps=1e-9):
    return _cross(a, b, p) >= -eps and _cross(b, c, p) >= -eps and _cross(c, a, p) >= -eps

def _bridge(V, outer, hole):
    m = max(range(len(hole)), key=lambda k: (V[hole[k]][0], V[hole[k]][1]))
    M = V[hole[m]]
    best = None
    n = len(outer)
    for k in range(n):
        a, b = V[outer[k]], V[outer[(k + 1) % n]]
        if (a[1] - M[1]) * (b[1] - M[1]) > 0 or a[1] == b[1]:
            continue
        t = (M[1] - a[1]) / (b[1] - a[1])
        x = a[0] + t * (b[0] - a[0])
        if x <= M[0] + 1e-9:
            continue
        if best is None or x < best[0]:
            best = (x, k)
    if best is None:
        raise RuntimeError('hole bridge failed (hole outside outer?)')
    x, k = best
    a_i, b_i = k, (k + 1) % n
    I = (x, M[1])
    vi = a_i if V[outer[a_i]][0] > V[outer[b_i]][0] else b_i
    if math.dist(I, V[outer[a_i]]) < 1e-9:
        vi = a_i
    elif math.dist(I, V[outer[b_i]]) < 1e-9:
        vi = b_i
    else:
        Pk = V[outer[vi]]
        tri = (M, I, Pk) if _cross(M, I, Pk) > 0 else (M, Pk, I)
        cands = []
        for j in range(n):
            p = V[outer[j]]
            if outer[j] == outer[vi]:
                continue
            if _cross(V[outer[j - 1]], p, V[outer[(j + 1) % n]]) < 0 and _in_tri(p, *tri, eps=-1e-9):
                cands.append((abs(math.atan2(p[1] - M[1], p[0] - M[0])), math.dist(p, M), j))
        if cands:
            vi = min(cands)[2]
    rot = hole[m:] + hole[:m] + [hole[m]]
    return outer[:vi + 1] + rot + [outer[vi]] + outer[vi + 1:]

def _earclip(V, poly):
    L = list(poly)
    P = np.array(V)
    tris = []
    guard = 0
    while len(L) > 3:
        guard += 1
        if guard > 200000:
            raise RuntimeError('earclip guard')
        idx = np.array(L)
        pts = P[idx]
        prv, nxt = np.roll(pts, 1, 0), np.roll(pts, -1, 0)
        cr = (pts[:, 0] - prv[:, 0]) * (nxt[:, 1] - prv[:, 1]) - (pts[:, 1] - prv[:, 1]) * (nxt[:, 0] - prv[:, 0])
        reflex = np.where(cr <= 1e-12)[0]
        rpts, ridx = pts[reflex], idx[reflex]
        n = len(L)
        found = False
        for i in np.where(cr > 1e-12)[0]:
            ia, ib, ic = L[i - 1], L[i], L[(i + 1) % n]
            a, b, c = P[ia], P[ib], P[ic]
            msk = (ridx != ia) & (ridx != ib) & (ridx != ic)
            for vv in (a, b, c):
                msk &= ~((np.abs(rpts[:, 0] - vv[0]) < 1e-9) & (np.abs(rpts[:, 1] - vv[1]) < 1e-9))
            Q = rpts[msk]
            if len(Q):
                d1 = (b[0] - a[0]) * (Q[:, 1] - a[1]) - (b[1] - a[1]) * (Q[:, 0] - a[0])
                d2 = (c[0] - b[0]) * (Q[:, 1] - b[1]) - (c[1] - b[1]) * (Q[:, 0] - b[0])
                d3 = (a[0] - c[0]) * (Q[:, 1] - c[1]) - (a[1] - c[1]) * (Q[:, 0] - c[0])
                if np.any((d1 >= -1e-9) & (d2 >= -1e-9) & (d3 >= -1e-9)):
                    continue
            tris.append((ia, ib, ic)); del L[i]; found = True; break
        if not found:
            for i in range(n):
                if abs(cr[i]) <= 1e-12:
                    del L[i]; found = True; break
        if not found:
            raise RuntimeError('ear clipping failed')
    if abs(_cross(P[L[0]], P[L[1]], P[L[2]])) > 1e-12:
        tris.append(tuple(L))
    return tris

# ---------------- 實體 ----------------
class Solid:
    def __init__(self, verts, faces):
        self.v = np.asarray(verts, float); self.f = np.asarray(faces, int)

    def map(self, fn, flip=False):
        v = np.array([fn(*p) for p in self.v])
        f = self.f[:, ::-1] if flip else self.f
        return Solid(v, f)

def extrude(outer, holes=(), z0=0.0, z1=1.0):
    """2D 多邊形（外框 + 孔）沿 z 擠出。"""
    V = []
    def add(pts):
        ids = []
        for p in pts:
            V.append((float(p[0]), float(p[1]))); ids.append(len(V) - 1)
        return ids
    o = add(outer)
    if area2([V[i] for i in o]) < 0: o.reverse()
    hs = []
    for h in holes:
        hr = add(h)
        if area2([V[i] for i in hr]) > 0: hr.reverse()
        hs.append(hr)
    # 三角化在微旋轉後的座標上做，避開大量軸向對齊造成的退化情況；
    # 旋轉不改變拓撲與方向，所得三角形對原座標同樣有效。
    tris, last = None, None
    for deg in (0.731, 1.913, 3.17, 5.3, 0.123, 7.77, 11.1, 17.3, 23.9, 31.7):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        Vr = [(x * c - y * s, x * s + y * c) for x, y in V]
        try:
            merged = list(o)
            for h in sorted(hs, key=lambda h: -max(Vr[i][0] for i in h)):
                merged = _bridge(Vr, merged, h)
            tris = _earclip(Vr, merged)
            break
        except RuntimeError as e:
            last = e
    if tris is None:
        raise last
    rings = [o] + hs
    nb = len(V)
    verts = [(x, y, z0) for x, y in V] + [(x, y, z1) for x, y in V]
    faces = []
    for a, b, c in tris:
        faces.append((a + nb, b + nb, c + nb)); faces.append((a, c, b))
    for r in rings:
        for k in range(len(r)):
            a, b = r[k], r[(k + 1) % len(r)]
            faces.append((a, b, b + nb)); faces.append((a, b + nb, a + nb))
    return Solid(verts, faces)

def extrude_xz(outer, holes, y0, y1):
    """在 XZ 平面畫（u=x, v=z），沿 y 擠出：做垂直牆。"""
    s = extrude(outer, holes, y0, y1)
    return s.map(lambda u, v, w: (u, w, v), flip=True)

def extrude_yz(outer, holes, x0, x1):
    """在 YZ 平面畫（u=y, v=z），沿 x 擠出。"""
    s = extrude(outer, holes, x0, x1)
    return s.map(lambda u, v, w: (w, u, v), flip=False)

def box(x0, y0, z0, x1, y1, z1):
    return extrude(rect(x0, y0, x1, y1), [], z0, z1)

def check(s):
    edges = {}
    for f in s.f:
        for i in range(3):
            e = (f[i], f[(i + 1) % 3]); edges[e] = edges.get(e, 0) + 1
    bad = sum(1 for e, c in edges.items() if c != 1 or edges.get((e[1], e[0]), 0) != 1)
    t = s.v[s.f]
    vol = np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6
    return bad, vol

def write_stl(path, solids, name='part'):
    tris = np.concatenate([s.v[s.f] for s in solids])
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True); ln[ln == 0] = 1
    nrm /= ln
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        data = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        data['n'] = nrm; data['v'] = tris
        f.write(data.tobytes())
    return tris

def report(name, solids):
    tot = 0
    for i, s in enumerate(solids):
        bad, vol = check(s)
        assert bad == 0, f'{name}: 第 {i} 個實體不是水密（{bad} 條壞邊）'
        assert vol > 0, f'{name}: 第 {i} 個實體法線反了'
        tot += vol
    allv = np.concatenate([s.v for s in solids])
    mn, mx = allv.min(0), allv.max(0)
    return dict(n=len(solids), tris=sum(len(s.f) for s in solids), vol=tot,
                bbox=tuple(np.round(mx - mn, 1)), mn=tuple(np.round(mn, 1)), mx=tuple(np.round(mx, 1)))
