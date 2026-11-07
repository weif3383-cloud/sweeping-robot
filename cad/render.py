"""簡易 z-buffer 算圖：把 STL 三角形畫成有光影的立體圖（PNG）。"""
import numpy as np
from PIL import Image

def render(tris, path, elev=35, azim=-55, size=(1100, 820), color=(232, 199, 102), bg=(246, 247, 246), margin=40):
    t = np.asarray(tris, float)
    c = t.reshape(-1, 3).mean(0)
    t = t - c
    a, e = np.radians(azim), np.radians(elev)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, np.cos(e - np.pi / 2), -np.sin(e - np.pi / 2)], [0, np.sin(e - np.pi / 2), np.cos(e - np.pi / 2)]])
    M = Rx @ Rz
    p = t @ M.T                      # x right, y up, z toward viewer
    W, H = size
    xy = p[:, :, :2].reshape(-1, 2)
    lo, hi = xy.min(0), xy.max(0)
    sc = min((W - 2 * margin) / (hi[0] - lo[0]), (H - 2 * margin) / (hi[1] - lo[1]))
    px = (p[:, :, 0] - (lo[0] + hi[0]) / 2) * sc + W / 2
    py = H / 2 - (p[:, :, 1] - (lo[1] + hi[1]) / 2) * sc
    pz = p[:, :, 2]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    nv = n @ M.T
    light = np.array([-0.35, 0.55, 0.76]); light /= np.linalg.norm(light)
    img = np.empty((H, W, 3)); img[:] = bg
    zb = np.full((H, W), -1e18)
    for i in range(len(t)):
        if nv[i, 2] <= 0:   # back-face
            continue
        x0, x1 = int(max(0, np.floor(px[i].min()))), int(min(W - 1, np.ceil(px[i].max())))
        y0, y1 = int(max(0, np.floor(py[i].min()))), int(min(H - 1, np.ceil(py[i].max())))
        if x1 < x0 or y1 < y0:
            continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        (ax, ay), (bx, by), (cx, cy) = zip(px[i], py[i])
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-9:
            continue
        w0 = ((by - cy) * (X - cx) + (cx - bx) * (Y - cy)) / d
        w1 = ((cy - ay) * (X - cx) + (ax - cx) * (Y - cy)) / d
        w2 = 1 - w0 - w1
        m = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not m.any():
            continue
        z = w0 * pz[i, 0] + w1 * pz[i, 1] + w2 * pz[i, 2]
        sub = zb[y0:y1 + 1, x0:x1 + 1]
        upd = m & (z > sub)
        sub[upd] = z[upd]
        shade = 0.38 + 0.62 * max(0.0, float(nv[i] @ light))
        img[y0:y1 + 1, x0:x1 + 1][upd] = np.array(color) * shade
    # 輪廓描邊：深度不連續處加深
    gz = np.zeros_like(zb); valid = zb > -1e17
    dz = np.zeros_like(zb)
    dz[1:, :] = np.maximum(dz[1:, :], np.abs(np.diff(np.where(valid, zb, 0), axis=0)))
    dz[:, 1:] = np.maximum(dz[:, 1:], np.abs(np.diff(np.where(valid, zb, 0), axis=1)))
    edge = dz > 1.2
    img[edge] = img[edge] * 0.45
    Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(path)
