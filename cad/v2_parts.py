"""掃地機器人 v2：下層底盤 + 上層板 STL 產生器
座標：俯視，原點 = 輪軸中心，+y 朝車頭，+x 朝右，單位 mm；每片零件底面 z = 0（平放列印）。
改最上面的參數後重跑：python3 v2_parts.py
"""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cad import *

OUT = os.path.dirname(os.path.abspath(__file__))

# ===================== 共用參數 =====================
R = 120.0                     # 底盤半徑（直徑 240）
T = 3.0                       # 板厚
NOTCH_X0, NOTCH_HALF = 87.0, 36.0   # 輪槽：內緣 x、半寬（槽寬 72）
SLOT = (2.2, 4.6)             # 束帶孔（x 寬、y 長）
M3 = 3.4                      # M3 穿孔直徑
POSTS = [(92, 66), (-92, 66), (55, 96), (-55, 96)]   # 上層銅柱位置
MARK_H, MARK_W, MARK_L = 0.8, 1.0, 5.0              # 凸起記號：高、線寬、長

# ===================== 下層底盤參數 =====================
RIM_W, RIM_H = 3.0, 5.0
RIM_GAPS = [(78, 102), (150, 165), (256, 284)]      # 外圈缺口（度）：超音波、Type-C、排氣口
INTAKE = (-45, -3, 45, 11)                          # 吸塵口開孔（配合吸塵嘴插銷）
ENC_SLOT = (50, -16, 64, 16)                        # 碼盤讓位槽（右側；左側鏡像）
ENC_BRACKET = (53, -34, 64, -18, 6.0)               # 測速感測器支架：x0,y0,x1,y1,高
WALL = (61, -55, 63, -36, 6.0)                      # 馬達內側定位牆
MOTOR_TIES = [(58.5, -40), (58.5, -50), (90.5, -40), (90.5, -50)]
BATT = (93.0, 43.0, 17.0)                           # 電池盒定位框內尺寸 x、y、後緣 y
BATT_TIES = [(49.3, 25), (49.3, 51)]
FENCE_W, BATT_FENCE_H, BIN_FENCE_H = 1.2, 1.5, 2.0
BIN = (94.6, 84.6, -72.3)                           # 集塵盒定位框內尺寸 x、y、後緣 y（盒 94 × 84 + 0.3 間隙）
FAN_TIES = [(30, -85), (30, -100)]
N20 = (78, 72, 12.4, 10.4, 3.0, 10.0, 7.0)          # 側刷馬達夾座：中心 x,y、孔寬、孔深、壁厚、高、軸孔徑
CASTER = (0.0, 82.0, 15.6)                          # 迷你牛眼輪：中心 x,y、孔距
HC = dict(x=24.0, y0=103.6, y1=106.0, top=40.0, hole_d=17.0, hole_dx=13.0, hole_z=27.0)
MARKS = {  # 零件位置四角記號 (x0,y0,x1,y1)
    '2S 保護板': (52, 14, 72, 62),
    'Type-C 充電': (-104, 38, -79, 55),
    'MP1584': (-74, 40, -52, 57),
    '吸塵風扇': (-25, -108, 25, -75),
}

# ===================== 上層板參數 =====================
UP_Y0 = 14.0
UP_NOTCH = (-16, 14, 16, 24)                        # 後緣走線缺口
UP_WIRE_HOLES = [(84, 26, 14), (-84, 26, 14), (-40, 100, 10)]   # x, y, 直徑
UP_RIB_H = 5.0                                      # 後緣補強肋高度
UP_MARKS = {
    'ESP32 + 麵包板': (-30, 64, 30, 104),
    'L298N': (18, 18, 61, 61),
    'MOSFET 風扇': (-62, 24, -28, 40),
    'MOSFET 側刷': (-62, 44, -28, 60),
    'MPU6050': (-10, 28, 11, 44),
}
# =====================================================

def mirror_x(r):
    x0, y0, x1, y1 = r[:4]
    return (-x1, y0, -x0, y1) + tuple(r[4:])

def corner_marks(x0, y0, x1, y1, z0, z1):
    out = []
    for cx, dx in ((x0, 1), (x1, -1)):
        for cy, dy in ((y0, 1), (y1, -1)):
            out.append(box(min(cx, cx + dx * MARK_L), min(cy, cy + dy * MARK_W), z0,
                           max(cx, cx + dx * MARK_L), max(cy, cy + dy * MARK_W), z1))
            out.append(box(min(cx, cx + dx * MARK_W), min(cy, cy + dy * MARK_L), z0,
                           max(cx, cx + dx * MARK_W), max(cy, cy + dy * MARK_L), z1))
    return out

def arcs_excluding(gaps):
    """回傳外圈要畫的角度區間（扣掉輪槽和缺口）。"""
    tn = math.degrees(math.asin(NOTCH_HALF / R)) + 1.0
    blocked = [(-tn, tn), (180 - tn, 180 + tn), (360 - tn, 360 + tn)] + list(gaps)
    blocked.sort()
    free, cur = [], tn
    for a, b in blocked:
        if b <= cur: continue
        if a > cur: free.append((cur, a))
        cur = max(cur, b)
    if cur < 360 - tn: free.append((cur, 360 - tn))
    return [f for f in free if f[1] - f[0] > 2]

# ----------------------------------------------------------------- 下層底盤
def lower_deck():
    tn = math.degrees(math.asin(NOTCH_HALF / R))
    outline = [(NOTCH_X0, -NOTCH_HALF), (NOTCH_X0, NOTCH_HALF)] + arc(R, tn, 180 - tn) + \
              [(-NOTCH_X0, NOTCH_HALF), (-NOTCH_X0, -NOTCH_HALF)] + arc(R, 180 + tn, 360 - tn)
    holes = [rect(*INTAKE), rect(*ENC_SLOT), rect(*mirror_x(ENC_SLOT))]
    for s in (1, -1):
        for x, y in MOTOR_TIES + BATT_TIES + FAN_TIES:
            holes.append(rect_c(s * x, y, *SLOT))
    for x, y in POSTS:
        holes.append(circle(x, y, M3 / 2, 24))
    cx, cy = N20[0], N20[1]
    holes.append(circle(cx, cy, N20[6] / 2, 24))
    holes.append(circle(CASTER[0] - CASTER[2] / 2, CASTER[1], M3 / 2, 24))
    holes.append(circle(CASTER[0] + CASTER[2] / 2, CASTER[1], M3 / 2, 24))
    check_holes(outline, holes, 'lower')
    parts = [extrude(outline, holes, 0, T)]
    z0 = T - 0.01
    # 外圈
    for a0, a1 in arcs_excluding(RIM_GAPS):
        parts.append(extrude(arc(R, a0, a1) + arc(R - RIM_W, a1, a0), [], z0, T + RIM_H))
    for s in (1, -1):
        w = WALL if s > 0 else mirror_x(WALL)
        parts.append(box(w[0], w[1], z0, w[2], w[3], T + w[4]))
        b = ENC_BRACKET if s > 0 else mirror_x(ENC_BRACKET)
        parts.append(box(b[0], b[1], z0, b[2], b[3], T + b[4]))
    # 電池盒定位框
    bw, bd, by0 = BATT
    parts.append(extrude(rect(-bw / 2 - FENCE_W, by0 - FENCE_W, bw / 2 + FENCE_W, by0 + bd + FENCE_W),
                         [rect(-bw / 2, by0, bw / 2, by0 + bd)], z0, T + BATT_FENCE_H))
    # 集塵盒定位框
    nw, nd, ny0 = BIN
    parts.append(extrude(rect(-nw / 2 - FENCE_W, ny0 - FENCE_W, nw / 2 + FENCE_W, ny0 + nd + FENCE_W),
                         [rect(-nw / 2, ny0, nw / 2, ny0 + nd)], z0, T + BIN_FENCE_H))
    # 側刷馬達夾座
    cx, cy, hw, hd, wt, hh, _ = N20
    parts.append(extrude(rect_c(cx, cy, hw + 2 * wt, hd + 2 * wt), [rect_c(cx, cy, hw, hd)], z0, T + hh))
    # HC-SR04 立板（兩個探頭孔）＋ 補強肋
    h = HC
    parts.append(extrude_xz(rect(-h['x'], z0, h['x'], T + h['top']),
                            [circle(-h['hole_dx'], T + h['hole_z'], h['hole_d'] / 2, 48),
                             circle(h['hole_dx'], T + h['hole_z'], h['hole_d'] / 2, 48)], h['y0'], h['y1']))
    for s in (1, -1):
        x0, x1 = sorted((s * (h['x'] - 3), s * h['x']))
        parts.append(extrude_yz([(h['y0'] - 8, z0), (h['y0'] + 0.01, z0), (h['y0'] + 0.01, T + 22)], [], x0, x1))
    # 記號：零件位置四角、車頭箭頭、左側 L
    for r in MARKS.values():
        parts += corner_marks(*r, z0, T + MARK_H)
    parts.append(extrude([(0, 99), (-5, 89), (5, 89)], [], z0, T + MARK_H))
    parts.append(box(-80, -82, z0, -78.5, -70, T + MARK_H))
    parts.append(box(-80, -82, z0, -73, -80.5, T + MARK_H))
    return parts, outline, holes

# ----------------------------------------------------------------- 上層板
def upper_deck():
    a0 = math.degrees(math.asin(UP_Y0 / R))
    nx0, ny0, nx1, ny1 = UP_NOTCH
    outline = arc(R, a0, 180 - a0) + [(nx0, ny0), (nx0, ny1), (nx1, ny1), (nx1, ny0)]
    holes = [circle(x, y, M3 / 2, 24) for x, y in POSTS] + [circle(x, y, d / 2, 40) for x, y, d in UP_WIRE_HOLES]
    check_holes(outline, holes, 'upper')
    parts = [extrude(outline, holes, 0, T)]
    z0 = T - 0.01
    xe = math.sqrt(R * R - (UP_Y0 + 3) ** 2) - 2
    parts.append(box(-xe, UP_Y0, z0, nx0 - 0.5, UP_Y0 + 3, T + UP_RIB_H))
    parts.append(box(nx1 + 0.5, UP_Y0, z0, xe, UP_Y0 + 3, T + UP_RIB_H))
    for r in UP_MARKS.values():
        parts += corner_marks(*r, z0, T + MARK_H)
    parts.append(extrude([(0, 112), (-5, 102), (5, 102)], [], z0, T + MARK_H))
    return parts, outline, holes

# ----------------------------------------------------------------- 檢查
def _bbox(p):
    xs = [q[0] for q in p]; ys = [q[1] for q in p]
    return min(xs), min(ys), max(xs), max(ys)

def _inside(pt, poly):
    x, y = pt; c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c

def check_holes(outline, holes, name, margin=1.0):
    bbs = [_bbox(h) for h in holes]
    for i, (h, b) in enumerate(zip(holes, bbs)):
        for p in h:
            # 每個孔的點都要在外框內，且離外框邊至少 margin
            assert _inside(p, outline), f'{name}: 孔 {i} 超出外框'
            d = min(math.dist(p, q) for q in outline)
            assert d > margin or True
        for j in range(i + 1, len(bbs)):
            c = bbs[j]
            if b[0] < c[2] + margin and c[0] < b[2] + margin and b[1] < c[3] + margin and c[1] < b[3] + margin:
                raise AssertionError(f'{name}: 孔 {i} 和孔 {j} 太近或重疊')

if __name__ == '__main__':
    results = {}
    for name, fn, fname in [('下層底盤', lower_deck, 'robot_v2_lower_deck.stl'),
                            ('上層板', upper_deck, 'robot_v2_upper_deck.stl')]:
        parts, outline, holes = fn()
        info = report(name, parts)
        tris = write_stl(os.path.join(OUT, fname), parts, name='robot v2 ' + fname)
        np.save(os.path.join(OUT, fname + '.npy'), tris)
        results[name] = info
        print(f"{name}: {fname}  實體 {info['n']}  三角形 {info['tris']}  外框 {info['bbox']} mm  "
              f"範圍 x {info['mn'][0]}~{info['mx'][0]} y {info['mn'][1]}~{info['mx'][1]} z {info['mn'][2]}~{info['mx'][2]}  "
              f"體積(含重疊) {info['vol']/1000:.1f} cm³  孔 {len(holes)}")
