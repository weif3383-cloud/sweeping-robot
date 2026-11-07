"""v2 下層組裝圖：俯視（上面）＋仰視（底面）＋組裝順序。"""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, FancyArrowPatch
from matplotlib.collections import PolyCollection
from matplotlib import font_manager
import logging
logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
font_manager.fontManager.addfont('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')
plt.rcParams['font.family'] = 'WenQuanYi Zen Hei'
plt.rcParams['axes.unicode_minus'] = False
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import v2_parts as V

C = dict(drive='#2C5AA0', sense='#C46A1C', power='#3C7A2B', clean='#8E44AD', ctrl='#C0392B', struct='#555555', later='#9AA0A6')

def plate_top(ax):
    T = np.load(os.path.join(HERE, 'robot_v2_lower_deck.stl.npy'))
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    up = n[:, 2] > 1e-6 * np.linalg.norm(n, axis=1)
    top = T[up]; z = top[:, :, 2].mean(1); o = np.argsort(z); top, z = top[o], z[o]
    col = ['#F1E3B4' if v < 3.5 else ('#D9BE6E' if v < 5 else '#B89A45') for v in z]
    ax.add_collection(PolyCollection(top[:, :, :2], facecolors=col, edgecolors='face', linewidths=0.2, zorder=1))

def part(ax, x0, y0, x1, y1, cat, num=None, label=None, later=False, alpha=0.30, lab_xy=None, fs=8):
    c = C['later'] if later else C[cat]
    ax.add_patch(Rectangle((min(x0, x1), y0), abs(x1 - x0), y1 - y0, fc=c, alpha=0.12 if later else alpha,
                           ec=c, lw=1.3, ls='--' if later else '-', zorder=3))
    cx, cy = lab_xy if lab_xy else ((x0 + x1) / 2, (y0 + y1) / 2)
    if label:
        ax.text(cx, cy, label, ha='center', va='center', fontsize=fs, color=c, zorder=6)
    if num is not None:
        badge(ax, x0, y1, num, c)

def badge(ax, x, y, num, c):
    ax.add_patch(Circle((x, y), 5.2, fc=c, ec='white', lw=1.2, zorder=8))
    ax.text(x, y - 0.3, str(num), ha='center', va='center', fontsize=8.5, color='white', fontweight='bold', zorder=9)

def tie(ax, x0, x1, y, later=False):
    ax.plot([x0, x1], [y, y], color=C['later'] if later else '#333', lw=1.6, ls=(0, (1, 1)), zorder=5)

fig = plt.figure(figsize=(17, 11.5), dpi=110)
fig.suptitle('掃地機器人 v2　下層組裝圖', fontsize=20, color='#1F6F78', x=0.02, ha='left', y=0.975)
fig.text(0.02, 0.935, '實線＝現在要裝的零件　虛線灰色＝之後才裝（集塵盒、風扇、吸塵嘴、側刷）　點線＝束帶　單位 mm',
         fontsize=10.5, color='#555')

# ---------------- 俯視 ----------------
ax = fig.add_axes([0.0, 0.0, 0.60, 0.88])
plate_top(ax)
ax.set_title('上面（俯視，車頭朝上）', fontsize=13, color='#1F6F78')
for s, side in ((1, '右'), (-1, '左')):
    part(ax, s * 89, -32.5, s * 115, 32.5, 'drive', 3 if s < 0 else None, side + '輪', fs=9)
    part(ax, s * 64, -25, s * 87, 12, 'drive', 2 if s < 0 else None, '齒輪箱', fs=7.5)
    part(ax, s * 66, -55, s * 86, -25, 'drive', None, side + ' TT\n馬達', fs=8, lab_xy=(s * 76, -33))
    ax.add_patch(Rectangle((s * 58.5 - 0.8, -12), 1.6, 24, fc=C['sense'], ec=C['sense'], zorder=4))
    part(ax, s * 55, -27, s * 62, -9, 'sense', 4 if s < 0 else None, None)
    tie(ax, s * 58.5, s * 90.5, -40); tie(ax, s * 58.5, s * 90.5, -50)
ax.annotate('碼盤＋HC-020K\n（支架上，缺口夾住碼盤後緣）', (-58.5, -18), (-40, -14), fontsize=8, color=C['sense'],
            ha='left', arrowprops=dict(arrowstyle='-', color=C['sense'], lw=0.8), zorder=7)
part(ax, -46.5, 17, 46.5, 59, 'power', 5, '18650 電池盒\n（放進定位框，束帶 ×2）', fs=9)
tie(ax, -49.3, 49.3, 25); tie(ax, -49.3, 49.3, 51)
part(ax, 52, 14, 72, 62, 'power', 6, '2S\n保護板', fs=8)
part(ax, -104, 38, -79, 55, 'power', 7, 'Type-C\n充電', fs=7.5)
ax.add_patch(FancyArrowPatch((-104, 46.5), (-119, 46.5), arrowstyle='-|>', mutation_scale=12, color=C['power'], lw=1.5, zorder=7))
ax.text(-113, 39, '插孔\n朝外', fontsize=7.5, color=C['power'], ha='center')
part(ax, -74, 40, -52, 57, 'power', 8, 'MP1584', fs=7.5)
part(ax, 72, 67, 84, 77, 'clean', 9, None)
ax.text(62, 84, 'N20 側刷馬達', fontsize=7.5, color=C['clean'], ha='right')
part(ax, -22.5, 100.5, 22.5, 102, 'sense', 10, None)
for sx in (-13, 13):
    part(ax, sx - 8, 102, sx + 8, 114.5, 'sense', None, None)
ax.text(0, 93, 'HC-SR04（板子在立板後面，探頭穿過圓孔朝前）', fontsize=8, color=C['sense'], ha='center')
for i, (x, y) in enumerate(V.POSTS):
    ax.add_patch(Circle((x, y), 3.2, fc=C['struct'], ec='white', lw=0.8, zorder=6))
badge(ax, 92, 76, 11, C['struct'])
ax.text(100, 80, 'M3 銅柱', fontsize=7.5, color=C['struct'])
part(ax, -47, -72, 47, 12, 'clean', None, '集塵盒（之後）\n吸塵口在盒子前端底下', later=True, fs=9)
part(ax, -25, -108, 25, -75, 'clean', None, '吸塵風扇（之後）', later=True, fs=8)
tie(ax, -30, 30, -85, later=True); tie(ax, -30, 30, -100, later=True)
ax.axhline(0, color='#C0392B', lw=0.7, ls='--', zorder=2)
ax.text(-125, 1.5, '輪軸線', fontsize=8, color='#C0392B')
ax.annotate('', xy=(0, 131), xytext=(0, 121), arrowprops=dict(arrowstyle='-|>', color='#1F6F78', lw=1.6))
ax.text(4, 125, '車頭', fontsize=10, color='#1F6F78')
ax.set_xlim(-132, 132); ax.set_ylim(-128, 135); ax.set_aspect('equal'); ax.axis('off')

# ---------------- 仰視 ----------------
ab = fig.add_axes([0.60, 0.50, 0.40, 0.42])
ab.set_title('底面（從底下往上看，左右相反）', fontsize=12, color='#1F6F78')
lo, outline, holes = V.lower_deck()
mx = lambda pts: [(-x, y) for x, y in pts]
ab.add_patch(Polygon(mx(outline), closed=True, fc='#F1E3B4', ec='#8a7a40', lw=1.0, zorder=1))
for h in holes:
    ab.add_patch(Polygon(mx(h), closed=True, fc='white', ec='#8a7a40', lw=0.6, zorder=2))
for s in (1, -1):
    ab.add_patch(Rectangle((s * 89 if s > 0 else -115, -32.5), 26, 65, fc=C['drive'], alpha=0.25, ec=C['drive'], zorder=3))
ab.text(-102, 40, '右輪', fontsize=8, color=C['drive'], ha='center'); ab.text(102, 40, '左輪', fontsize=8, color=C['drive'], ha='center')
ab.add_patch(Rectangle((-13, 72), 26, 20, fc=C['struct'], alpha=0.35, ec=C['struct'], zorder=4))
ab.add_patch(Circle((0, 82), 6, fc='#888', ec='#444', zorder=5))
badge(ab, 18, 92, 1, C['struct'])
ab.text(0, 100, '迷你牛眼輪（M3 螺絲 ×2）', fontsize=8, color=C['struct'], ha='center')
ab.add_patch(Rectangle((-48, -6), 96, 20, fc=C['later'], alpha=0.15, ec=C['later'], ls='--', zorder=3))
ab.text(0, -16, '吸塵嘴（之後，黏在吸塵口下方）', fontsize=8, color=C['later'], ha='center')
ab.add_patch(Circle((-78, 72), 45, fc='none', ec=C['later'], ls='--', lw=1.0, zorder=3))
ab.add_patch(Circle((-78, 72), 10, fc=C['later'], alpha=0.25, ec=C['later'], zorder=3))
ab.text(-78, 122, '側刷（之後）', fontsize=8, color=C['later'], ha='center')
ab.annotate('', xy=(0, 131), xytext=(0, 121), arrowprops=dict(arrowstyle='-|>', color='#1F6F78', lw=1.4))
ab.text(4, 125, '車頭', fontsize=9, color='#1F6F78')
ab.set_xlim(-132, 132); ab.set_ylim(-125, 135); ab.set_aspect('equal'); ab.axis('off')

# ---------------- 組裝順序 ----------------
at = fig.add_axes([0.61, 0.02, 0.38, 0.46]); at.axis('off')
steps = [
    (1, 'struct', '迷你牛眼輪', '裝在底面，螺絲由上往下穿，螺帽鎖在底下'),
    (2, 'drive', '左右 TT 馬達', '內側貼定位牆、出軸對準讓位槽中心；每邊 2 條束帶綁住馬達尾段'),
    (3, 'drive', '輪子', '壓進外側出軸，轉轉看不會刮到輪槽'),
    (4, 'sense', '碼盤＋HC-020K', '碼盤套內側出軸；感測器固定在支架上，缺口夾住碼盤後緣'),
    (5, 'power', '電池盒', '放進定位框，2 條束帶橫跨綁住'),
    (6, 'power', '2S 保護板', '雙面膠貼在記號內'),
    (7, 'power', 'Type-C 充電模組', '插孔朝左外側，對準外圈缺口'),
    (8, 'power', 'MP1584', '先用電表調到 5.0 V 再貼上'),
    (9, 'clean', 'N20 側刷馬達', '塞進夾座，出軸朝下穿過圓孔'),
    (10, 'sense', 'HC-SR04', '板子在立板後面，兩個探頭穿過圓孔'),
    (11, 'struct', 'M3 銅柱 ×4', '公螺紋朝下穿孔，底下鎖螺帽（上層之後再裝）'),
]
at.text(0, 1.0, '組裝順序', fontsize=13, color='#1F6F78', va='top', transform=at.transAxes)
for i, (num, cat, name, how) in enumerate(steps):
    y = 0.92 - i * 0.083
    at.add_patch(Circle((0.025, y), 0.018, fc=C[cat], ec='white', transform=at.transAxes, zorder=3))
    at.text(0.025, y - 0.003, str(num), fontsize=8.5, color='white', ha='center', va='center', fontweight='bold', transform=at.transAxes, zorder=4)
    at.text(0.06, y + 0.012, name, fontsize=10, color='#222', va='center', transform=at.transAxes)
    at.text(0.06, y - 0.025, how, fontsize=8.3, color='#555', va='center', transform=at.transAxes)

out = os.path.join(HERE, 'assembly_lower_v2.png')
plt.savefig(out, dpi=110)
print('saved', out)
