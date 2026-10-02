#!/usr/bin/env python3
"""M2：金属 / 镜面 / 抛光表面材质测试专用分析。

背景
----
S01（透明塑料瓶）与 S05（ArUco 标定板）已确立两种深度失效特征：

    不透明漫反射   深度完整（整框有效 87.4%），z 与横向都正确
    红外可穿透     中心空洞（整框 26.8%、中心 8.1%、r=0.30），
                   z 正确但横向刚性偏移 +17 px

M2 要回答的是：**金属 / 镜面是否构成第三种失效模式（反射而非穿透）？**

与 run_material_experiment.py 的关系
------------------------------------
* V0~V4p 六种方法**直接 import 复用**，保证与 S01/S05 的数字逐位可比；
* 在其之上新增金属专用诊断（原脚本没有的）：

    1. 解除「有效像素 < 12 就丢弃整帧」的限制 ——
       镜面最可能的形态是**零回波**，原脚本会把这最强的证据直接藏掉；
    2. 嵌套 ROI 有效占比剖面 v(1.0/0.8/0.6/0.4/0.2) ——
       区分「中心空洞（穿透）」与「整体缺失（镜面）」；
    3. 逐列有效占比剖面（8 列）——
       抛光圆柱的高光带会让**某几列**整条丢失，而不是中心丢失；
    4. 横向偏移 u_offset_px —— 复现 S05 的 +17 px 判据；
    5. 自动失效模式判定，并检验设计草案的两分类规则会不会**误判**。

失效模式判定（阈值都取得了很宽的余量）
--------------------------------------
    no_return    整框有效占比 < 5%           深度基本没有回波
    reflection   V4 的 z 偏离真值 > 0.10 m   z 本身错了（虚拟像 / 错配）
                 或「打在物体上」的回波 < 50%  有回波，但回波不是这个物体
    penetration  z 正确但有效占比 < 60%       中心空洞、横向不可信
    diffuse      z 正确且有效占比 >= 60%      正常漫反射

    参考量级：S01/S05 正常时的 z 噪声是 2~10 mm，而均值被背景拉偏是 1.47 m，
    所以 0.10 m 的门限两边都有 1 个数量级余量。

    两条 GT 参照指标是判「反射」的关键（M2 新增）：
        gt_hit_of_valid  有效回波里，落在真值 ±5 cm 内的比例
                         穿透物体应接近 1（回波确实来自物体表面，只是稀疏）
                         镜面若拿到的是虚拟像，该比例会显著下降
        behind_frac      有效回波里，比真值远 10 cm 以上的比例
                         镜面把相机身后的东西成虚像时，这个值会抬头

用法
----
    source /opt/ros/humble/setup.bash
    source ~/VIO/semantic_ws/install/setup.bash

    python3 ~/VIO/tools/analyze_metal_m2.py \
        ~/VIO/data/bags/_metal150 \
        --class-name bottle --z-gt 1.5 \
        --label "金属保温杯_1.5m_远背景" \
        --out ~/VIO/data/material_exp/metal_150 \
        --summary ~/VIO/data/material_exp/summary.csv
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sqlite3
import statistics
import sys
from collections import Counter, deque

import numpy as np

import matplotlib
matplotlib.use("Agg")  # 无显示环境（ssh / 后台）也能出图
import matplotlib.pyplot as plt
from matplotlib import font_manager

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 复用 S01/S05 的方法实现 —— 保证 V0~V4p 的数字与已有实验逐位可比
from run_material_experiment import (  # noqa: E402
    METHODS, MIN_DEPTH, MAX_DEPTH, DEPTH_TOL, MIN_VALID,
    box_bounds, valid_mask, eval_method, stamp_ns,
)

# ---------------------------------------------------------------- 常量

COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
OBJ_TOPIC = "/semantics/objects"

NESTED_RATIOS = (1.0, 0.8, 0.6, 0.4, 0.2)
N_COL_BINS = 8

NO_RETURN_V = 0.05   # 整框有效占比低于此值 → 视为没有回波
Z_ERR_TOL = 0.10     # z 误差门限 [m]
PARTIAL_V = 0.60     # 有效占比低于此值 → 视为部分缺失（穿透候选）
GT_TOL = 0.05        # 与卷尺真值差 5 cm 以内 → 认为打在该物体上
BEYOND_TOL = 0.10    # 比真值远 10 cm 以上 → 怀疑反射 / 虚拟像
GT_HIT_MIN = 0.40    # 「打在物体上」的比例低于此值 → 回波不是这个物体
                     # 已知良性值：S01 透明瓶 70.3%，S05 标定板 ≈96%
                     # 取 0.40 对已知良性样本留了近 2 倍余量

MODES = ("diffuse", "penetration", "reflection", "no_return")
MODE_CN = {
    "diffuse": "漫反射正常",
    "penetration": "红外穿透",
    "reflection": "镜面反射",
    "no_return": "无回波",
}
BRANCH_CN = {"opaque": "opaque(V4)", "transparent": "transparent(z+彩色框)",
             "unknown": "unknown(保守)", "undefined": "undefined(r 不可算)"}

# S01 / S05 参考基线，用于图与报告里的横向对照
REF_TRANSPARENT = dict(name="S01 透明瓶", v_box=0.268, v_roi=0.0811, r=0.302, u_off=17.0)
REF_OPAQUE = dict(name="S05 标定板", v_box=0.874, v_roi=0.973, r=1.11, u_off=2.8)

FRAME_CACHE_MAX = 400  # 最多缓存这么多帧用于出图，避免内存爆掉


# ---------------------------------------------------------------- 工具

def _extract_sc_face():
    """从 Noto Sans CJK 的 .ttc 里抽出简体中文（SC）那一面，缓存成独立 .otf。

    为什么需要这一步：matplotlib 的 addfont() 对 .ttc 只会注册第 0 面，
    而 Noto Sans CJK 的第 0 面是 **JP** —— 中文能显示，但部分汉字是日文字形。
    论文插图想要正确的简体字形，就得把第 2 面（SC）单独抽出来。
    失败就返回 None，脚本会退回用 .ttc 的 JP 面（能显示，只是字形略有差异）。
    """
    cache = os.path.join(_HERE, "fonts", "NotoSansCJKsc-Regular.otf")
    if os.path.exists(cache):
        return cache
    src = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    if not os.path.exists(src):
        return None
    try:
        from fontTools.ttLib import TTCollection
        tc = TTCollection(src, lazy=True)
        for f in tc.fonts:
            try:
                full = (f["name"].getDebugName(4) or "").strip()
            except Exception:
                continue
            if full == "Noto Sans CJK SC":   # 精确匹配，排除 Mono
                os.makedirs(os.path.dirname(cache), exist_ok=True)
                f.save(cache)
                return cache
    except Exception:
        return None
    return None


def setup_cjk_font():
    """让 matplotlib 能画中文，返回实际用上的字体名。"""
    plt.rcParams["axes.unicode_minus"] = False
    cands = []
    sc = _extract_sc_face()
    if sc:
        cands.append(sc)                       # 首选：正确的简体字形
    for p in ("/usr/share/fonts/**/*CJK*.ttc", "/usr/share/fonts/**/*CJK*.otf"):
        cands += sorted(glob.glob(p, recursive=True))
    for path in cands:
        try:
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = "sans-serif"
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            return name
        except Exception:
            continue
    return None


def fmean(vals):
    vals = [float(v) for v in vals if v is not None and v == v]
    return statistics.fmean(vals) if vals else float("nan")


def fstd(vals):
    vals = [float(v) for v in vals if v is not None and v == v]
    return statistics.pstdev(vals) if len(vals) > 1 else (0.0 if vals else float("nan"))


def signed(vals):
    """带符号的均值（用于判断 z 是偏近还是偏远）。"""
    return fmean(vals)


def fmt(v, nd=3, suffix=""):
    return "nan" if v != v else f"{v:.{nd}f}{suffix}"


def jn(vals, nd=4):
    return ";".join("nan" if v != v else f"{v:.{nd}g}" for v in vals)


def unj(s):
    out = []
    for t in str(s).split(";"):
        try:
            out.append(float(t))
        except ValueError:
            out.append(float("nan"))
    return out


# ---------------------------------------------------------------- 空间诊断

def nested_validity_profile(depth, bbox):
    """嵌套中心 ROI 的有效深度占比：{1.0, 0.8, 0.6, 0.4, 0.2} → 占比。"""
    h, w = depth.shape[:2]
    out = {}
    for r in NESTED_RATIOS:
        x0, y0, x1, y1 = box_bounds(bbox, r, w, h)
        sub = depth[y0:y1, x0:x1]
        out[r] = float(valid_mask(sub).mean()) if sub.size >= 4 else float("nan")
    return out


def column_validity_profile(depth, bbox, nbins=N_COL_BINS):
    """把框按列等分，给出每列的有效占比 —— 用来发现镜面高光带。"""
    h, w = depth.shape[:2]
    x0, y0, x1, y1 = box_bounds(bbox, 1.0, w, h)
    patch = depth[y0:y1, x0:x1]
    if patch.size < nbins * 4:
        return [float("nan")] * nbins
    edges = np.linspace(0, patch.shape[1], nbins + 1).astype(int)
    prof = []
    for i in range(nbins):
        sub = patch[:, edges[i]:edges[i + 1]]
        prof.append(float(valid_mask(sub).mean()) if sub.size else float("nan"))
    return prof


def material_features_m2(depth, bbox, z_gt):
    """M2 版材质特征。

    与 run_material_experiment.material_features 的关键差别：
    **零回波也照样记录**（原版 valid_px < 12 时返回 None，会把帧丢掉）。
    只有框小到无法计算时才返回 None。
    """
    h, w = depth.shape[:2]
    bx0, by0, bx1, by1 = box_bounds(bbox, 1.0, w, h)
    if bx1 - bx0 < 2 or by1 - by0 < 2:
        return None

    patch = depth[by0:by1, bx0:bx1]
    pv = valid_mask(patch)
    n_valid = int(pv.sum())
    n_box = int(patch.size)

    rx0, ry0, rx1, ry1 = box_bounds(bbox, 0.6, w, h)
    roi = depth[ry0:ry1, rx0:rx1]
    n_roi_valid = int(valid_mask(roi).sum())

    prof = nested_validity_profile(depth, bbox)
    cols = column_validity_profile(depth, bbox)

    v_box = n_valid / n_box
    v_roi = n_roi_valid / max(1, roi.size)

    f = {
        "box_px": n_box, "valid_px": n_valid, "valid_ratio": v_box,
        "roi_px": int(roi.size), "roi_valid_px": n_roi_valid, "roi_valid_ratio": v_roi,
        "ratio_r": (v_roi / v_box) if v_box > 1e-9 else float("nan"),
        "v_full": prof[1.0], "v_080": prof[0.8], "v_060": prof[0.6],
        "v_040": prof[0.4], "v_020": prof[0.2],
        "nested_profile": jn([prof[r] for r in NESTED_RATIOS]),
        "col_profile": jn(cols),
    }

    if n_valid >= 1:
        vals = patch[pv]
        med = float(np.median(vals))
        p20 = float(np.percentile(vals, 20.0))
        fg = pv & (np.abs(patch - med) <= DEPTH_TOL)
        n_fg = int(fg.sum())
        in_roi = np.zeros(patch.shape, dtype=bool)
        cx0 = max(0, rx0 - bx0); cx1 = min(patch.shape[1], rx1 - bx0)
        cy0 = max(0, ry0 - by0); cy1 = min(patch.shape[0], ry1 - by0)
        in_roi[cy0:cy1, cx0:cx1] = True

        # --- 以卷尺真值为参照的三项指标（M2 判「反射」的关键）
        near = np.abs(vals - z_gt) <= GT_TOL
        n_near = int(near.sum())
        n_behind = int((vals > z_gt + BEYOND_TOL).sum())
        n_front = int((vals < z_gt - BEYOND_TOL).sum())

        f.update(
            z_median=med, z_p20=p20,
            z_min=float(vals.min()), z_max=float(vals.max()),
            z_iqr=float(np.percentile(vals, 75) - np.percentile(vals, 25)),
            fg_px=n_fg, fg_frac=n_fg / max(1, n_valid),
            fg_in_roi_px=int((fg & in_roi).sum()),
            fg_in_roi_frac=int((fg & in_roi).sum()) / max(1, n_fg),
            gt_hit_px=n_near,
            gt_hit_frac=n_near / n_box,
            gt_hit_of_valid=n_near / n_valid,
            behind_frac=n_behind / n_valid,
            front_frac=n_front / n_valid,
            z_mean=float(vals.mean()),
        )
    else:
        f.update(z_median=float("nan"), z_p20=float("nan"),
                 z_min=float("nan"), z_max=float("nan"), z_iqr=float("nan"),
                 fg_px=0, fg_frac=float("nan"),
                 fg_in_roi_px=0, fg_in_roi_frac=float("nan"),
                 gt_hit_px=0, gt_hit_frac=0.0, gt_hit_of_valid=float("nan"),
                 behind_frac=float("nan"), front_frac=float("nan"),
                 z_mean=float("nan"))
    return f


# ---------------------------------------------------------------- 判定

def classify_mode(f, z_est, z_gt):
    """失效模式判定，见模块 docstring 的阈值表。"""
    v_box = f["valid_ratio"]
    if not (v_box == v_box) or v_box < NO_RETURN_V or z_est is None:
        return "no_return"
    if abs(z_est - z_gt) > Z_ERR_TOL:
        return "reflection"
    gh = f.get("gt_hit_of_valid", float("nan"))
    if gh == gh and gh < GT_HIT_MIN:
        # 有回波，但回波基本不在物体上 —— 典型镜面：拿到的是别处的像
        return "reflection"
    if v_box < PARTIAL_V:
        return "penetration"
    return "diffuse"


def design_branch(r):
    """设计草案 2.2/2.3 节的两分类规则。"""
    if not (r == r):
        return "undefined"
    if r > 1.0:
        return "opaque"
    if r < 0.7:
        return "transparent"
    return "unknown"


def branch_verdict(mode, branch):
    """两分类规则导出的处理分支，对真实失效模式是否安全。

    ok       分支与真实模式匹配，策略正确
    degraded 落到 unknown 的保守策略：不会注入错误约束，但也没有收益
    fail     分支与真实模式不符 —— 会注入**错误**的几何约束
    """
    if mode == "diffuse":
        if branch == "opaque":
            return "ok"
        return "degraded" if branch == "unknown" else "fail"
    if mode == "penetration":
        if branch == "transparent":
            return "ok"
        return "degraded" if branch == "unknown" else "fail"
    # reflection / no_return：两个分支都不可用
    #   opaque 分支信 V4 的 z —— 错
    #   transparent 分支信深度的 z —— 也错
    return "fail"


# ---------------------------------------------------------------- 读包

def iter_bag(db, class_name, max_det):
    """遍历 bag，产出 (颜色帧, 深度帧, bbox, 内参) 的检测记录。"""
    conn = sqlite3.connect(db)
    tmap = {int(i): (str(n), str(t)) for i, n, t in
            conn.execute("SELECT id,name,type FROM topics")}
    bridge = CvBridge()
    dh = deque(maxlen=8)
    ch = deque(maxlen=8)
    info = None
    n_det = 0

    for tid, _ts, data in conn.execute(
            "SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp"):
        e = tmap.get(int(tid))
        if e is None:
            continue
        tp, ts = e
        m = deserialize_message(bytes(data), get_message(ts))

        if tp.endswith("camera_info"):
            info = m
            continue
        if tp == DEPTH_TOPIC:
            raw = bridge.imgmsg_to_cv2(m, "passthrough")
            scale = 0.001 if m.encoding == "16UC1" else 1.0
            dh.append((stamp_ns(m), raw.astype(np.float32) * scale))
            continue
        if tp == COLOR_TOPIC:
            ch.append((stamp_ns(m), bridge.imgmsg_to_cv2(m, "bgr8")))
            continue
        if tp != OBJ_TOPIC or info is None or not dh:
            continue

        objs = [o for o in m.objects if o.class_name == class_name]
        if not objs:
            continue

        ot = stamp_ns(m)
        _, depth = min(dh, key=lambda x: abs(x[0] - ot))
        color = None
        if ch:
            _, color = min(ch, key=lambda x: abs(x[0] - ot))

        k = np.asarray(info.k, dtype=np.float64)
        intr = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

        for o in objs:
            if max_det and n_det >= max_det:
                conn.close()
                return
            bbox = tuple(float(v) for v in o.bbox)
            score = float(getattr(o, "score", getattr(o, "confidence", float("nan"))))
            yield dict(depth=depth, color=color, bbox=bbox, intr=intr,
                       score=score, det_index=n_det)
            n_det += 1

    conn.close()


# ---------------------------------------------------------------- 主流程

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--class-name", required=True)
    ap.add_argument("--z-gt", type=float, required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--summary", default="")
    ap.add_argument("--max-det", type=int, default=0)
    args = ap.parse_args()

    font = setup_cjk_font()
    os.makedirs(os.path.expanduser(args.out), exist_ok=True)
    label = args.label or f"{args.class_name}_{args.z_gt}m"

    db = os.path.expanduser(args.bag)
    if os.path.isdir(db):
        c = [f for f in sorted(os.listdir(db)) if f.endswith(".db3")]
        if not c:
            raise SystemExit(f"{db} 里没有 .db3，若是 zstd 压缩请先 zstd -d 解压")
        db = os.path.join(db, c[0])

    feats, rows, frames = [], [], []
    n_det = 0

    for rec in iter_bag(db, args.class_name, args.max_det):
        depth, bbox = rec["depth"], rec["bbox"]
        fx, fy, cx, cy = rec["intr"]
        f = material_features_m2(depth, bbox, args.z_gt)
        if f is None:
            continue
        n_det += 1
        f = dict(f)
        f.update(label=label, class_name=args.class_name, z_gt=args.z_gt,
                 det_index=rec["det_index"], score=rec["score"],
                 bbox_x0=bbox[0], bbox_y0=bbox[1], bbox_x1=bbox[2], bbox_y1=bbox[3],
                 bbox_w=bbox[2] - bbox[0], bbox_h=bbox[3] - bbox[1],
                 bbox_cx=(bbox[0] + bbox[2]) * 0.5,
                 bbox_cy=(bbox[1] + bbox[3]) * 0.5)
        feats.append(f)

        z_by_method = {}
        u_by_method = {}
        for meth in METHODS:
            r = eval_method(depth, bbox, fx, fy, cx, cy, meth)
            row = dict(label=label, class_name=args.class_name, z_gt=args.z_gt,
                       det_index=rec["det_index"], method=meth,
                       valid=int(r is not None))
            if r:
                z = float(r["pos"][2])
                z_by_method[meth] = z
                u_by_method[meth] = float(r["u_mean"])
                row.update(z_est=z, x_est=float(r["pos"][0]), y_est=float(r["pos"][1]),
                           abs_err=abs(z - args.z_gt), bias=z - args.z_gt,
                           points=r["points"], z0=r["z0"], u_mean=r["u_mean"],
                           u_offset_px=r["u_mean"] - f["bbox_cx"])
            else:
                row.update(z_est="", x_est="", y_est="", abs_err="", bias="",
                           points="", z0="", u_mean="", u_offset_px="")
            rows.append(row)

        z_v4 = z_by_method.get("V4_roi_p20_cluster")
        u_v4 = u_by_method.get("V4_roi_p20_cluster")
        u_off = (u_v4 - f["bbox_cx"]) if u_v4 is not None else float("nan")
        z_ref = z_v4 if z_v4 is not None else f["z_median"]
        u_off_mm = (u_off * z_ref / fx * 1000.0) if (u_off == u_off and z_ref == z_ref) \
            else float("nan")
        mode = classify_mode(f, z_v4 if z_v4 is not None else f["z_median"], args.z_gt)
        branch = design_branch(f["ratio_r"])
        f.update(z_v4=z_v4 if z_v4 is not None else float("nan"),
                 u_offset_px=u_off, u_offset_mm=u_off_mm,
                 mode=mode, branch=branch, verdict=branch_verdict(mode, branch))
        if len(frames) < FRAME_CACHE_MAX:
            frames.append((rec["depth"], rec["color"], bbox, dict(f)))

    if not feats:
        raise SystemExit(
            f"没有匹配到 {args.class_name} 的检测。\n"
            f"检查：bag 里有 /semantics/objects 吗？类别名对不对？\n"
            f"提示：YOLO 可能把目标检成了别的类，先不加过滤跑一次看日志里实际的类别名。")

    # ---------------- 逐帧明细
    rf = os.path.join(os.path.expanduser(args.out), "per_frame.csv")
    keys = ["label", "class_name", "z_gt", "det_index", "method", "valid", "z_est",
            "x_est", "y_est", "abs_err", "bias", "points", "z0", "u_mean", "u_offset_px"]
    with open(rf, "w", newline="", encoding="utf-8") as fp:
        wr = csv.DictWriter(fp, fieldnames=keys, extrasaction="ignore")
        wr.writeheader(); wr.writerows(rows)

    # ---------------- 特征明细
    ff = os.path.join(os.path.expanduser(args.out), "features.csv")
    fkeys = ["label", "class_name", "z_gt", "det_index", "score",
             "box_px", "valid_px", "valid_ratio", "roi_px", "roi_valid_px",
             "roi_valid_ratio", "ratio_r",
             "v_full", "v_080", "v_060", "v_040", "v_020", "nested_profile",
             "col_profile", "fg_px", "fg_frac", "fg_in_roi_px", "fg_in_roi_frac",
             "z_median", "z_p20", "z_mean", "z_min", "z_max", "z_iqr",
             "gt_hit_px", "gt_hit_frac", "gt_hit_of_valid", "behind_frac", "front_frac",
             "z_v4", "u_offset_px", "u_offset_mm", "mode", "branch", "verdict",
             "bbox_x0", "bbox_y0", "bbox_x1", "bbox_y1",
             "bbox_w", "bbox_h", "bbox_cx", "bbox_cy"]
    with open(ff, "w", newline="", encoding="utf-8") as fp:
        wr = csv.DictWriter(fp, fieldnames=fkeys, extrasaction="ignore")
        wr.writeheader(); wr.writerows(feats)

    # ---------------- 汇总
    modes = Counter(f["mode"] for f in feats)
    branches = Counter(f["branch"] for f in feats)
    verdicts = Counter(f["verdict"] for f in feats)
    dom_mode = modes.most_common(1)[0][0]

    summ = {
        "label": label, "class_name": args.class_name, "z_gt": args.z_gt,
        "n_det": n_det, "n_det_valid": len(feats),
        "box_px": fmean([f["box_px"] for f in feats]),
        "valid_ratio": fmean([f["valid_ratio"] for f in feats]),
        "roi_valid_ratio": fmean([f["roi_valid_ratio"] for f in feats]),
        "ratio_r": fmean([f["ratio_r"] for f in feats]),
        "fg_frac": fmean([f["fg_frac"] for f in feats]),
        "fg_in_roi_frac": fmean([f["fg_in_roi_frac"] for f in feats]),
        "gt_hit_frac": fmean([f["gt_hit_frac"] for f in feats]),
        "gt_hit_of_valid": fmean([f["gt_hit_of_valid"] for f in feats]),
        "behind_frac": fmean([f["behind_frac"] for f in feats]),
        "front_frac": fmean([f["front_frac"] for f in feats]),
        "v_full": fmean([f["v_full"] for f in feats]),
        "v_080": fmean([f["v_080"] for f in feats]),
        "v_060": fmean([f["v_060"] for f in feats]),
        "v_040": fmean([f["v_040"] for f in feats]),
        "v_020": fmean([f["v_020"] for f in feats]),
        "zero_return_frac": fmean([1.0 if f["valid_ratio"] < NO_RETURN_V else 0.0
                                   for f in feats]),
        "mode_dominant": dom_mode,
        "mode_counts": ";".join(f"{m}:{modes.get(m, 0)}" for m in MODES),
        "branch_counts": ";".join(f"{b}:{branches.get(b, 0)}" for b in
                                  ("opaque", "transparent", "unknown", "undefined")),
        "verdict_counts": ";".join(f"{v}:{verdicts.get(v, 0)}"
                                   for v in ("ok", "degraded", "fail")),
        "z_v4_mae": fmean([abs(f["z_v4"] - args.z_gt) for f in feats
                           if f["z_v4"] == f["z_v4"]]),
        "z_v4_bias": signed([f["z_v4"] - args.z_gt for f in feats
                             if f["z_v4"] == f["z_v4"]]),
        "z_v4_std": fstd([f["z_v4"] for f in feats if f["z_v4"] == f["z_v4"]]),
        "u_offset_px": float("nan"),
        "u_offset_mm": float("nan"),
        "u_offset_std": float("nan"),
    }
    u_off = [f["u_offset_px"] for f in feats if f["u_offset_px"] == f["u_offset_px"]]
    u_off_mm = [f["u_offset_mm"] for f in feats if f["u_offset_mm"] == f["u_offset_mm"]]
    summ["u_offset_px"] = fmean(u_off)
    summ["u_offset_std"] = fstd(u_off)
    summ["u_offset_mm"] = fmean(u_off_mm)

    for meth in METHODS:
        tag = meth.split("_")[0]
        er = [float(r["abs_err"]) for r in rows if r["method"] == meth and r["valid"] == 1]
        bi = [float(r["bias"]) for r in rows if r["method"] == meth and r["valid"] == 1]
        zs = [float(r["z_est"]) for r in rows if r["method"] == meth and r["valid"] == 1]
        summ[f"{tag}_mae"] = fmean(er)
        summ[f"{tag}_bias"] = fmean(bi)
        summ[f"{tag}_std"] = fstd(zs)
        summ[f"{tag}_max"] = max(er) if er else float("nan")
        summ[f"{tag}_n"] = len(er)

    summ_path = (os.path.expanduser(args.summary) if args.summary
                 else os.path.join(os.path.expanduser(args.out), "summary.csv"))
    if args.summary:
        os.makedirs(os.path.dirname(summ_path), exist_ok=True)
        old = []
        if os.path.exists(summ_path):
            with open(summ_path, newline="", encoding="utf-8") as fp:
                old = [r for r in csv.DictReader(fp) if r.get("label") != label]
        with open(summ_path, "w", newline="", encoding="utf-8") as fp:
            wr = csv.DictWriter(fp, fieldnames=list(summ.keys()))
            wr.writeheader()
            for r in old:
                wr.writerow({k: r.get(k, "") for k in summ.keys()})
            wr.writerow(summ)
    else:
        with open(summ_path, "w", newline="", encoding="utf-8") as fp:
            wr = csv.DictWriter(fp, fieldnames=list(summ.keys()))
            wr.writeheader(); wr.writerow(summ)

    # ---------------- 出图
    fig_path = os.path.join(os.path.expanduser(args.out), f"fig_m2_{label}.png")
    make_figure(fig_path, label, args.z_gt, feats, frames, summ)

    # ---------------- 打印报告
    zgt = args.z_gt
    print("=" * 82)
    print(f"{label}    类别={args.class_name}    真值={zgt} m    检测数={n_det}")
    print("=" * 82)
    print(f"matplotlib 中文字体：{font or '未找到（图上中文会变方块）'}")
    print()
    print("【材质特征】                    本样本      | S01 透明瓶 | S05 标定板")
    print(f"  框面积 [px]                {summ['box_px']:>9.0f}      |"
          f"     2352   |    45435")
    print(f"  整框有效深度占比           {summ['valid_ratio']*100:>8.1f}%      |"
          f"    26.8%   |    87.4%")
    print(f"  中心 60% 有效占比          {summ['roi_valid_ratio']*100:>8.1f}%      |"
          f"     8.1%   |    97.3%")
    print(f"  比值 r                     {summ['ratio_r']:>9.2f}      |"
          f"     0.30   |     1.11")
    print(f"  前景占比                   {summ['fg_frac']*100:>8.1f}%      |"
          f"    74.2%   |    96.4%")
    print(f"  前景落在中心 ROI 内        {summ['fg_in_roi_frac']*100:>8.1f}%      |"
          f"    15.2%   |        —")
    print(f"  零回波帧占比               {summ['zero_return_frac']*100:>8.1f}%")
    print()
    print("【以卷尺真值为参照】判「反射」的关键，M2 新增")
    print(f"  有效回波落在真值 ±5cm 内   {summ['gt_hit_of_valid']*100:>8.1f}%"
          f"   （穿透物体应≈100%：回波确实来自物体，只是稀疏）")
    print(f"  占整框面积的比例           {summ['gt_hit_frac']*100:>8.1f}%")
    print(f"  比真值远 >10cm（怀疑虚像） {summ['behind_frac']*100:>8.1f}%")
    print(f"  比真值近 >10cm（怀疑飞点） {summ['front_frac']*100:>8.1f}%")
    print()
    print("【嵌套 ROI 有效占比剖面】随 ROI 收缩的变化（这是区分穿透/镜面的关键）")
    print("  ROI 比例      1.0      0.8      0.6      0.4      0.2")
    print(f"  本样本    " + "".join(
        f"{summ[k]*100:>8.1f}%" for k in ("v_full", "v_080", "v_060", "v_040", "v_020")))
    print(f"  S01 透明瓶    26.8%       —      8.1%       —        —"
          f"     ← 向内递减 = 中心最透明")
    print(f"  S05 标定板    87.4%       —     97.3%       —        —"
          f"     ← 向内递增 = 中心最实")
    print()
    print("【失效模式判定】")
    for m in MODES:
        if modes.get(m):
            print(f"  {m:<12}{MODE_CN[m]:<10}{modes[m]:>5} / {len(feats)} 帧"
                  f"  ({modes[m]/len(feats)*100:.1f}%)")
    print(f"  → 主模式：{dom_mode}（{MODE_CN[dom_mode]}）")
    print()
    print("【横向偏移】深度簇质心 u − 彩色检测框中心 u")
    print("  （这就是设计草案拿来做决策的那个量：偏差多大 → 改用彩色框中心能省多少）")
    if summ["u_offset_px"] == summ["u_offset_px"]:
        print(f"  {summ['u_offset_px']:+.2f} px  (σ {summ['u_offset_std']:.2f}, n={len(u_off)})"
              f"   ↔ 横向 {summ['u_offset_mm']:+.1f} mm @ {zgt} m")
        print(f"  对照：透明瓶 决策量的第 3 层实测为 25.6 mm（本脚本同口径，"
              f"V4 簇口径略小）")
        print(f"        S05 的「+17.0 px」是**另一个量**（彩色分割质心 − 深度质心），"
              f"不能直接比")
    else:
        print("  无法计算（V4 在所有帧上都拿不到有效深度）")
    print()
    print("【V0~V4p】")
    print(f"{'方法':<24}{'有效':>5}{'MAE-z/m':>11}{'偏差/m':>11}{'STD-z/m':>11}{'最大/m':>11}")
    for meth in METHODS:
        tag = meth.split("_")[0]
        print(f"{meth:<24}{summ[f'{tag}_n']:>5}{summ[f'{tag}_mae']:>11.5f}"
              f"{summ[f'{tag}_bias']:>11.5f}{summ[f'{tag}_std']:>11.5f}"
              f"{summ[f'{tag}_max']:>11.5f}")
    print()
    print("【设计草案两分类规则检验】")
    print("  规则 r>1.0→opaque ｜ r<0.7→transparent ｜ 其余→unknown")
    for b in ("opaque", "transparent", "unknown", "undefined"):
        if branches.get(b):
            print(f"    判为 {b:<12}{BRANCH_CN[b]:<26}{branches[b]:>5} 帧")
    for v, desc in (("ok", "分支与真实模式匹配，策略正确"),
                    ("degraded", "落到保守策略：不注入错误约束，但也没有收益"),
                    ("fail", "分支与真实模式不符 —— 会注入**错误**几何约束")):
        if verdicts.get(v):
            print(f"    {v:<9}{desc:<44}{verdicts[v]:>5} 帧")
    n_fail = verdicts.get("fail", 0)
    if n_fail:
        print(f"  ⚠ {n_fail}/{len(feats)} 帧两分类规则会选错分支 —— "
              f"该材质需要**第三类**处理。")
    else:
        print("  ✓ 两分类规则在本样本上没有选错分支。")
    print()
    print(f"逐帧: {rf}")
    print(f"特征: {ff}")
    print(f"汇总: {summ_path}")
    print(f"图  : {fig_path}")


# ---------------------------------------------------------------- 出图

def make_figure(path, label, z_gt, feats, frames, summ):
    """六格图：有效性掩码 / 深度图 / 嵌套剖面 / 逐列剖面 / 真值直方图 / 逐帧时序。"""
    # 选一帧最具代表性的（valid_ratio 最接近样本均值）
    target = summ["valid_ratio"]
    if frames:
        d, c, bbox, f = min(
            frames, key=lambda t: abs(t[3]["valid_ratio"] - target)
            if t[3]["valid_ratio"] == t[3]["valid_ratio"] else 9e9)
    else:
        d = c = bbox = f = None

    fig, axes = plt.subplots(2, 3, figsize=(18, 9.5))

    # --- 1. 彩色 + 有效掩码
    ax = axes[0][0]
    if c is not None:
        ax.imshow(c[:, :, ::-1])
        vm = valid_mask(d)
        h, w = d.shape[:2]
        x0, y0, x1, y1 = box_bounds(bbox, 1.0, w, h)
        box = np.zeros_like(vm)
        box[y0:y1, x0:x1] = True
        ov = np.zeros((*vm.shape, 4), dtype=np.float32)
        ov[box & vm] = (0, 1, 0, 0.42)      # 绿 = 框内有有效深度
        ov[box & ~vm] = (1, 0, 0, 0.42)     # 红 = 框内无有效深度
        ax.imshow(ov)
        ax.add_patch(plt.Rectangle((bbox[0], bbox[1]), bbox[2] - bbox[0],
                                   bbox[3] - bbox[1], fill=False,
                                   edgecolor="yellow", lw=1.8))
        ax.set_title(f"彩色 + 深度有效性掩码\n绿=有回波 红=无回波  （r={f['ratio_r']:.2f}）")
    ax.axis("off")

    # --- 2. 深度图
    ax = axes[0][1]
    if d is not None:
        dsp = np.where(valid_mask(d), d, np.nan)
        im = ax.imshow(dsp, cmap="turbo",
                       vmin=max(0.0, z_gt - 0.8), vmax=z_gt + 2.0)
        plt.colorbar(im, ax=ax, fraction=0.046, label="深度 [m]")
        ax.set_title(f"对齐深度图（显示范围 {max(0.0, z_gt-0.8):.1f}~{z_gt+2.0:.1f} m）")
    ax.axis("off")

    # --- 3. 嵌套 ROI 剖面
    ax = axes[0][2]
    rs = list(NESTED_RATIOS)
    mine = [summ[k] for k in ("v_full", "v_080", "v_060", "v_040", "v_020")]
    ax.plot(rs, [v * 100 for v in mine], "o-", lw=2.2, ms=8, label=f"{label}（本样本）")
    ax.plot([1.0, 0.6], [REF_TRANSPARENT["v_box"] * 100,
                        REF_TRANSPARENT["v_roi"] * 100],
            "s--", color="tab:blue", alpha=0.75, label="S01 透明瓶")
    ax.plot([1.0, 0.6], [REF_OPAQUE["v_box"] * 100, REF_OPAQUE["v_roi"] * 100],
            "^--", color="tab:green", alpha=0.75, label="S05 标定板")
    ax.set_xlabel("中心 ROI 比例（1.0 = 完整检测框）")
    ax.set_ylabel("有效深度占比 [%]")
    ax.set_title("嵌套 ROI 有效占比剖面\n向内递减=穿透特征，向内递增=不透明特征")
    ax.invert_xaxis()
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)

    # --- 4. 逐列剖面
    ax = axes[1][0]
    cols = np.array([unj(f0["col_profile"]) for f0 in feats], dtype=float)
    xs = np.arange(N_COL_BINS) + 0.5
    if cols.size:
        m = np.nanmean(cols, axis=0)
        s = np.nanstd(cols, axis=0)
        ax.bar(xs, m * 100, yerr=s * 100, capsize=3, color="tab:orange",
               edgecolor="black", lw=0.6)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{i+1}" for i in range(N_COL_BINS)])
    ax.set_xlabel("检测框从左到右等分的第几列")
    ax.set_ylabel("有效深度占比 [%]")
    ax.set_title("逐列有效占比（整框平均 ± 逐帧标准差）\n"
                 "镜面高光带 → 某几列整条缺失，而不是中心缺失")
    ax.grid(alpha=0.3, axis="y")

    # --- 5. 框内深度直方图（带卷尺真值）★ M2 判「反射」的核心证据
    ax = axes[1][1]
    allz = []
    for d0, _c0, b0, _f0 in frames:
        h, w = d0.shape[:2]
        x0, y0, x1, y1 = box_bounds(b0, 1.0, w, h)
        sub = d0[y0:y1, x0:x1]
        v = sub[valid_mask(sub)]
        if v.size:
            allz.append(v)
    if allz:
        allz = np.concatenate(allz)
        lo, hi = max(0.0, z_gt - 1.0), z_gt + 2.5
        ax.hist(allz, bins=90, range=(lo, hi), color="tab:purple",
                edgecolor="black", lw=0.3)
        ax.axvline(z_gt, color="red", lw=2.0, label=f"卷尺真值 {z_gt} m")
        ax.axvspan(z_gt - GT_TOL, z_gt + GT_TOL, color="red", alpha=0.16,
                   label=f"±{GT_TOL*100:.0f} cm 判为「打在物体上」")
        n_out = int((allz > hi).sum())
        ax.set_title(f"检测框内有效深度直方图（累计 {allz.size} px）\n"
                     f"打在真值±5cm内 {summ['gt_hit_of_valid']*100:.1f}% ｜ "
                     f">{hi:.1f} m 有 {n_out} px（未画出）")
        ax.set_xlabel("深度 [m]")
        ax.set_ylabel("像素数")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    else:
        ax.text(0.5, 0.5, "框内完全没有有效深度\n（零回波 —— 镜面最可能的形态）",
                ha="center", va="center", fontsize=13, color="crimson")
        ax.set_title("检测框内有效深度直方图")
        ax.axis("off")

    # --- 6. 逐帧时序：有效占比 与 V4 的 z
    ax = axes[1][2]
    idx = [i for i in range(len(feats))]
    ax.plot(idx, [feats[i]["valid_ratio"] * 100 for i in idx], "o-", ms=3,
            color="tab:orange", label="整框有效占比 [%]")
    ax.set_xlabel("检测序号")
    ax.set_ylabel("有效深度占比 [%]", color="tab:orange")
    ax.tick_params(axis="y", labelcolor="tab:orange")
    ax.set_ylim(0, 105)
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    zv = [feats[i]["z_v4"] for i in idx]
    ax2.plot(idx, zv, "s-", ms=3, color="tab:blue", label="V4 的 z [m]")
    ax2.axhline(z_gt, color="red", lw=1.6, ls="--", label=f"真值 {z_gt} m")
    ax2.axhspan(z_gt - Z_ERR_TOL, z_gt + Z_ERR_TOL, color="red", alpha=0.10)
    ax2.set_ylabel("V4 估计的 z [m]", color="tab:blue")
    ax2.tick_params(axis="y", labelcolor="tab:blue")
    lo2 = min([v for v in zv if v == v] + [z_gt]) - 0.3
    hi2 = max([v for v in zv if v == v] + [z_gt]) + 0.3
    ax2.set_ylim(lo2, hi2)
    ax.set_title("逐帧时序：深度完整性 与 z 估计\n"
                 "红色带 = ±10 cm 容差，出带即判 reflection")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="lower left")

    mc = summ["mode_counts"]
    fig.suptitle(f"M2 金属/镜面材质测试 — {label}\n"
                 f"真值 {z_gt} m ｜ 整框有效 {summ['valid_ratio']*100:.1f}% ｜ "
                 f"r={summ['ratio_r']:.2f} ｜ 打在真值±5cm内 "
                 f"{summ['gt_hit_of_valid']*100:.1f}% ｜ "
                 f"主模式 {MODE_CN[summ['mode_dominant']]} ｜ "
                 f"n={summ['n_det_valid']} ｜ {mc}",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
