#!/usr/bin/env python3
"""在同一份 ROS 2 bag 上运行 V0~V4 反投影消融实验。

实验输入
--------
- /semantics/objects：在线 YOLO 生成的同一批 2D 检测框
- /camera/camera/aligned_depth_to_color/image_raw：原始深度图
- /camera/camera/color/camera_info：相机内参

注意：本脚本不重新运行 YOLO，只改变 2D 框到 3D 的深度估计方法，
因此比较结果只反映反投影策略差异。

默认真值
--------
本次成功样本是 1.5 m 处的美汁源瓶。可用 --z-gt 覆盖。
"""

import argparse
import csv
import math
import os
import sqlite3
import statistics
from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from sensor_msgs.msg import CameraInfo, Image
from semantic_rtabmap_msgs.msg import SemanticObjectArray


METHODS = ("V0_full_median", "V1_full_mean", "V2_roi_median",
           "V3_roi_p20", "V4_roi_p20_cluster")


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def clamp_box(bbox, w, h):
    x0, y0, x1, y1 = [float(v) for v in bbox]
    x0 = max(0, min(w - 1, x0))
    y0 = max(0, min(h - 1, y0))
    x1 = max(x0 + 1, min(w, x1))
    y1 = max(y0 + 1, min(h, y1))
    return x0, y0, x1, y1


def roi_bounds(bbox, ratio, w, h):
    x0, y0, x1, y1 = clamp_box(bbox, w, h)
    if ratio >= 0.999:
        return int(math.floor(x0)), int(math.floor(y0)), int(math.ceil(x1)), int(math.ceil(y1))
    mx = (x0 + x1) * 0.5
    my = (y0 + y1) * 0.5
    hw = max(1.0, (x1 - x0) * ratio * 0.5)
    hh = max(1.0, (y1 - y0) * ratio * 0.5)
    return (
        int(max(0, math.floor(mx - hw))),
        int(max(0, math.floor(my - hh))),
        int(min(w, math.ceil(mx + hw))),
        int(min(h, math.ceil(my + hh))),
    )


def valid_values(patch, min_depth=0.2, max_depth=12.0):
    mask = np.isfinite(patch) & (patch > min_depth) & (patch < max_depth)
    return patch[mask], mask


def estimate_variant(depth, bbox, fx, fy, cx, cy, method,
                     depth_tol=0.35, min_valid=12, min_cluster=10):
    """返回 position(x,y,z), points_used 或 None。"""
    h, w = depth.shape[:2]
    bx0, by0, bx1, by1 = roi_bounds(bbox, 1.0, w, h)
    use_roi = method != "V0_full_median" and method != "V1_full_mean"
    ratio = 0.6 if use_roi else 1.0
    rx0, ry0, rx1, ry1 = roi_bounds(bbox, ratio, w, h)
    roi = depth[ry0:ry1, rx0:rx1]
    vals, roi_valid = valid_values(roi)
    if vals.size < min_valid:
        return None

    if method in ("V0_full_median", "V2_roi_median"):
        z0 = float(np.median(vals))
    else:
        z0 = float(np.percentile(vals, 20.0)) if method in ("V3_roi_p20", "V4_roi_p20_cluster") else float(vals.mean())
    if not (0.2 < z0 < 12.0):
        return None

    if method == "V4_roi_p20_cluster":
        patch = depth[by0:by1, bx0:bx1]
        values, valid = valid_values(patch)
        cluster = valid & (np.abs(patch - z0) <= depth_tol)
        if int(cluster.sum()) < min_cluster:
            cluster = valid
            if int(cluster.sum()) < min_valid:
                return None
        vs, us = np.nonzero(cluster)
        zs = patch[vs, us].astype(np.float64)
        us = us.astype(np.float64) + bx0
        vs = vs.astype(np.float64) + by0
    else:
        # V0~V3 是“深度估计器”基线，不做聚类质心。
        # 为了只比较均值/中位数/分位数和 ROI 的影响，位置的 x/y
        # 统一取检测框中心投影，z 使用该变体的深度估计值 z0。
        # 如果这里再对完整框所有像素求质心，就会把 V0/V1 变成同一个
        # “完整框点云质心”方法，无法完成真正的消融比较。
        bx_center = (float(bbox[0]) + float(bbox[2])) * 0.5
        by_center = (float(bbox[1]) + float(bbox[3])) * 0.5
        position = np.array([
            (bx_center - cx) * z0 / fx,
            (by_center - cy) * z0 / fy,
            z0,
        ], dtype=np.float64)
        return position, int(vals.size), z0

    if zs.size < min_valid:
        return None
    xs = (us - cx) * zs / fx
    ys = (vs - cy) * zs / fy
    position = np.array([xs.mean(), ys.mean(), zs.mean()], dtype=np.float64)
    return position, int(zs.size), z0


def resolve_db_path(bag):
    """定位可直接读取的 sqlite 数据库文件。

    run_once.sh 录包时用了 zstd FILE 压缩，磁盘上是 *.db3.zstd。
    rosbag2_py 的 SequentialReader 在 Humble 下会把它当 sqlite 直接打开而失败，
    因此这里直接用 sqlite3 读数据库：
      1. 若目录内已有解压好的 *.db3，直接用它；
      2. 否则若存在 *.db3.zstd，提示先解压（避免在本脚本里偷偷占几 GB 磁盘）。
    """
    bag = os.path.abspath(bag)
    if os.path.isfile(bag):
        return bag
    dbs = sorted(
        os.path.join(bag, f) for f in os.listdir(bag) if f.endswith(".db3")
    )
    if dbs:
        return dbs[0]
    zst = [f for f in os.listdir(bag) if f.endswith(".db3.zstd")]
    if zst:
        raise RuntimeError(
            "bag 是 zstd 压缩的，请先解压再运行：\n"
            f"  mkdir -p {bag}/_plain && cd {bag}/_plain && "
            f"zstd -d -o {zst[0][:-5]} ../{zst[0]}\n"
            "然后用 --bag-dir 指向 _plain 目录。"
        )
    raise RuntimeError(f"在 {bag} 里找不到 .db3 或 .db3.zstd")


def open_reader(path):
    """返回 (连接, topic_id -> (名称, 类型)) 供直接读取消息。"""
    db = resolve_db_path(path)
    conn = sqlite3.connect(db)
    topic_map = {}
    for tid, name, typ in conn.execute("SELECT id, name, type FROM topics"):
        topic_map[int(tid)] = (str(name), str(typ))
    return conn, topic_map


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="ROS 2 bag directory")
    ap.add_argument("--out", default="~/VIO/data/ablation_s01")
    ap.add_argument("--z-gt", type=float, default=1.5)
    ap.add_argument("--class-name", default="bottle")
    ap.add_argument("--max-objects", type=int, default=0,
                    help="最多处理多少个检测；0=全部")
    args = ap.parse_args()
    out_dir = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "raw_results.csv")
    summary_path = os.path.join(out_dir, "summary.csv")

    type_map = {
        "/semantics/objects": SemanticObjectArray,
        "/camera/camera/aligned_depth_to_color/image_raw": Image,
        "/camera/camera/color/camera_info": CameraInfo,
    }
    conn, topic_map = open_reader(os.path.expanduser(args.bag))
    bridge = CvBridge()
    depth_history = deque(maxlen=8)
    latest_info = None
    rows = []
    detection_count = 0
    depth_count = 0
    info_count = 0
    skipped_no_depth = 0
    available = {name for name, _ in topic_map.values()}
    missing = set(type_map) - available
    if missing:
        raise RuntimeError(f"bag 缺少话题: {sorted(missing)}")

    # 直接按时间顺序扫描 sqlite 消息表（避开 rosbag2 压缩插件问题）。
    cursor = conn.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
    )
    for tid, bag_time, data in cursor:
        entry = topic_map.get(int(tid))
        if entry is None:
            continue
        topic = entry[0]
        if topic not in type_map:
            continue
        msg = deserialize_message(bytes(data), type_map[topic])
        if topic == "/camera/camera/color/camera_info":
            latest_info = msg
            info_count += 1
            continue
        if topic == "/camera/camera/aligned_depth_to_color/image_raw":
            try:
                raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if msg.encoding == "16UC1":
                    depth = raw.astype(np.float32) * 0.001
                elif msg.encoding == "32FC1":
                    depth = raw.astype(np.float32)
                else:
                    continue
                depth_history.append((stamp_ns(msg), depth))
                depth_count += 1
            except Exception:
                continue
            continue

        # 语义对象消息：选择时间上最近的已到达深度帧。
        if latest_info is None or not depth_history:
            continue
        arr = msg
        candidates = [x for x in arr.objects if x.class_name == args.class_name]
        if not candidates:
            continue
        # 在线检测器每帧通常只有一个瓶子；若有多个，逐个记录。
        for obj in candidates:
            if args.max_objects and detection_count >= args.max_objects:
                break
            obj_t = stamp_ns(arr)
            depth_t, depth = min(depth_history, key=lambda x: abs(x[0] - obj_t))
            k = np.asarray(latest_info.k, dtype=np.float64)
            fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
            bbox = tuple(float(v) for v in obj.bbox)
            for method in METHODS:
                result = estimate_variant(depth, bbox, fx, fy, cx, cy, method)
                row = {
                    "detection_index": detection_count,
                    "object_stamp_ns": obj_t,
                    "depth_stamp_ns": depth_t,
                    "class_name": obj.class_name,
                    "method": method,
                    "bbox_x0": bbox[0], "bbox_y0": bbox[1],
                    "bbox_x1": bbox[2], "bbox_y1": bbox[3],
                    "z_gt": args.z_gt,
                    "valid": int(result is not None),
                    "x_est": "", "y_est": "", "z_est": "",
                    "range_error": "", "points_used": "", "z0": "",
                }
                if result is not None:
                    pos, points_used, z0 = result
                    row.update({
                        "x_est": float(pos[0]), "y_est": float(pos[1]),
                        "z_est": float(pos[2]),
                        "range_error": abs(float(pos[2]) - args.z_gt),
                        "points_used": points_used,
                        "z0": z0,
                    })
                rows.append(row)
            detection_count += 1
        if args.max_objects and detection_count >= args.max_objects:
            break

    fieldnames = [
        "detection_index", "object_stamp_ns", "depth_stamp_ns", "class_name",
        "method", "bbox_x0", "bbox_y0", "bbox_x1", "bbox_y1", "z_gt",
        "valid", "x_est", "y_est", "z_est", "range_error", "points_used", "z0",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary_rows = []
    for method in METHODS:
        valid_rows = [r for r in rows if r["method"] == method and r["valid"]]
        errors = [float(r["range_error"]) for r in valid_rows]
        zs = [float(r["z_est"]) for r in valid_rows]
        summary_rows.append({
            "method": method,
            "detections": len(errors),
            "mae_z_m": statistics.fmean(errors) if errors else "",
            "std_z_m": statistics.pstdev(zs) if len(zs) > 1 else (0.0 if zs else ""),
            "max_error_z_m": max(errors) if errors else "",
            "failure_rate": 1.0 - len(errors) / detection_count if detection_count else "",
            "mean_points": statistics.fmean(float(r["points_used"]) for r in valid_rows) if valid_rows else "",
        })
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        fields = ["method", "detections", "mae_z_m", "std_z_m", "max_error_z_m", "failure_rate", "mean_points"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary_rows)

    print(f"bag: {os.path.expanduser(args.bag)}")
    print(f"深度帧: {depth_count} | 相机内参: {info_count} | bottle 检测: {detection_count}")
    print(f"结果: {csv_path}")
    print(f"汇总: {summary_path}")
    print("\n方法                 有效帧   MAE-z(m)  STD-z(m)  最大误差(m)  失败率")
    for r in summary_rows:
        print(f"{r['method']:<21} {r['detections']:>5}   {str(r['mae_z_m'])[:9]:>9}  {str(r['std_z_m'])[:9]:>9}  {str(r['max_error_z_m'])[:10]:>10}  {str(r['failure_rate'])[:7]:>7}")


if __name__ == "__main__":
    main()
