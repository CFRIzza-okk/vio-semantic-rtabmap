#!/usr/bin/env python3
"""只读的 D435 点云观测节点。

用途
----
在**不修改 Nav2 任何配置**的前提下，把 D435 的点云接进来观察，回答三个问题：

1. 点云到底有没有在发、频率多少、点数多少、有效率如何；
2. 地面在相机坐标系里的位置 —— 这决定了将来 ``min_obstacle_height`` 该设多少，
   设错了不是"效果差一点"，而是要么把地面标成致命障碍（车以为被围住原地不动），
   要么把真障碍滤掉（危险）；
3. **反推相机外参**。地面在相机光学坐标系里是一个平面：相机水平时它落在
   ``y = 相机离地高度``（常数），俯仰时 ``y`` 随 ``z`` 线性变化，斜率就是俯仰角。
   所以对地面点做一次直线拟合，就能量出 ``base_link -> camera_link`` 缺的
   高度和俯仰角，不必再用尺子量。

安全属性
--------
本节点**纯只读**：只订阅点云、只打印和发布统计数据。
它不发布速度、不发布 TF、不写 costmap，运行它不可能让机器人动。

报告里的 ``configured_mount`` 字段会**从 TF 树实时读取** ``base_link -> camera_link``
并回报实际值，所以它反映的是"现在真的配了什么"，不是一句会过期的说明。

若 ``base_link`` 不在 TF 树上（例如只跑本文件的观测启动、没跑
``r680_navigation.launch.py``），该字段会报 ``unavailable`` —— 这是如实告知，
因为此时确实无从得知车体系与相机的相对关系。

在拿到真实 TF 之前，所有以 ``base_footprint`` 为参考的输出仍**不可信**，
因此默认只用相机自身坐标系报告。
"""

from __future__ import annotations

import itertools
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:  # Humble 提供 read_points_numpy，优先使用
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover
    pc2 = None


def pitch_from_quaternion(q) -> float:
    """Pitch in radians from an RPY quaternion. Positive = looking down.

    Kept as a free function so the sign convention is unit-testable: this is the
    one number where getting the sign wrong yields a camera tilted 2 degrees the
    *wrong* way instead of not tilted at all, which looks plausible in a log.
    """
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, sin_pitch)))


class PointCloudInspector(Node):
    """Report rate, density and the height profile of a depth cloud."""

    def __init__(self) -> None:
        super().__init__("point_cloud_inspector")
        self.declare_parameter(
            "cloud_topic", "/camera/camera/depth/color/points"
        )
        self.declare_parameter("report_period", 5.0)
        self.declare_parameter("max_analyze_points", 60000)
        # 只在可靠深度范围内拟合地面，D435 超过约 3 米后精度明显下降。
        self.declare_parameter("floor_z_min", 0.4)
        self.declare_parameter("floor_z_max", 3.0)
        self.declare_parameter("floor_bin_width", 0.3)
        self.declare_parameter("min_points_per_bin", 40)
        # A z slice only counts as floor if it is a *plane*: within one 0.3 m
        # bin the floor's y varies by bin_width*tan(pitch), while clutter
        # spreads over everything. Measured on the real robot: floor slices had
        # (p99-p90) of 0.003-0.019 m, clutter slices 0.069-0.265 m, with nothing
        # in between -- so 0.05 separates them cleanly.
        self.declare_parameter("floor_max_slice_spread", 0.05)
        # Being locally flat is not enough: a wall segment or a box face is
        # locally flat too, but sits at a completely different height. Real
        # floor slices must additionally lie on *one* line. Fit by consensus and
        # keep the largest mutually consistent set; measured on the real robot,
        # a slice at z=2.95 m was planar (spread 0.043) yet sat 10 cm below
        # where the floor must be, and dragged the pitch to -0.56 deg (the
        # camera apparently looking *up*).
        # 8 mm, not 15: a genuinely flat floor's slices agree to a couple of mm
        # (measured residuals 0.000/0.002/0.000 m on the real robot), so a loose
        # tolerance lets a steeper line swallow one bad slice and win on inlier
        # count alone -- observed: 4 inliers at 0.200 m and pitch -4.2 deg
        # (looking *up*) beat the correct 3-inlier answer at 0.181 m / 2.8 deg.
        self.declare_parameter("floor_inlier_tol", 0.008)
        # A candidate line must span at least this much z, otherwise three
        # adjacent slices can define a wildly sloped line that fits nothing else.
        self.declare_parameter("floor_min_z_span", 0.4)
        # |dy/dz| beyond this is not a floor (0.5 ~ 26.6 deg of pitch).
        self.declare_parameter("floor_max_abs_slope", 0.5)
        # Minimum number of raw points a candidate line must capture to count.
        self.declare_parameter("floor_min_support", 2000)
        # Half-width of the symmetric band used for the final refit.
        self.declare_parameter("floor_refit_band", 0.02)

        # realsense2_camera 以 SENSOR_DATA（BEST_EFFORT, depth 5）发布点云。
        # RELIABLE 订阅者与 BEST_EFFORT 发布者**不兼容**，会一帧都收不到，
        # 而且不报错、话题也照样出现在 topic list 里 —— 所以这里必须 best_effort。
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._count = 0
        self._first_stamp = 0.0
        self._last_stamp = 0.0
        self._last_report = 0.0
        self._last_msg = None
        self._reported_once = False

        self._pub = self.create_publisher(String, "/depth_cloud/report", qos)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            qos,
        )
        # 只读地跟踪 TF，用来如实回报"当前配置的相机安装位姿"。
        # 订阅 TF 不会发布任何东西，不影响本节点的只读属性。
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            "inspector watching %s (read-only, no motion, no costmap writes)"
            % self.get_parameter("cloud_topic").value
        )

    # ------------------------------------------------------------------
    def _on_cloud(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if self._first_stamp == 0.0:
            self._first_stamp = now
        self._last_stamp = now
        self._count += 1
        self._last_msg = msg

    def _points_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Return an ``(N, 3)`` float32 array of finite points."""
        limit = int(self.get_parameter("max_analyze_points").value)
        if pc2 is not None:
            try:
                arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
            except Exception:
                arr = None
            if arr is not None and arr.size:
                arr = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
                if arr.shape[0] > limit:
                    step = max(1, arr.shape[0] // limit)
                    arr = arr[::step]
                return arr
        # 退化路径：按 point_step / 字段偏移自行解析。
        fields = {f.name: f.offset for f in msg.fields}
        if not {"x", "y", "z"} <= set(fields):
            return np.zeros((0, 3), dtype=np.float32)
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        usable = (raw.size // step) * step
        raw = raw[:usable].reshape(-1, step)
        if raw.shape[0] > limit:
            stride = max(1, raw.shape[0] // limit)
            raw = raw[::stride]
        out = np.empty((raw.shape[0], 3), dtype=np.float32)
        for i, name in enumerate(("x", "y", "z")):
            off = fields[name]
            out[:, i] = raw[:, off:off + 4].copy().view(np.float32).reshape(-1)
        return out

    # ------------------------------------------------------------------
    def _analyze(self, msg: PointCloud2) -> dict:
        started = time.monotonic()
        pts = self._points_xyz(msg)
        total = int(pts.shape[0])
        if total == 0:
            return {"error": "no points could be parsed"}

        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        # 深度相机的"无效"点常以全 0 出现，单独统计。
        nonzero = ~np.all(pts == 0.0, axis=1)
        pts = pts[nonzero]
        n = int(pts.shape[0])
        if n == 0:
            return {"error": "all points are NaN or zero"}

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        report = {
            "frame_id": msg.header.frame_id,
            "width": int(msg.width),
            "height": int(msg.height),
            "organized": bool(msg.height > 1),
            "point_step": int(msg.point_step),
            "sampled_points": total,
            "valid_ratio_after_filter": round(n / float(total), 3),
            "z_range_m": [round(float(np.percentile(z, 1)), 3),
                          round(float(np.percentile(z, 99)), 3)],
            "y_percentiles_m": {
                "p05": round(float(np.percentile(y, 5)), 3),
                "p50": round(float(np.percentile(y, 50)), 3),
                "p95": round(float(np.percentile(y, 95)), 3),
            },
            "analyze_ms": round((time.monotonic() - started) * 1000.0, 1),
            # 这里说的是"启动文件里配置的 TF 仍是默认 0"，不是"没量过"。
            # 实测值记录在 R680_DEPTH_CLOUD_OBSTACLES.md；在把它写进静态 TF
            # 之前，一切以 base_footprint 为参考的输出都不可信。
            "configured_mount": self._configured_mount(),
        }
        report["floor_fit"] = self._floor_fit(x, y, z)
        report["z_histogram"] = self._z_histogram(z)
        report["near_surface"] = self._near_surface(y, z)
        # 只有真识别出地面时才给高度带。地面都没识别出来时，"离地面多高"
        # 本身就是未定义的，这时还输出百分比只会被误当成已标定好的阈值。
        if report["floor_fit"].get("status") == "ok":
            report["candidate_bands"] = self._band_sweep(y, z)
        else:
            report["candidate_bands"] = {}
            report["candidate_bands_note"] = (
                "withheld: floor not identified, height-above-floor is "
                "undefined and any band numbers would mislead"
            )
        return report

    # ------------------------------------------------------------------
    def _configured_mount(self) -> dict:
        """Report the *actual* ``base_link -> camera_link`` transform from TF.

        This used to be a hardcoded string claiming the extrinsics were still
        zero.  That was true when written and became a lie the moment the
        measured mount was applied to ``r680_navigation.launch.py`` -- a
        diagnostic that confidently reports the wrong thing is worse than one
        that reports nothing.  Look the transform up instead.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link", "camera_link", rclpy.time.Time()
            )
        except Exception as exc:  # tf2 raises several unrelated types
            return {
                "status": "unavailable",
                "note": "base_link -> camera_link is not on the TF tree; start "
                        "r680_navigation.launch.py to publish the configured "
                        "mount. Height-above-floor below stays camera-relative.",
                "error": type(exc).__name__,
            }

        tr = tf.transform.translation
        pitch = pitch_from_quaternion(tf.transform.rotation)
        return {
            "status": "ok",
            "translation_m": {
                "x": round(tr.x, 4), "y": round(tr.y, 4), "z": round(tr.z, 4)
            },
            "pitch_deg": round(math.degrees(pitch), 2),
            "pitch_sign_note": "positive = camera looking down",
            "frame_id": tf.header.frame_id,
            "child_frame_id": tf.child_frame_id,
        }

    def _floor_fit(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> dict:
        """Fit the floor line y(z) to estimate mount height and pitch.

        The floor is the *lowest* surface in front of the camera, i.e. the
        largest ``y`` (optical y points down). Taking a high percentile of y
        inside each z slice picks the floor even when obstacles are present.
        A level camera gives a constant y; a pitched camera gives a slope, and
        that slope is the pitch angle.

        A slice is only used if it is *planar*.  The 90th percentile of y finds
        the floor only when the floor is the lowest thing in that slice; in a
        slice full of clutter the percentile lands on whatever furniture
        happens to be there, at an arbitrary height.  Fitting through those
        points drags the line away from the real floor while still returning a
        confident-looking number -- measured on the real robot, nine slices of
        which only three were floor gave r2 = 0.11 and a height 3.7 cm off.

        The (p99 - p90) gap of y separates the two cases: a floor slice is thin
        (it spans only bin_width*tan(pitch)), a clutter slice is not.
        """
        z_min = float(self.get_parameter("floor_z_min").value)
        z_max = float(self.get_parameter("floor_z_max").value)
        width = float(self.get_parameter("floor_bin_width").value)
        min_pts = int(self.get_parameter("min_points_per_bin").value)
        max_spread = float(self.get_parameter("floor_max_slice_spread").value)

        edges = np.arange(z_min, z_max + width, width)
        mid, y_low, z_eff, counts, spreads = [], [], [], [], []
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = (z >= lo) & (z < hi)
            cnt = int(sel.sum())
            # 每个 z 切片都记下点数：拟合失败时必须能区分"看不到地面"
            # 和"点数不够"，否则只丢一句含糊的失败没法定位。
            mid.append(0.5 * (lo + hi))
            counts.append(cnt)
            if cnt >= min_pts:
                yy, zz = y[sel], z[sel]
                p90 = float(np.percentile(yy, 90))
                p99 = float(np.percentile(yy, 99))
                spreads.append(p99 - p90)
                # 横坐标必须用这批点自己的 z，而不是切片中心。
                # 切片中心和"地面上那批点"所在的 z 常常差好几厘米：俯仰一大
                # （12 度时一格地面横跨 6 cm）这个错位足以把本该共线的切片
                # 打散——实测合成 12 度数据因此只认到 4/8 个切片，拟合出
                # 俯仰 14.5 度、高度 0.27 m（真值 0.17 m），差了一倍。
                near = yy >= p90
                y_low.append(float(yy[near].mean()))
                z_eff.append(float(zz[near].mean()))
            else:
                y_low.append(float("nan"))
                z_eff.append(float("nan"))
                spreads.append(float("nan"))

        y_low_a = np.asarray(y_low)
        spread_a = np.asarray(spreads)
        has_pts = np.isfinite(y_low_a)
        planar = has_pts & (spread_a <= max_spread)

        diag = {
            "bin_centers_m": [round(v, 2) for v in mid],
            # 拟合实际使用的横坐标（地面候选点自己的 z），与切片中心不同。
            "bin_z_eff_m": [None if not np.isfinite(v) else round(v, 3)
                            for v in np.asarray(z_eff)],
            "bin_point_counts": counts,
            "min_points_per_bin": min_pts,
            # 把每个切片的平整度和判定一起报出来，这样"为什么被丢掉"是可见的，
            # 而不是只给一个 status 让人猜。
            "bin_y_p90_m": [None if not np.isfinite(v) else round(v, 3)
                            for v in y_low_a],
            "bin_spread_m": [None if not np.isfinite(v) else round(v, 3)
                             for v in spread_a],
            "bin_is_planar": [bool(v) for v in planar],
            "max_slice_spread_m": max_spread,
        }

        if int(planar.sum()) < 3:
            if int(has_pts.sum()) < 3:
                return {
                    "status": "not_enough_z_slices",
                    "reason": "fewer than 3 z slices hold enough points; the "
                              "floor is probably not visible within %.1f..%.1f m"
                              % (z_min, z_max),
                    **diag,
                }
            # 点数够但不够平：场景太杂，或相机没朝地面。这和"看不到"是两回事，
            # 必须分开报，否则调参会朝错方向修。
            return {
                "status": "not_enough_planar_slices",
                "reason": "only %d of %d populated z slices are planar "
                          "((p99-p90) <= %.3f m); the scene is cluttered or the "
                          "camera is not looking at open floor"
                          % (int(planar.sum()), int(has_pts.sum()), max_spread),
                **diag,
            }

        # Being planar is necessary but not sufficient: a wall or a box face is
        # planar too, just at the wrong height.  The floor slices must also lie
        # on *one* line, so search the lines through every pair of candidates.
        # How the winning line is chosen is explained at the scoring loop below.
        cand = [(ze, v) for ze, v, keep in zip(z_eff, y_low_a, planar) if keep]
        cand_counts = [c for c, keep in zip(counts, planar) if keep]
        inlier_tol = float(self.get_parameter("floor_inlier_tol").value)
        min_span = float(self.get_parameter("floor_min_z_span").value)
        max_slope = float(self.get_parameter("floor_max_abs_slope").value)
        min_support = int(self.get_parameter("floor_min_support").value)
        band = float(self.get_parameter("floor_refit_band").value)

        # Which line is the floor? Score it by the *points* its inlier slices
        # hold, not by how many slices agree.
        #
        # Two simpler rules were tried and each failed a scene that matters:
        #   * "most inlier slices" -- on a synthetic scene of five slabs beyond
        #     the floor's visible extent, four slab slices happened to be nearly
        #     collinear and beat the three genuine floor slices, 4 > 3;
        #   * "lowest line wins" (the floor *is* the lowest surface) -- that same
        #     scene has a slab far *below* the floor line, impossible in reality
        #     yet unambiguously "lowest", and the real robot produced exactly
        #     such a slice (y = 0.170 at z = 2.95 m).
        #
        # Scoring whole lines against the raw cloud also fails: a line tilted to
        # bridge the floor and a far slab captures floor + slab points (measured
        # 0.205 m / 4.49 deg). Weighting the consensus by each slice's own point
        # count keeps the two behaviours that matter -- the floor holds most of
        # the points, and a genuinely collinear minority can no longer win on
        # slice count alone.
        best = None
        for (z1, y1), (z2, y2) in itertools.combinations(cand, 2):
            if abs(z2 - z1) < 1e-9:
                continue
            slope = (y2 - y1) / (z2 - z1)
            if abs(slope) > max_slope:
                continue
            intercept = y1 - slope * z1
            resid = [abs(yy - (slope * zz + intercept)) for zz, yy in cand]
            # index into `cand` -> index into the original per-bin arrays
            members = [i for i, r in enumerate(resid) if r <= inlier_tol]
            if len(members) < 3:
                continue
            zs = [cand[i][0] for i in members]
            if max(zs) - min(zs) < min_span:
                continue
            weight = sum(cand_counts[i] for i in members)
            if weight < min_support:
                continue
            worst = max(resid[i] for i in members)
            key = (weight, len(members), -worst)
            if best is None or key > best[0]:
                best = (key, members, slope, intercept)

        diag["inlier_tol_m"] = inlier_tol
        diag["min_z_span_m"] = min_span
        diag["min_support_points"] = min_support

        if best is None:
            # Planar slices exist but no line through them explains enough of the
            # cloud over a long enough span: the scene is a set of unrelated flat
            # things, not one floor.
            diag["floor_consensus_slices"] = []
            return {
                "status": "no_consistent_floor_plane",
                "reason": "no line explains >= %d points over a z span of "
                          ">= %.2f m; no single floor plane is visible"
                          % (min_support, min_span),
                **diag,
            }

        _key, members, slope, intercept = best

        # Final symmetric refit against the raw cloud.
        #
        # The candidates above are the mean of the top decile of y per slice, and
        # "take a high percentile" means picking the maximum out of the noise:
        # that mean sits 1.755*sigma_y above the true floor, so the height came
        # out systematically high (+7.0 mm at sigma_y = 4 mm -- exactly
        # 1.755 * 4 mm). Selecting with a symmetric band has no preferred side
        # and removes the offset, while leaving the slope untouched (a constant
        # y shift cannot change a slope, which is why the pitch was right all
        # along even while the height was not).
        on_plane = np.abs(y - (slope * z + intercept)) <= band
        if int(on_plane.sum()) >= 100:
            slope, intercept = np.polyfit(z[on_plane], y[on_plane], 1)

        # Which percentile candidates ended up on the winning line: the numbers a
        # reader needs in order to judge whether the winner really is the floor.
        members = [i for i, (zz, yy) in enumerate(cand)
                   if abs(yy - (slope * zz + intercept)) <= band]
        diag["floor_consensus_slices"] = [round(cand[i][0], 2) for i in members]

        # Quality is measured on the points that were actually fitted.
        #
        # It used to be measured on the per-bin candidates, which yields a
        # *negative* r2 even for a perfectly good floor: those candidates are the
        # mean of the top decile of y, so by construction they sit ~1.755*sigma_y
        # above the final line -- exactly the offset the refit exists to remove.
        # The real robot reported r2 = -0.682 on a fit that was fine. A metric
        # that calls a good fit bad is worse than no metric at all.
        on_plane = np.abs(y - (slope * z + intercept)) <= band
        z_fit, y_fit = z[on_plane], y[on_plane]
        pred = slope * z_fit + intercept
        ss_res = float(((y_fit - pred) ** 2).sum())
        ss_tot = float(((y_fit - y_fit.mean()) ** 2).sum())
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 1.0
        diag["refit_points"] = int(on_plane.sum())

        pitch_rad = -float(slope)
        return {
            "status": "ok",
            "slices": len(members),
            "slices_rejected_clutter": int(has_pts.sum() - planar.sum()),
            "slices_rejected_inconsistent": int(planar.sum() - len(members)),
            "slope_dy_dz": round(float(slope), 4),
            "y_at_z0_m": round(float(intercept), 3),
            "r2": round(r2, 3),
            # 正数 = 相机朝下俯视
            "estimated_pitch_deg_down": round(math.degrees(pitch_rad), 2),
            "estimated_mount_height_m": round(
                float(intercept) * math.cos(pitch_rad), 3
            ),
            "note": "estimate from the floor plane; robot must be "
                    "stationary on flat ground with floor visible ahead",
            **diag,
        }

    def _near_surface(self, y: np.ndarray, z: np.ndarray,
                      z_lo: float = 0.3, z_hi: float = 0.9) -> dict:
        """Characterise the surface right in front of the camera.

        Useful when the floor is *not* visible further out: the nearest cluster
        is often the ground (or a table) right under the lens. A tight y
        spread means a plane, i.e. a surface we can measure a height from;
        a wide spread means it is not a horizontal plane at all.
        """
        sel = (z >= z_lo) & (z <= z_hi)
        n = int(sel.sum())
        if n < 100:
            return {"status": "too_few_points", "points": n}
        yy = y[sel]
        p25, p50, p75, p90, p99 = (
            float(np.percentile(yy, q)) for q in (25, 50, 75, 90, 99)
        )
        return {
            "status": "ok",
            "points": n,
            "z_window_m": [z_lo, z_hi],
            "y_p50_m": round(p50, 3),
            "y_p90_m": round(p90, 3),
            "y_p99_m": round(p99, 3),
            "y_iqr_m": round(p75 - p25, 3),
            "planar": bool((p75 - p25) < 0.10),
            "hint": "if planar, y_p50 approximates the camera height above "
                    "that surface (extrinsics still 0 => treat as indicative)",
        }

    def _z_histogram(self, z: np.ndarray, bin_width: float = 0.5,
                     z_max: float = 6.0) -> dict:
        """Forward-distance histogram — shows where the scene actually is."""
        edges = np.arange(0.0, z_max + bin_width, bin_width)
        counts, _ = np.histogram(z, bins=edges)
        return {
            "bin_width_m": bin_width,
            "bins": {
                "%.1f-%.1f" % (edges[i], edges[i + 1]): int(c)
                for i, c in enumerate(counts)
            },
        }

    def _band_sweep(self, y: np.ndarray, z: np.ndarray) -> dict:
        """Show how many points survive for candidate height bands.

        Because the camera frame's y is down-positive, a point is *above the
        ground* by ``(floor_y - y)``. Until the extrinsics are measured this
        uses the median floor estimate, and it is reported as a relative sweep
        rather than a final answer.
        """
        z_max = float(self.get_parameter("floor_z_max").value)
        keep = z <= z_max
        if int(keep.sum()) < 50:
            return {}
        yy = y[keep]
        floor_y = float(np.percentile(yy, 90))
        height_above_floor = floor_y - yy
        out = {}
        for minimum in (0.0, 0.05, 0.10, 0.15, 0.20, 0.30):
            frac = float((height_above_floor > minimum).mean())
            out["min_height_%.2f" % minimum] = round(frac, 4)
        return out

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        period = float(self.get_parameter("report_period").value)

        if self._count == 0:
            if now - self._last_report > period:
                self._last_report = now
                self.get_logger().warn(
                    "no point cloud received yet on %s"
                    % self.get_parameter("cloud_topic").value
                )
            return

        if not self._reported_once or now - self._last_report >= period:
            elapsed = max(1e-6, now - self._first_stamp)
            hz = self._count / elapsed
            report = {
                "rate_hz": round(hz, 2),
                "messages": self._count,
                "last_msg_age_s": round(now - self._last_stamp, 2),
            }
            if self._last_msg is not None:
                report.update(self._analyze(self._last_msg))
            self._last_report = now
            self._reported_once = True
            self._pub.publish(String(data=json.dumps(report)))
            self._log_human(report)

    def _log_human(self, report: dict) -> None:
        if "error" in report:
            self.get_logger().warn("cloud analysis failed: %s" % report["error"])
            return
        fit = report.get("floor_fit")
        lines = [
            "rate=%.1f Hz  %dx%d points=%d  valid=%.2f  frame=%s"
            % (
                report["rate_hz"],
                report["width"],
                report["height"],
                report["sampled_points"],
                report["valid_ratio_after_filter"],
                report["frame_id"],
            ),
            "z range %.2f..%.2f m   y p05/p50/p95 = %.3f / %.3f / %.3f m"
            % (
                report["z_range_m"][0],
                report["z_range_m"][1],
                report["y_percentiles_m"]["p05"],
                report["y_percentiles_m"]["p50"],
                report["y_percentiles_m"]["p95"],
            ),
        ]
        if isinstance(fit, dict) and fit.get("status") == "ok":
            lines.append(
                "floor fit: height=%.3f m  pitch=%.2f deg down  r2=%.3f  (%d slices)"
                % (
                    fit["estimated_mount_height_m"],
                    fit["estimated_pitch_deg_down"],
                    fit["r2"],
                    fit["slices"],
                )
            )
        elif isinstance(fit, dict):
            lines.append(
                "floor fit FAILED (%s); points per z slice: %s"
                % (fit.get("reason", fit.get("status")), fit.get("bin_point_counts"))
            )
        else:
            lines.append("floor fit: %s" % fit)
        bands = report.get("candidate_bands") or {}
        if bands:
            lines.append(
                "points above floor: "
                + "  ".join("%s=%.1f%%" % (k.replace("min_height_", ">"), v * 100)
                            for k, v in bands.items())
            )
        else:
            lines.append("points above floor: WITHHELD (floor not identified)")
        near = report.get("near_surface") or {}
        if near.get("status") == "ok":
            lines.append(
                "near surface %.1f-%.1f m: %d pts  y p50=%.3f p90=%.3f  iqr=%.3f  planar=%s"
                % (
                    near["z_window_m"][0],
                    near["z_window_m"][1],
                    near["points"],
                    near["y_p50_m"],
                    near["y_p90_m"],
                    near["y_iqr_m"],
                    near["planar"],
                )
            )
        mount = report["configured_mount"]
        if mount.get("status") == "ok":
            lines.append(
                "configured mount base_link->camera_link: "
                "x=%(x).3f y=%(y).3f z=%(z).3f pitch=%(p).2f deg down"
                % {
                    "x": mount["translation_m"]["x"],
                    "y": mount["translation_m"]["y"],
                    "z": mount["translation_m"]["z"],
                    "p": mount["pitch_deg"],
                }
            )
        else:
            lines.append("configured mount: unavailable (%s)" % mount.get("error", "no TF"))
        self.get_logger().info(" | ".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
