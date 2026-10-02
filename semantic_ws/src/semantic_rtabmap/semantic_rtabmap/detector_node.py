#!/usr/bin/env python3
"""semantic_detector: YOLO 2D 检测 -> 深度反投影 -> 3D 语义物体（Phase 1 感知层）

数据流:
    /camera/camera/color/image_raw                   (rgb8)
    /camera/camera/aligned_depth_to_color/image_raw  (16UC1, 已对齐到彩色)
    /camera/camera/color/camera_info                (内参)
        -> ApproximateTimeSynchronizer 时间同步
        -> 最新帧邮箱(mailbox): 只保留最新一帧, 处理不过来的旧帧直接丢
        -> 推理线程: YOLO(TensorRT) -> 2D 框 -> 深度反投影 -> 3D 质心
        -> TF: camera_color_optical_frame -> target_frame (默认 map)
        -> 发布 /semantics/objects   (SemanticObjectArray)
        -> 发布 /semantics/markers   (MarkerArray, RViz 直接可看)
        -> 发布 /semantics/debug_image (可选, 带框和距离标注)

为什么用"邮箱 + 单推理线程"而不是直接在回调里跑推理:
    在 Orin NX 8GB 上 RTAB-Map 本身就要吃 CPU/GPU。如果同步回调直接跑
    YOLO(19ms), 订阅队列会堆积, 延迟越滚越大, 最终拖垮整个 SLAM。
    邮箱模式保证"永远只处理最新的一帧", 处理能力不足时自动降帧,
    延迟恒定, 不会雪崩。
"""

import math
import os
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from semantic_rtabmap.backprojection import backproject_bbox, depth_to_meters
from semantic_rtabmap_msgs.msg import SemanticObject, SemanticObjectArray


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 -> 3x3 旋转矩阵（不引入 tf_transformations 依赖）。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def class_color(class_id: int) -> ColorRGBA:
    """按类别 ID 生成一个稳定的伪随机颜色，RViz 里同类同色。"""
    h = (int(class_id) * 2654435761) & 0xFFFFFFFF
    r = ((h >> 16) & 0xFF) / 255.0
    g = ((h >> 8) & 0xFF) / 255.0
    b = (h & 0xFF) / 255.0
    # 提亮，避免深色在 RViz 里看不清
    return ColorRGBA(r=0.35 + 0.65 * r, g=0.35 + 0.65 * g, b=0.35 + 0.65 * b, a=1.0)


class SemanticDetector(Node):
    def __init__(self) -> None:
        super().__init__("semantic_detector")

        # ------------------------- 参数 -------------------------
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("objects_topic", "/semantics/objects")
        self.declare_parameter("markers_topic", "/semantics/markers")
        self.declare_parameter("debug_image_topic", "/semantics/debug_image")
        self.declare_parameter("label_markers_topic", "/semantics/label_markers")
        self.declare_parameter("publish_label_markers", True)
        self.declare_parameter("colored_cloud_topic", "/semantics/colored_cloud")
        self.declare_parameter("publish_colored_cloud", True)
        self.declare_parameter("colored_cloud_stride", 2)

        # 模型: 优先用 TensorRT engine, 回退到 .pt
        self.declare_parameter(
            "model_path", "/home/jetson/ultralytics/ultralytics/yolo11n.engine"
        )
        self.declare_parameter("device", "0")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_detections", 30)
        # 类别白名单，逗号分隔的字符串；空串 = 全部 COCO 类别。
        # 用字符串而不是 string_array，因为 launch 的 LaunchConfiguration
        # 只能传字符串，用数组类型会和 launch 参数冲突。
        self.declare_parameter("class_filter", "")

        # 坐标系
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("tf_timeout", 0.05)

        # 反投影
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth", 0.25)
        self.declare_parameter("max_depth", 12.0)
        self.declare_parameter("roi_ratio", 0.6)
        self.declare_parameter("depth_percentile", 20.0)
        self.declare_parameter("depth_tolerance", 0.35)
        self.declare_parameter("min_valid_px", 12)

        # 其它
        self.declare_parameter("publish_debug_image", True)
        # 推理限速（Hz）。0 = 不限速（有多少帧处理多少帧）。
        #
        # 为什么需要这个：相机 30 Hz 时推理线程会跑满，实测占 38% CPU。
        # 但语义建图完全不需要这个频率 —— 物体是静止的，观测几次就够，
        # 而 RTAB-Map 自己的回环检测也只看 2 Hz。在 CPU 紧张的设备上
        # （比如 Orin NX 同时跑 VO + RTAB-Map + 桌面），把推理限到 5 Hz
        # 能省下约 25% CPU，代价只是位置融合的样本数少一些，精度几乎不变。
        self.declare_parameter("process_rate", 0.0)
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("sync_queue", 10)

        gp = self.get_parameter
        self.model_path = gp("model_path").value
        self.device = gp("device").value
        self.conf_thr = float(gp("conf_threshold").value)
        self.iou_thr = float(gp("iou_threshold").value)
        self.max_det = int(gp("max_detections").value)
        self.class_filter = [
            c.strip() for c in str(gp("class_filter").value).split(",") if c.strip()
        ]
        self.target_frame = gp("target_frame").value
        self.camera_frame = gp("camera_frame").value
        self.use_latest_tf = bool(gp("use_latest_tf").value)
        self.tf_timeout = float(gp("tf_timeout").value)
        self.depth_scale = float(gp("depth_scale").value)
        self.min_depth = float(gp("min_depth").value)
        self.max_depth = float(gp("max_depth").value)
        self.roi_ratio = float(gp("roi_ratio").value)
        self.depth_pct = float(gp("depth_percentile").value)
        self.depth_tol = float(gp("depth_tolerance").value)
        self.min_valid_px = int(gp("min_valid_px").value)
        self.publish_debug = bool(gp("publish_debug_image").value)
        self.publish_colored_cloud = bool(gp("publish_colored_cloud").value)
        self.cloud_stride = max(1, int(gp("colored_cloud_stride").value))
        self.publish_label_markers = bool(gp("publish_label_markers").value)

        # ------------------------- TF -------------------------
        self.tf_buffer = Buffer()
        # spin_thread=True: 让 TF 监听在独立线程里跑, 不被推理阻塞
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ------------------------- 模型 -------------------------
        self.model = self._load_model()
        self.model_names = dict(self.model.names)
        self.get_logger().info(f"模型加载完成, 共 {len(self.model_names)} 个类别")

        # ------------------------- 发布/订阅 -------------------------
        self.bridge = CvBridge()
        self.pub_objects = self.create_publisher(
            SemanticObjectArray, gp("objects_topic").value, 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, gp("markers_topic").value, 10
        )
        self.pub_label_markers = self.create_publisher(
            MarkerArray, gp("label_markers_topic").value, 2
        )
        self.pub_debug = self.create_publisher(
            Image, gp("debug_image_topic").value, 2
        )
        self.pub_colored_cloud = self.create_publisher(
            PointCloud2, gp("colored_cloud_topic").value, 2
        )

        # 图像话题必须用 sensor QoS(best effort), 否则和 realsense 驱动不匹配。
        # 注意 ROS 2 的 message_filters.Subscriber 签名是
        #     Subscriber(node, msg_type, topic, qos_profile=...)
        # 第一个参数必须是节点本身; 漏掉它会在内部 args[2] 处抛 IndexError,
        # 而且订阅根本不会被创建（表现为话题永远没有数据）。
        self.sub_rgb = Subscriber(
            self, Image, gp("rgb_topic").value, qos_profile=qos_profile_sensor_data
        )
        self.sub_depth = Subscriber(
            self, Image, gp("depth_topic").value, qos_profile=qos_profile_sensor_data
        )
        self.sub_info = Subscriber(
            self,
            CameraInfo,
            gp("camera_info_topic").value,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_info],
            queue_size=int(gp("sync_queue").value),
            slop=float(gp("sync_slop").value),
        )
        self.sync.registerCallback(self._on_synced)

        # ------------------------- 邮箱 + 推理线程 -------------------------
        self._lock = threading.Lock()
        self._mailbox = None
        self._mailbox_event = threading.Event()
        self._running = True
        self.process_rate = float(gp("process_rate").value)
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        # 统计
        self._frame_seq = 0
        self._processed = 0
        self._dropped = 0
        self._rate_limited = 0
        self._last_log = time.time()
        self._last_tf_warn = 0.0

        rate_desc = "不限速" if self.process_rate <= 0.0 else f"限速 {self.process_rate:g} Hz"
        self.get_logger().info(
            f"semantic_detector 就绪: target_frame={self.target_frame}, "
            f"camera_frame={self.camera_frame}, 推理{rate_desc}"
        )

    # ==================================================================
    # 模型加载
    # ==================================================================
    def _load_model(self):
        """按优先级加载模型，并在失败时明确报错。

        为什么要先检查文件是否存在：
            ultralytics 的 YOLO("xxx/yolo11n.pt") 在文件不存在时**不会**报错，
            而是把文件名当模型名，从 GitHub 自动下载到指定路径。这会在答辩
            现场因为断网而静默失败，也会让"我明明指定了 engine"变成悄悄用
            .pt 跑（FPS 从 52 掉到 35）。所以这里显式判断，宁可启动失败。
        """
        from ultralytics import YOLO  # 延迟导入，缩短非推理场景的启动时间

        candidates = [self.model_path]
        if self.model_path.endswith(".engine"):
            # TensorRT engine 与 TensorRT/驱动版本强绑定，换设备或升级
            # JetPack 后可能失效，此时回退到同名的 .pt
            candidates.append(self.model_path[: -len(".engine")] + ".pt")

        errors = []
        for path in candidates:
            if not os.path.isfile(path):
                self.get_logger().warn(f"模型文件不存在, 跳过: {path}")
                errors.append(f"{path}: 文件不存在")
                continue
            try:
                self.get_logger().info(f"正在加载模型: {path}")
                model = YOLO(path, task="detect")
                self.get_logger().info(f"模型加载成功: {path}")
                return model
            except Exception as exc:
                self.get_logger().warn(
                    f"加载 {path} 失败: {type(exc).__name__}: {exc}"
                )
                errors.append(f"{path}: {type(exc).__name__}: {exc}")

        raise RuntimeError(
            "没有可用的模型。候选路径全部失败:\n  "
            + "\n  ".join(errors)
            + "\n请检查 model_path 参数，或确认模型文件已正确部署。"
        )

    # ==================================================================
    # 同步回调：只做一件事 —— 把最新一帧塞进邮箱
    # ==================================================================
    def _on_synced(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        with self._lock:
            if self._mailbox is not None:
                self._dropped += 1
            self._mailbox = (rgb_msg, depth_msg, info_msg)
        self._mailbox_event.set()

    # ==================================================================
    # 推理线程
    # ==================================================================
    def _inference_loop(self) -> None:
        period = (1.0 / self.process_rate) if self.process_rate > 0.0 else 0.0
        next_allowed = 0.0
        while self._running:
            if not self._mailbox_event.wait(timeout=0.5):
                continue
            self._mailbox_event.clear()
            now = time.time()

            # 限速期内：丢弃这一帧。物体是静止的，少看几帧不影响结果，
            # 但省下的 CPU 对同时运行的视觉里程计是实打实的。
            if period > 0.0 and now < next_allowed:
                with self._lock:
                    self._mailbox = None
                self._rate_limited += 1
                continue

            with self._lock:
                packet = self._mailbox
                self._mailbox = None
            if packet is None:
                continue
            next_allowed = now + period
            try:
                self._process(*packet)
            except Exception as exc:  # 单帧失败不能拖垮整个节点
                self.get_logger().error(
                    f"处理帧失败: {type(exc).__name__}: {exc}", throttle_duration_sec=5.0
                )

    # ==================================================================
    # 核心：一帧的完整处理
    # ==================================================================
    def _process(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        t_start = time.time()

        # --- 1. 图像解码 ---
        # 用 bgr8 让 cv_bridge 统一处理 rgb8/bgr8, 避免通道顺序错误
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_m = depth_to_meters(depth_raw, depth_msg.encoding, self.depth_scale)

        if depth_m.shape[:2] != rgb.shape[:2]:
            self.get_logger().error(
                f"深度图 {depth_m.shape[:2]} 与彩色图 {rgb.shape[:2]} 尺寸不一致, "
                "请确认 align_depth.enable:=true",
                throttle_duration_sec=10.0,
            )
            return

        # --- 2. 内参 ---
        k = info_msg.k
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error("相机内参非法 (fx/fy <= 0)", throttle_duration_sec=10.0)
            return

        # --- 3. YOLO 推理 ---
        t_inf = time.time()
        results = self.model.predict(
            rgb,
            conf=self.conf_thr,
            iou=self.iou_thr,
            max_det=self.max_det,
            device=self.device,
            verbose=False,
        )
        inference_ms = (time.time() - t_inf) * 1000.0

        # --- 4. 取一次 TF 变换, 本帧所有物体共用 ---
        T = self._lookup_camera_to_target(rgb_msg.header.stamp)

        # --- 5. 逐框反投影 ---
        objects = []
        cloud_regions = []
        debug = rgb.copy() if self.publish_debug else None

        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy().astype(int)

            for (x0, y0, x1, y1), conf, cid in zip(xyxy, confs, clss):
                cname = self.model_names.get(int(cid), str(cid))
                if self.class_filter and cname not in self.class_filter:
                    continue

                res = backproject_bbox(
                    depth_m,
                    (float(x0), float(y0), float(x1), float(y1)),
                    fx,
                    fy,
                    cx,
                    cy,
                    roi_ratio=self.roi_ratio,
                    percentile=self.depth_pct,
                    depth_tolerance=self.depth_tol,
                    min_depth=self.min_depth,
                    max_depth=self.max_depth,
                    min_valid_px=self.min_valid_px,
                )
                if res is None:
                    # 深度无效：物体太远 / 反光 / 被遮挡。仍然画框提示，但不发布 3D
                    if debug is not None:
                        cv2.rectangle(
                            debug,
                            (int(x0), int(y0)),
                            (int(x1), int(y1)),
                            (128, 128, 128),
                            1,
                        )
                        cv2.putText(
                            debug,
                            f"{cname} {conf:.2f} no-depth",
                            (int(x0), max(12, int(y0) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (128, 128, 128),
                            1,
                        )
                    continue

                obj = SemanticObject()
                obj.header = Header(
                    stamp=rgb_msg.header.stamp, frame_id=self.target_frame
                )
                obj.class_name = cname
                obj.class_id = int(cid)
                obj.confidence = float(conf)
                obj.instance_id = -1
                obj.observation_count = 1
                obj.fused_confidence = float(conf)
                obj.bbox = [float(x0), float(y0), float(x1), float(y1)]

                # 相机系 -> 目标系
                #
                # 必须 position / size / yaw 三者一起变换，否则消息内部会出现
                # "位置在 map 系、尺寸和朝向还在相机光学系"的坐标系不一致。
                # 相机光学系是 (x 右, y 下, z 前)，与 map/base_link 的
                # (x 前, y 左, z 上) 差一个固定旋转，不处理的话 RViz 里画的
                # 包围盒会朝向错误的方向。
                R = T[:3, :3]
                p_cam = res.position
                p_tgt = R @ p_cam + T[:3, 3]
                obj.position = Point(
                    x=float(p_tgt[0]), y=float(p_tgt[1]), z=float(p_tgt[2])
                )

                # 尺寸：相机系下的轴对齐包围盒 (sx, sy, sz) 经旋转后，在目标
                # 各轴上的等效极差就是 |R| @ s（对旋转矩阵取绝对值）。
                size_tgt = np.abs(R) @ res.size
                obj.size = Vector3(
                    x=float(size_tgt[0]), y=float(size_tgt[1]), z=float(size_tgt[2])
                )

                # 朝向：把相机 x-y 平面内的主方向旋到目标系，再投影到目标
                # x-y 平面取角度。
                d_cam = np.array(
                    [math.cos(res.yaw), math.sin(res.yaw), 0.0], dtype=np.float64
                )
                d_tgt = R @ d_cam
                obj.yaw = float(math.atan2(d_tgt[1], d_tgt[0]))

                obj.distance = float(res.distance)
                obj.points_used = int(res.points_used)
                obj.rtabmap_node_id = -1
                objects.append(obj)
                cloud_regions.append((
                    float(x0), float(y0), float(x1), float(y1),
                    float(res.position[2]), int(cid),
                ))

                if debug is not None:
                    self._draw_detection(debug, obj, res, (int(x0), int(y0), int(x1), int(y1)))

        # --- 6. 发布语义点云 ---
        # 只把检测框内、且深度接近物体前景的像素发布出来；每一类使用统一颜色。
        # 该点云保留在相机光学坐标系，RViz 会通过 TF 自动显示到 map 中。
        if self.publish_colored_cloud:
            self._publish_colored_cloud(
                rgb, depth_m, rgb_msg.header.stamp,
                fx, fy, cx, cy, cloud_regions,
            )

        # --- 7. 发布 ---
        arr = SemanticObjectArray()
        arr.header = Header(stamp=rgb_msg.header.stamp, frame_id=self.target_frame)
        self._frame_seq += 1
        arr.frame_seq = self._frame_seq
        arr.inference_ms = float(inference_ms)
        arr.objects = objects
        self.pub_objects.publish(arr)

        if objects:
            self.pub_markers.publish(self._build_markers(objects, rgb_msg.header.stamp))

        if debug is not None:
            hdr = Header(stamp=rgb_msg.header.stamp, frame_id=rgb_msg.header.frame_id)
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8", header=hdr))

        # --- 8. 发布文字标签（每个物体头顶贴类别名） ---
        if self.publish_label_markers:
            self._publish_label_markers(objects, rgb_msg.header.stamp)

        # --- 7. 统计日志 ---
        self._processed += 1
        now = time.time()
        if now - self._last_log >= 10.0:
            dt = now - self._last_log
            rate_note = (
                f" | 限速丢弃 {self._rate_limited}" if self.process_rate > 0.0 else ""
            )
            self.get_logger().info(
                f"处理 {self._processed / dt:.1f} Hz | 丢帧 {self._dropped}{rate_note} | "
                f"推理 {inference_ms:.1f} ms | 本帧物体 {len(objects)} | "
                f"总耗时 {(now - t_start) * 1000:.1f} ms"
            )
            self._processed = 0
            self._dropped = 0
            self._rate_limited = 0
            self._last_log = now

    def _publish_colored_cloud(
        self, rgb, depth_m, stamp, fx, fy, cx, cy, regions
    ) -> None:
        """发布当前帧的物体点云；不同类别使用统一颜色。

        背景点云不在这里重复发布，RViz 继续使用 RTAB-Map 的原始点云作为
        背景，这样既清楚又不会把 Jetson 的 CPU 浪费在复制整帧灰色点云上。

        ⚠ 性能说明（务必保留）：本函数早期版本用 Python 双层循环遍历整幅
        深度图（640x480、stride=2 共 76800 次迭代），实测单帧耗时约 350 ms，
        加上推理 40 ms 后语义节点被压到 2.4 Hz（日志：处理 2.4 Hz、总耗时
        400 ms），远低于配置的 5 Hz。现改为纯 numpy 向量化，每帧只做常数次
        数组运算，实测单帧约 5 ms。
        """
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(stamp=stamp, frame_id=self.camera_frame)

        def _publish(points: np.ndarray) -> None:
            self.pub_colored_cloud.publish(
                point_cloud2.create_cloud(header, fields, points)
            )

        empty = np.empty((0, 4), dtype=np.float32)
        if not regions:
            # 清空上一帧的彩色点云，避免目标消失后 RViz 还保留旧点。
            _publish(empty)
            return

        h, w = depth_m.shape[:2]
        stride = self.cloud_stride

        # --- 1. 稀疏采样网格：一次取出深度与像素坐标 ---
        z = np.asarray(depth_m[::stride, ::stride], dtype=np.float32)
        vs = np.arange(0, h, stride, dtype=np.float32)
        us = np.arange(0, w, stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)  # 均为 (len(vs), len(us))

        # --- 2. 有效深度掩码 ---
        valid = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        if not valid.any():
            _publish(empty)
            return

        # --- 3. 像素归属：命中检测框且深度落在物体前景容差内 ---
        # labels 取值 0=背景（不发布），i+1 = 第 i 个 region。
        # 与旧实现保持一致：同一像素命中多个框时取第一个（labels == 0 保证）。
        labels = np.zeros(z.shape, dtype=np.int32)
        for i, (x0, y0, x1, y1, z0, _cid) in enumerate(regions, start=1):
            hit = (
                valid
                & (labels == 0)
                & (uu >= x0) & (uu <= x1)
                & (vv >= y0) & (vv <= y1)
                & (np.abs(z - z0) <= self.depth_tol)
            )
            labels[hit] = i

        sel = labels > 0
        if not sel.any():
            _publish(empty)
            return

        # --- 4. 反投影到相机光学坐标系 ---
        zz = z[sel]
        xx = (uu[sel] - cx) * zz / fx
        yy = (vv[sel] - cy) * zz / fy

        # --- 5. 按类别着色，打包为 RGB8 浮点 ---
        # 同一类别的颜色由 class_color 决定，与检测框 marker 保持一致。
        # ⚠ 字节序必须与 PointCloud2 的 rgb 约定一致：0x00RRGGBB，
        #   即 [23:16]=R、[15:8]=G、[7:0]=B。写反会导致红蓝互换。
        lut = np.zeros((len(regions) + 1, 4), dtype=np.uint32)
        for i, region in enumerate(regions, start=1):
            c = class_color(int(region[5]))
            lut[i, 0] = min(255, max(0, int(round(c.r * 255.0))))  # R -> [7:0]
            lut[i, 1] = min(255, max(0, int(round(c.g * 255.0))))  # G -> [15:8]
            lut[i, 2] = min(255, max(0, int(round(c.b * 255.0))))  # B -> [23:16]
            lut[i, 3] = 255                                        # A -> [31:24]
        px = lut[labels[sel]]
        rgba = (px[:, 3] << 24) | (px[:, 2] << 16) | (px[:, 1] << 8) | px[:, 0]
        rgb_float = rgba.astype(np.uint32).view(np.float32)

        points = np.empty((zz.shape[0], 4), dtype=np.float32)
        points[:, 0] = xx
        points[:, 1] = yy
        points[:, 2] = zz
        points[:, 3] = rgb_float
        _publish(points)

    def _publish_label_markers(self, objects, stamp) -> None:
        """把每个物体的类别名作为 TEXT_VIEW_FACING 标记，悬浮在物体上方。

        使用 TEXT_VIEW_FACING 让文字始终朝向相机；位置 = 物体顶部 + 0.25 m，
        这样无论怎样缩放 RViz，标签都不会贴住点云表面。
        """
        arr = MarkerArray()
        # 先发 DELETEALL，避免上一帧残留
        clear = Marker(action=Marker.DELETEALL)
        arr.markers.append(clear)
        if not objects:
            self.pub_label_markers.publish(arr)
            return
        for i, obj in enumerate(objects):
            top_z = float(obj.position.z) + max(0.05, 0.5 * float(obj.size.z)) + 0.25
            text = Marker()
            text.header = Header(stamp=stamp, frame_id=obj.header.frame_id or self.target_frame)
            text.ns = "semantic_label"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=obj.position.x, y=obj.position.y, z=top_z)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"{obj.class_name} {obj.confidence:.2f}"
            arr.markers.append(text)
        self.pub_label_markers.publish(arr)

    # ==================================================================
    # 辅助
    # ==================================================================
    def _lookup_camera_to_target(self, stamp) -> np.ndarray:
        """返回 camera_frame -> target_frame 的 4x4 齐次矩阵。

        TF 不可用时退化为单位阵（等于把点留在相机系），并打印告警。
        这样即使 PX4 没启动也能看到 /semantics/objects 有数据出来。
        """
        when = Time() if self.use_latest_tf else Time.from_msg(stamp)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                when,
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            now = time.time()
            if now - self._last_tf_warn > 10.0:
                self.get_logger().warn(
                    f"TF {self.target_frame} <- {self.camera_frame} 不可用 "
                    f"({type(exc).__name__}): 物体位置将留在相机坐标系。"
                    f"检查 PX4/MAVROS 是否在运行, 或把 target_frame 改成立即可用的坐标系。"
                )
                self._last_tf_warn = now
            return np.eye(4)

        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quaternion_to_matrix(rot.x, rot.y, rot.z, rot.w)
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def _draw_detection(self, img, obj: SemanticObject, res, xyxy) -> None:
        x0, y0, x1, y1 = xyxy
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        label = f"{obj.class_name} {obj.confidence:.2f} {obj.distance:.2f}m"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x0, max(0, y0 - th - 6)), (x0 + tw + 4, y0), (0, 255, 0), -1)
        cv2.putText(
            img, label, (x0 + 2, max(th, y0 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
        )
        # 画反投影质心，用来直观检查 2D 框和 3D 点的对应关系
        cv2.circle(img, (int((x0 + x1) / 2), int((y0 + y1) / 2)), 3, (0, 255, 255), -1)

    def _build_markers(self, objects, stamp) -> MarkerArray:
        """把语义物体画成 RViz 标记：球体 + 速度矢量(朝向) + 文字标签。"""
        arr = MarkerArray()
        for i, obj in enumerate(objects):
            c = class_color(obj.class_id)
            ns = obj.class_name

            sphere = Marker()
            sphere.header = Header(stamp=stamp, frame_id=self.target_frame)
            sphere.ns = ns
            sphere.id = i * 3 + 0
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = obj.position
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = max(0.08, float(obj.size.x))
            sphere.scale.y = max(0.08, float(obj.size.y))
            sphere.scale.z = max(0.08, float(obj.size.z))
            sphere.color = c
            sphere.lifetime = Duration(seconds=1.0).to_msg()
            arr.markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = ns + "_label"
            text.id = i * 3 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=obj.position.x, y=obj.position.y, z=obj.position.z + 0.35
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"{obj.class_name} #{obj.class_id} {obj.confidence:.2f}"
            text.lifetime = Duration(seconds=1.0).to_msg()
            arr.markers.append(text)

            arrow = Marker()
            arrow.header = sphere.header
            arrow.ns = ns + "_yaw"
            arrow.id = i * 3 + 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position = obj.position
            arrow.pose.orientation.z = math.sin(obj.yaw * 0.5)
            arrow.pose.orientation.w = math.cos(obj.yaw * 0.5)
            arrow.scale.x = 0.4
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color = c
            arrow.lifetime = Duration(seconds=1.0).to_msg()
            arr.markers.append(arrow)

        return arr

    def destroy_node(self) -> bool:
        self._running = False
        self._mailbox_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticDetector()
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
