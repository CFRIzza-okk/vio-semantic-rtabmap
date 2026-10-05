#!/usr/bin/env python3
"""semantic_mapper: 把逐帧检测融合成持久的语义地图（Phase 2）

职责
----
detector_node 每帧给出的是彼此独立的观测，没有任何跨帧身份。本节点负责：

  1. 数据关联：判断"这一帧的杯子和上一帧的杯子是不是同一个"（见 association.py）
  2. 状态融合：多帧位置/尺寸/置信度融合，压低单帧深度噪声
  3. 语义地图：维护一份带全局实例 ID 的物体列表，可查询、可持久化
  4. 对接 RTAB-Map：把物体作为 LandmarkDetections 发布，让 RTAB-Map
     把它们作为地标因子纳入图优化

话题与服务
----------
    订阅 /semantics/objects            (SemanticObjectArray, instance_id 未填)
    发布 /semantics/objects_tracked    (SemanticObjectArray, 已填 instance_id)
    发布 /semantics/map_markers        (MarkerArray, 语义地图可视化)
    发布 /semantics/landmarks          (rtabmap_msgs/LandmarkDetections)
    服务 /semantic_mapper/query        (QuerySemanticMap, 供导航节点查询)
"""

import json
import math
import os
import time
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from semantic_rtabmap.association import Associator, AssociatorConfig, Detection, Track
from semantic_rtabmap.detector_node import quaternion_to_matrix
from semantic_rtabmap.landmark_quality import (
    DEFAULT_CONFIG as LANDMARK_QUALITY_DEFAULT,
    LandmarkQualityConfig,
    landmark_covariance,
)
from semantic_rtabmap_msgs.msg import SemanticObject, SemanticObjectArray
from semantic_rtabmap_msgs.srv import QuerySemanticMap

try:
    from rtabmap_msgs.msg import LandmarkDetection, LandmarkDetections

    _HAS_RTABMAP = True
except ImportError:  # 允许在没有 rtabmap_msgs 的环境里单独跑语义层
    _HAS_RTABMAP = False


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


# RTAB-Map 的 landmark 订阅端是 KeepLast(1)+RELIABLE+VOLATILE。
# 必须用 RELIABLE 发布，BEST_EFFORT 会导致对方完全收不到消息且不报错。
LANDMARK_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def box_area(bbox) -> float:
    """检测框面积（像素^2）。退化框返回 0，调用方据此判定"算不出密度"。"""
    try:
        x0, y0, x1, y1 = (float(v) for v in bbox)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, x1 - x0) * max(0.0, y1 - y0)


def class_color(class_id: int, alpha: float = 1.0) -> ColorRGBA:
    h = (int(class_id) * 2654435761) & 0xFFFFFFFF
    r = ((h >> 16) & 0xFF) / 255.0
    g = ((h >> 8) & 0xFF) / 255.0
    b = (h & 0xFF) / 255.0
    return ColorRGBA(r=0.35 + 0.65 * r, g=0.35 + 0.65 * g, b=0.35 + 0.65 * b, a=alpha)


class SemanticMapper(Node):
    def __init__(self) -> None:
        super().__init__("semantic_mapper")

        # ------------------------- 参数 -------------------------
        self.declare_parameter("objects_topic", "/semantics/objects")
        self.declare_parameter("objects_tracked_topic", "/semantics/objects_tracked")
        self.declare_parameter("map_markers_topic", "/semantics/map_markers")
        self.declare_parameter("landmarks_topic", "/semantics/landmarks")
        self.declare_parameter("query_service", "/semantic_mapper/query")

        # 数据关联
        self.declare_parameter("assoc_base_gate", 0.6)
        self.declare_parameter("assoc_size_factor", 0.75)
        self.declare_parameter("assoc_max_gate", 3.0)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("prune_age", 30.0)
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("use_hungarian", True)

        # 输出
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_landmarks", True)
        # "camera" = 按 RTAB-Map 官方约定发布在相机光学系（推荐）
        # "map"    = 直接发布 map 系坐标（RTAB-Map 会多做一次运动补偿）
        self.declare_parameter("landmarks_frame_mode", "camera")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        # RTAB-Map 侧订阅的话题名是**硬编码**的 `landmark_detections`：
        #   - 没有 landmark_topic 之类的 ros 参数可以改（实测 ros2 param list 只有
        #     landmark_linear_variance / landmark_angular_variance /
        #     Optimizer/LandmarksIgnored）；
        #   - rtabmap.launch.py 的 tag_topic 只 remap apriltag 的 tag_detections，
        #     管不到 rtabmap_msgs/LandmarkDetections。
        # 所以除了本层自己的 /semantics/landmarks，再往下面这些话题各发一份**同样**的消息。
        #
        # 为什么是两个而不是一个：订阅名是**相对名**，会跟着节点命名空间走。
        # rtabmap.launch.py 的 namespace 默认值是 'rtabmap'（见该文件第 451 行），
        # 所以走官方 launch 起时，实际订阅的是 /rtabmap/landmark_detections；
        # 而直接 ros2 run rtabmap_slam rtabmap 起（没有命名空间）时是
        # /landmark_detections。两个都发就不必依赖别人的启动方式，代价极小。
        # 逗号分隔；置空字符串 = 关闭这一路，只保留 /semantics/landmarks。
        self.declare_parameter(
            "rtabmap_landmarks_topics",
            "/rtabmap/landmark_detections,/landmark_detections",
        )
        self.declare_parameter("marker_publish_period", 1.0)
        self.declare_parameter("log_period", 15.0)

        # ---------------- 语义质量自适应的地标协方差（核心创新）----------------
        # "uniform" = 发全零协方差，退回 RTAB-Map 的全局 landmark_linear_variance
        #             —— **默认值**，与改动前的行为完全一致。
        # "quality" = 每个地标按自身语义质量/材质类别计算协方差（本方案）。
        #             ⚠ 尚未在真实序列上量化过对 ATE/RPE 的影响。
        #               在跑完 quality vs uniform 对照实验之前，不要把它设为默认。
        self.declare_parameter("landmark_covariance_mode", "uniform")
        self.declare_parameter("quality_base_sigma", LANDMARK_QUALITY_DEFAULT.base_sigma)
        self.declare_parameter("quality_min_sigma", LANDMARK_QUALITY_DEFAULT.min_sigma)
        self.declare_parameter("quality_max_sigma", LANDMARK_QUALITY_DEFAULT.max_sigma)
        self.declare_parameter(
            "quality_angular_variance", LANDMARK_QUALITY_DEFAULT.angular_variance
        )
        self.declare_parameter(
            "quality_evidence_tau", LANDMARK_QUALITY_DEFAULT.evidence_tau
        )
        self.declare_parameter(
            "quality_density_ref", LANDMARK_QUALITY_DEFAULT.density_ref
        )
        self.declare_parameter(
            "quality_penetrable_density", LANDMARK_QUALITY_DEFAULT.penetrable_density
        )
        self.declare_parameter(
            "quality_opaque_density", LANDMARK_QUALITY_DEFAULT.opaque_density
        )
        self.declare_parameter(
            "quality_gain_penetrable", LANDMARK_QUALITY_DEFAULT.gain_penetrable
        )
        self.declare_parameter(
            "quality_gain_uncertain", LANDMARK_QUALITY_DEFAULT.gain_uncertain
        )

        # 持久化（默认关闭：跨会话复用要求 map 原点完全一致）
        self.declare_parameter("persistence_path", "")
        self.declare_parameter("load_on_startup", False)

        gp = self.get_parameter
        self.map_frame = gp("map_frame").value
        self.publish_landmarks = bool(gp("publish_landmarks").value)
        self.landmarks_frame_mode = str(gp("landmarks_frame_mode").value)
        self.camera_frame = str(gp("camera_frame").value)
        self.rtabmap_landmarks_topics = [
            t.strip()
            for t in str(gp("rtabmap_landmarks_topics").value).split(",")
            if t.strip()
        ]
        self.persistence_path = str(gp("persistence_path").value)
        self.marker_period = float(gp("marker_publish_period").value)
        self.log_period = float(gp("log_period").value)
        self._last_tf_warn = 0.0

        self.covariance_mode = str(gp("landmark_covariance_mode").value)
        self.quality_cfg = LandmarkQualityConfig(
            base_sigma=float(gp("quality_base_sigma").value),
            min_sigma=float(gp("quality_min_sigma").value),
            max_sigma=float(gp("quality_max_sigma").value),
            angular_variance=float(gp("quality_angular_variance").value),
            evidence_tau=float(gp("quality_evidence_tau").value),
            density_ref=float(gp("quality_density_ref").value),
            penetrable_density=float(gp("quality_penetrable_density").value),
            opaque_density=float(gp("quality_opaque_density").value),
            gain_penetrable=float(gp("quality_gain_penetrable").value),
            gain_uncertain=float(gp("quality_gain_uncertain").value),
        )
        if self.covariance_mode not in ("quality", "uniform"):
            self.get_logger().warn(
                f"未知的 landmark_covariance_mode='{self.covariance_mode}'，"
                "已退回 'uniform'"
            )
            self.covariance_mode = "uniform"

        # RTAB-Map 的地标位姿参考系是相机光学系，需要 TF 做坐标回变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        cfg = AssociatorConfig(
            base_gate=float(gp("assoc_base_gate").value),
            size_factor=float(gp("assoc_size_factor").value),
            max_gate=float(gp("assoc_max_gate").value),
            min_observations=int(gp("min_observations").value),
            prune_age=float(gp("prune_age").value),
            min_confidence=float(gp("min_confidence").value),
            use_hungarian=bool(gp("use_hungarian").value),
        )
        self.cfg = cfg
        self.associator = Associator(cfg)

        if self.persistence_path and bool(gp("load_on_startup").value):
            self._load()

        # ------------------------- 接口 -------------------------
        # 【顺序很重要，不要调整】
        # TransformListener(spin_thread=True) 在上面已经起了一个**独立自旋线程**，
        # 所以 create_subscription 一旦执行，_on_objects 就可能**在另一个线程上**
        # 被调用。如果发布者晚于订阅创建，回调就会撞上尚未赋值的属性，抛
        # AttributeError —— 而且这个异常会顺着 TF 监听线程冒出去，把 TF 监听打死。
        #
        # 实测必现条件：节点启动时检测话题已经在高频发布（例如重起语义层、
        # 或另起一个观测用 mapper）。此时消息会在几毫秒内到达。
        # 因此：所有发布者一律先于订阅建立。
        self.pub_tracked = self.create_publisher(
            SemanticObjectArray, gp("objects_tracked_topic").value, 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, gp("map_markers_topic").value, 10
        )
        if self.publish_landmarks and _HAS_RTABMAP:
            self.pub_landmarks = self.create_publisher(
                LandmarkDetections, gp("landmarks_topic").value, LANDMARK_QOS
            )
            # 其余几路：RTAB-Map 硬编码订阅的话题（见参数声明处的说明）。
            self.pub_landmarks_rtabmap = []
            primary = gp("landmarks_topic").value
            for topic in self.rtabmap_landmarks_topics:
                if topic == primary:
                    continue
                self.pub_landmarks_rtabmap.append(
                    self.create_publisher(LandmarkDetections, topic, LANDMARK_QOS)
                )
        else:
            self.pub_landmarks = None
            self.pub_landmarks_rtabmap = []
            if self.publish_landmarks and not _HAS_RTABMAP:
                self.get_logger().warn("未找到 rtabmap_msgs, 地标发布功能已禁用")

        # 所有发布者就绪之后，最后才建立订阅（原因见上面的顺序说明）
        self.create_subscription(
            SemanticObjectArray,
            gp("objects_topic").value,
            self._on_objects,
            10,
        )

        self.srv = self.create_service(
            QuerySemanticMap, gp("query_service").value, self._on_query
        )

        # 定时任务：可视化刷新 / 统计日志 / 持久化
        self.create_timer(self.marker_period, self._publish_map_markers)
        self.create_timer(self.log_period, self._log_stats)
        if self.persistence_path:
            self.create_timer(30.0, self._save)

        self._last_stamp = 0.0
        self._map_frame_origin = self.map_frame
        self.get_logger().info(
            f"semantic_mapper 就绪: 关联门限 {cfg.base_gate}+{cfg.size_factor}*尺度, "
            f"确认需 {cfg.min_observations} 次观测, "
            f"地标发布 {'开' if self.pub_landmarks else '关'}"
        )

    # ==================================================================
    # 主回调
    # ==================================================================
    def _on_objects(self, msg: SemanticObjectArray) -> None:
        stamp = self._msg_time(msg)
        self._last_stamp = stamp

        detections = [
            Detection(
                class_name=o.class_name,
                class_id=o.class_id,
                confidence=float(o.confidence),
                position=np.array(
                    [o.position.x, o.position.y, o.position.z], dtype=np.float64
                ),
                size=np.array([o.size.x, o.size.y, o.size.z], dtype=np.float64),
                yaw=float(o.yaw),
                stamp=stamp,
                bbox=tuple(float(v) for v in o.bbox),
                points_used=int(o.points_used),
                distance=float(o.distance),
                density=(
                    int(o.points_used) / area
                    if (area := box_area(o.bbox)) > 1.0
                    else None
                ),
            )
            for o in msg.objects
        ]

        self.associator.update(detections, stamp)

        # 回填 instance_id 后转发，便于其它节点（含 RViz）看到关联结果
        out = SemanticObjectArray()
        out.header = msg.header
        out.frame_seq = msg.frame_seq
        out.inference_ms = msg.inference_ms
        out.objects = [self._fill(o) for o in msg.objects]
        self.pub_tracked.publish(out)

        if self.pub_landmarks:
            self._publish_landmarks(msg.header)

    def _msg_time(self, msg: SemanticObjectArray) -> float:
        t = msg.header.stamp
        return float(t.sec) + float(t.nanosec) * 1e-9

    # ==================================================================
    def _fill(self, obj: SemanticObject) -> SemanticObject:
        """给单帧检测对象填上关联到的实例信息。

        这里用的是"最近邻回填"：检测和轨迹已经在上一步融合过了，本步只是
        把检测消息里的字段补全，供外部节点和 RViz 显示。它不参与地图状态，
        所以即使偶尔回填到相邻的同类实例，也不会污染语义地图。
        """
        p = np.array([obj.position.x, obj.position.y, obj.position.z])
        best, best_d = None, 1e9
        for tr in self.associator.tracks.values():
            if tr.class_name != obj.class_name:
                continue
            d = float(np.linalg.norm(tr.position - p))
            gate = tr.gate_radius(self.cfg.base_gate, self.cfg.size_factor)
            if d < gate and d < best_d:
                best, best_d = tr, d
        if best is not None:
            obj.instance_id = best.instance_id
            obj.observation_count = best.observation_count
            obj.fused_confidence = best.confidence
            obj.rtabmap_node_id = best.rtabmap_node_id
        return obj

    # ==================================================================
    # RTAB-Map 地标接入
    # ==================================================================
    def _publish_landmarks(self, header_in: Header) -> None:
        """把确认的语义物体作为地标（landmark）发布给 RTAB-Map。

        以下四条结论来自对 rtabmap_ros/humble-devel 源码的核查
        （详见 ~/VIO/rtabmap_landmark_label_interfaces.md），每一条都踩过坑：

        1) 订阅名是硬编码的**相对名** `landmark_detections`，**没有** landmark_topic
           之类的参数；rtabmap.launch.py 的 tag_topic 只 remap apriltag 的
           tag_detections，管不到 rtabmap_msgs/LandmarkDetections。
           因为是相对名，实际话题名 = <rtabmap 节点的命名空间>/landmark_detections，
           而 rtabmap.launch.py 的 namespace 默认是 'rtabmap'，所以走官方 launch
           起的时候真正订阅的是 /rtabmap/landmark_detections。本节点因此往
           /rtabmap/landmark_detections 和 /landmark_detections 各发一份，
           对两种启动方式都成立（见 rtabmap_landmarks_topics 参数）。

        2) 订阅端 QoS 是 KeepLast(1)+RELIABLE+VOLATILE，**不是** SensorDataQoS。
           如果我们用 BEST_EFFORT 发布，会完全收不到消息且没有任何报错。
           所以这里显式指定 RELIABLE。

        3) header.frame_id 是位姿的参考系，RTAB-Map 会用
               lookupTransform(frame_id 参数(默认 base_link), header.frame_id, stamp)
           把它变换到 base_link，再做一次 base->odom 运动补偿。TF 查询失败时
           该 landmark 被**静默丢弃**（只有日志里的 UERROR）。
           因此默认按官方约定发布在**相机光学系**下（与 apriltag_ros 一致），
           让 RTAB-Map 走它自己的标准变换链；如果直接发布在 map 系，
           运动补偿会被重复施加一次。

        4) id 必须 >0 且全局稳定唯一 —— RTAB-Map 内部把它当作"负数节点 id"
           使用，没有任何外观/几何重识别或合并逻辑。同一物体换个 id 就会变成
           两个互不关联的地标顶点，地图重复且永远拿不到回环收益。id 还会被
           写进数据库，跨 session 必须保持一致。
           这里用 class_id * 100000 + instance_id 编码，既全局唯一，
           也能从 id 反解出类别，便于在 RTAB-Map 数据库里检索。
        """
        # 统一取一次 TF：map -> 相机光学系（用于把融合后的位置换回相机系）
        T_cam_from_map = None
        if self.landmarks_frame_mode == "camera":
            T_cam_from_map = self._lookup_map_to_camera(header_in.stamp)
            if T_cam_from_map is None:
                # 【关键】TF 拿不到时宁可不发，也**绝不**退回 map 帧发送。
                #
                # 退回 map 帧的代价实测过一次，非常惨：
                #   RTAB-Map 收到 header.frame_id="map" 的地标后，会尝试
                #   getTransform("base_link" <- "map")。里程计一旦丢失，rtabmap
                #   就不再广播 odom->base_link，TF 裂成两棵互不相连的树
                #   （map->odom 一棵、base_link->camera_link 一棵），这个查询
                #   必然失败并且**阻塞**。结果 rtabmap 的 Conversion 从健康的
                #   0.0015s 恶化到 3.6979s，整条建图流水线停摆，
                #   /rtabmap/cloud_map 再也不发布 —— RViz / Foxglove 里全空白。
                #
                # 少发一次地标只是少一次观测；退化成 map 帧却会把整条链路拖死。
                return

        msg = LandmarkDetections()
        frame = (
            self.camera_frame
            if (self.landmarks_frame_mode == "camera" and T_cam_from_map is not None)
            else self.map_frame
        )
        msg.header = Header(stamp=header_in.stamp, frame_id=frame)

        lands: List[LandmarkDetection] = []
        self._landmark_debug: List[dict] = []
        for tr in self.associator.confirmed_tracks():
            if T_cam_from_map is not None:
                p = T_cam_from_map[:3, :3] @ tr.position + T_cam_from_map[:3, 3]
            else:
                p = tr.position

            d = LandmarkDetection()
            d.header = msg.header
            # 该字段在 RTAB-Map 内部完全未被使用（源码里找不到读取点），
            # 只作为人类可读的调试名保留。
            d.landmark_frame_id = f"object_{tr.class_name}_{tr.instance_id}"
            d.id = int(tr.class_id) * 100000 + int(tr.instance_id)
            d.size = float(np.max(tr.size)) if tr.size.size else 0.0

            d.pose = PoseWithCovariance()
            d.pose.pose = Pose(
                position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                orientation=yaw_to_quaternion(tr.yaw),
            )

            # ---- 协方差：语义质量自适应（本方案的核心改动）----
            #
            # RTAB-Map 在 MsgConversion.cpp:1996-2003 只检查协方差的 (0,0)：
            #   · 若为 0/非有限 -> 用**全局** landmark_linear_variance 填充所有地标
            #   · 若为正有限 -> **原样采用**我们给的协方差
            #
            # 所以发非零协方差，就等于把"一个地标该有多可信"的决定权从
            # RTAB-Map 的全局常数手里拿回到语义层，按每个物体自身的
            # 置信度 / 等效独立观测数 / 深度支撑密度（材质）分别计算。
            #
            # 注意角方差：一旦我们发了非零协方差，全局 tag_angular_variance
            # 就**不再生效**，角方差必须由我们自己给，且必须 >= 9999，
            # 否则 RTAB-Map（OptimizerGTSAM.cpp:362-364）会改用
            # BetweenFactor<Pose3> 并要求地标朝向可信——而 YOLO 的 yaw 不可靠。
            if self.covariance_mode == "quality":
                cov, sigma, quality, material = landmark_covariance(
                    confidence=tr.confidence,
                    n_independent=tr.independent_observations,
                    density=tr.density,
                    cfg=self.quality_cfg,
                )
                d.pose.covariance = cov
            else:
                # 消融对照：退回原行为，由 RTAB-Map 全局参数填充
                sigma, quality, material = 0.0, 0.0, "uniform"
                d.pose.covariance = [0.0] * 36
            lands.append(d)

            self._landmark_debug.append(
                {
                    "instance_id": int(tr.instance_id),
                    "class_name": tr.class_name,
                    "sigma": sigma,
                    "quality": quality,
                    "material": material,
                    "density": tr.density,
                }
            )

        msg.landmarks = lands
        self.pub_landmarks.publish(msg)
        # 同一份消息再发一份给 RTAB-Map 硬编码订阅的那几个话题
        for pub in self.pub_landmarks_rtabmap:
            pub.publish(msg)

    def _lookup_map_to_camera(self, stamp) -> Optional[np.ndarray]:
        """取 map -> 相机光学系 的 4x4 变换；失败返回 None。

        先用检测时间戳精确查，查不到再退化成"取当前最新"，两步都失败才放弃。
        退化那一步是必需的，不是偷懒：
          RTAB-Map 的 map->odom 是按它自己的处理节奏发布的（实测延迟
          0.3~0.45s，tf_delay=0.05），而语义地标的 header.stamp 是图像时间戳。
          检测刚算完的那一刻，那个时刻的 map->odom 根本还没发布，
          精确查询必然抛 ExtrapolationException —— 实测每次都在退化，
          于是地标全部退化成 map 系发布，被 RTAB-Map 重复施加一次运动补偿。
        地标是静止物体，用"最新 TF"代替的误差 ≈ 相机在这段延迟内的位移
        （手持约 0.5m/s × 0.4s ≈ 0.2m），与下面 landmark_linear_variance
        设的 σ=0.2m 同量级，可以接受；总好过整条链路退化成 map 系。
        """
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import TransformException

        tf_msg = None
        last_exc: Optional[Exception] = None
        # Time() 无参 = 0 时刻 = tf2 约定里的"用缓冲区里最新的那个"
        for query_time in (Time.from_msg(stamp), Time()):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.map_frame,
                    query_time,
                    timeout=Duration(seconds=0.05),
                )
                break
            except TransformException as exc:
                last_exc = exc

        if tf_msg is None:
            now = time.time()
            if now - self._last_tf_warn > 10.0:
                self.get_logger().warn(
                    f"TF {self.camera_frame} <- {self.map_frame} 不可用 "
                    f"({type(last_exc).__name__}): 本次跳过地标发布。"
                    "（不退回 map 帧，否则会拖垮 RTAB-Map 的 Conversion 阶段）"
                )
                self._last_tf_warn = now
            return None

        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quaternion_to_matrix(rot.x, rot.y, rot.z, rot.w)
        T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    # ==================================================================
    # 可视化
    # ==================================================================
    def _publish_map_markers(self) -> None:
        """发布完整的语义地图标记。

        每次都先发一个 DELETEALL 再重发全集，这样被删除的物体会从 RViz
        里消失，不会留下幽灵标记。
        """
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        frame = self.map_frame
        now = self.get_clock().now().to_msg()

        for i, tr in enumerate(self.associator.confirmed_tracks()):
            c = class_color(tr.class_id)
            sx = max(0.1, float(tr.size[0])) if tr.size.size else 0.2
            sy = max(0.1, float(tr.size[1])) if tr.size.size >= 2 else 0.2
            sz = max(0.1, float(tr.size[2])) if tr.size.size >= 3 else 0.2

            # 立方体表示物体的估计尺寸和朝向
            box = Marker()
            box.header = Header(stamp=now, frame_id=frame)
            box.ns = "semantic_map"
            box.id = i * 3 + 0
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position = Point(
                x=float(tr.position[0]), y=float(tr.position[1]), z=float(tr.position[2])
            )
            box.pose.orientation = yaw_to_quaternion(tr.yaw)
            box.scale = Vector3(x=sx, y=sy, z=sz)
            box.color = class_color(tr.class_id, alpha=0.75)
            arr.markers.append(box)

            text = Marker()
            text.header = box.header
            text.ns = "semantic_label"
            text.id = i * 3 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=float(tr.position[0]),
                y=float(tr.position[1]),
                z=float(tr.position[2]) + 0.5 * sz + 0.25,
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.24
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = (
                f"{tr.class_name} #{tr.instance_id}\n"
                f"p={tr.confidence:.2f} n={tr.observation_count}"
            )
            arr.markers.append(text)

            # 从原点指向物体的连线，方便判断方位
            line = Marker()
            line.header = box.header
            line.ns = "semantic_link"
            line.id = i * 3 + 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.02
            line.color = class_color(tr.class_id, alpha=0.35)
            line.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(
                    x=float(tr.position[0]),
                    y=float(tr.position[1]),
                    z=float(tr.position[2]),
                ),
            ]
            arr.markers.append(line)

        self.pub_markers.publish(arr)

    # ==================================================================
    # 查询服务
    # ==================================================================
    def _on_query(self, request, response):
        origin = None
        if request.use_query_origin and request.max_distance > 0.0:
            origin = np.array(
                [request.query_origin.x, request.query_origin.y, request.query_origin.z]
            )

        results = []
        for tr in self.associator.confirmed_tracks():
            if request.class_name and tr.class_name != request.class_name:
                continue
            if request.instance_id > 0 and tr.instance_id != request.instance_id:
                continue
            if origin is not None:
                if float(np.linalg.norm(tr.position - origin)) > request.max_distance:
                    continue
            results.append(tr)

        if origin is not None:
            results.sort(key=lambda t: float(np.linalg.norm(t.position - origin)))

        response.objects = [self._track_to_msg(tr, self.map_frame) for tr in results]
        response.success = True
        if not results:
            response.message = (
                f"没有找到匹配的语义物体 (class='{request.class_name}', "
                f"instance_id={request.instance_id}, 当前地图共 "
                f"{len(self.associator.confirmed_tracks())} 个确认物体)"
            )
        else:
            response.message = f"找到 {len(results)} 个匹配物体"
        return response

    def _track_to_msg(self, tr: Track, frame: str) -> SemanticObject:
        o = SemanticObject()
        o.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id=frame
        )
        o.class_name = tr.class_name
        o.class_id = tr.class_id
        o.confidence = tr.best_confidence
        o.fused_confidence = tr.confidence
        o.instance_id = tr.instance_id
        o.observation_count = tr.observation_count
        o.position = Point(
            x=float(tr.position[0]), y=float(tr.position[1]), z=float(tr.position[2])
        )
        o.size = Vector3(
            x=float(tr.size[0]) if tr.size.size > 0 else 0.0,
            y=float(tr.size[1]) if tr.size.size > 1 else 0.0,
            z=float(tr.size[2]) if tr.size.size > 2 else 0.0,
        )
        o.yaw = float(tr.yaw)
        o.distance = float(np.linalg.norm(tr.position))
        o.rtabmap_node_id = tr.rtabmap_node_id
        # 语义约束质量：供 RViz / Foxglove / 论文分析脚本读取
        if self.covariance_mode == "quality":
            cov, sigma, quality, material = landmark_covariance(
                confidence=tr.confidence,
                n_independent=tr.independent_observations,
                density=tr.density,
                cfg=self.quality_cfg,
            )
            o.landmark_linear_variance = float(sigma * sigma)
            o.landmark_angular_variance = float(self.quality_cfg.angular_variance)
            o.semantic_quality = float(quality)
        return o

    # ==================================================================
    # 统计与持久化
    # ==================================================================
    def _log_stats(self) -> None:
        s = self.associator.stats
        conf = self.associator.confirmed_tracks()
        by_class: dict = {}
        for tr in conf:
            by_class[tr.class_name] = by_class.get(tr.class_name, 0) + 1
        summary = ", ".join(f"{k}x{v}" for k, v in sorted(by_class.items())) or "空"
        self.get_logger().info(
            f"语义地图: {len(conf)} 个确认物体 [{summary}] | "
            f"轨迹总数 {len(self.associator.tracks)} | "
            f"累计匹配 {s['matched']} 新建 {s['created']} 清理 {s['pruned']}"
        )
        # 地标约束质量：直接反映"哪些地标被当作强约束、哪些被弱化"，
        # 是消融实验（quality vs uniform）最直接的观测点。
        if self.covariance_mode == "quality" and conf:
            rows = []
            for tr in conf:
                _, sigma, q, material = landmark_covariance(
                    confidence=tr.confidence,
                    n_independent=tr.independent_observations,
                    density=tr.density,
                    cfg=self.quality_cfg,
                )
                rows.append(
                    f"{tr.class_name}#{tr.instance_id}"
                    f"(σ={sigma:.2f}m q={q:.2f} {material})"
                )
            self.get_logger().info("地标约束: " + " ".join(rows))

    def _save(self) -> None:
        if not self.persistence_path:
            return
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.persistence_path)), exist_ok=True)
            data = self.associator.to_dict()
            data["map_frame"] = self.map_frame
            data["saved_at"] = time.time()
            tmp = self.persistence_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self.persistence_path)  # 原子替换，避免写坏文件
        except Exception as exc:
            self.get_logger().error(f"保存语义地图失败: {type(exc).__name__}: {exc}")

    def _load(self) -> None:
        if not os.path.isfile(self.persistence_path):
            self.get_logger().info(f"语义地图文件不存在, 从空地图开始: {self.persistence_path}")
            return
        try:
            with open(self.persistence_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            saved_frame = data.get("map_frame")
            if saved_frame and saved_frame != self.map_frame:
                self.get_logger().warn(
                    f"已保存的地图坐标系是 '{saved_frame}', 当前是 '{self.map_frame}', "
                    "位置可能完全不可用, 已跳过加载"
                )
                return
            self.associator.load_dict(data)
            self.get_logger().info(
                f"已从 {self.persistence_path} 载入 "
                f"{len(self.associator.tracks)} 条语义地图记录"
            )
        except Exception as exc:
            self.get_logger().error(f"载入语义地图失败: {type(exc).__name__}: {exc}")

    def destroy_node(self) -> bool:
        self._save()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapper()
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
