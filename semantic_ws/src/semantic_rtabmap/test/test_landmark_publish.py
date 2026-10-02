"""semantic_mapper 地标发布路径的集成测试。

单元测试只验证了数学，这里验证**真正发布出去的消息**：
确认 ``_publish_landmarks`` 产出的 LandmarkDetections 协方差满足
RTAB-Map 的三个硬性前提，否则改动会静默失效（不报错，只是约束没生效）。

  1. covariance[0] > 0      —— 否则 RTAB-Map 丢弃我们的协方差，
                                回退到全局 landmark_linear_variance，
                                本方案等于没做。
  2. 角方差 >= 9999          —— 否则 OptimizerGTSAM 改用 BetweenFactor<Pose3>，
                                要求地标朝向可信（YOLO 的 yaw 不可靠）。
  3. 可穿透目标方差 > 不透明目标方差 —— 这是本方案的全部意义。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

rclpy = pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import Point, Vector3  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from semantic_rtabmap.mapper_node import SemanticMapper  # noqa: E402
from semantic_rtabmap_msgs.msg import SemanticObject, SemanticObjectArray  # noqa: E402

RTABMAP_POINT_ONLY_THRESHOLD = 9999.0


def _time(t: float) -> Time:
    """构造真正的 builtin_interfaces/Time（Header.stamp 不接受鸭子类型）。"""
    return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))


class _Recorder:
    """替身发布者：把 publish 到的消息存下来。"""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _obj(class_id, class_name, conf, pos, bbox, points_used):
    o = SemanticObject()
    o.class_name = class_name
    o.class_id = class_id
    o.confidence = conf
    o.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
    o.size = Vector3(x=0.3, y=0.3, z=0.3)
    o.yaw = 0.0
    o.distance = float(np.linalg.norm(pos))
    o.bbox = [float(v) for v in bbox]
    o.points_used = int(points_used)
    return o


def _frame(stamp, objs, seq):
    arr = SemanticObjectArray()
    arr.header = Header(stamp=stamp, frame_id="map")
    arr.frame_seq = seq
    arr.inference_ms = 30.0
    arr.objects = objs
    return arr


def _build_node(mode="quality"):
    node = SemanticMapper()
    # 绕过 TF：直接当作相机系与 map 系重合，专注验证协方差逻辑
    node._lookup_map_to_camera = lambda stamp: np.eye(4)
    node.covariance_mode = mode
    rec = _Recorder()
    node.pub_landmarks = rec
    node.pub_landmarks_rtabmap = []
    return node, rec


def _feed(node, frames):
    t = 0.0
    for i in range(frames):
        t += 0.5
        stamp = _time(t)
        objs = [
            # 透明嫌疑：深度支撑稀疏（1000 / 5000 = 0.20）
            _obj(39, "bottle", 0.80, (0.0, 0.0, 1.5), (100, 100, 150, 200), 1000),
            # 不透明：深度支撑充分（38000 / 40000 = 0.95）
            _obj(56, "chair", 0.90, (2.0, 0.0, 1.5), (300, 100, 500, 300), 38000),
        ]
        node._on_objects(_frame(stamp, objs, i))


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_publishes_adaptive_covariance(ros):
    node, rec = _build_node("quality")
    try:
        _feed(node, 5)
    finally:
        node.destroy_node()

    # _on_objects 每帧都会发布一次地标；取最后一帧（此时轨迹已确认）
    assert len(rec.msgs) >= 1, "应当发布地标"
    lands = rec.msgs[-1].landmarks
    assert len(lands) == 2, f"应有 2 个确认地标，实际 {len(lands)}"

    # 按 id 找回：id = class_id * 100000 + instance_id
    by_class = {}
    for d in lands:
        by_class[d.id // 100000] = d

    bottle = by_class[39]
    chair = by_class[56]

    for d, name in ((bottle, "bottle"), (chair, "chair")):
        cov = list(d.pose.covariance)
        assert len(cov) == 36
        # 前提 1：RTAB-Map 靠 (0,0) 判断是否采用我们的协方差
        assert cov[0] > 0.0, f"{name}: covariance[0] 必须为正，否则协方差被丢弃"
        assert np.isfinite(cov[0])
        # 前提 2：角方差 >= 9999，保证走 PointXYZ 分支
        for i in (3, 4, 5):
            assert cov[i * 6 + i] >= RTABMAP_POINT_ONLY_THRESHOLD, (
                f"{name}: 角方差必须 >= 9999，否则 RTAB-Map 要求地标朝向"
            )

    # 前提 3：稀疏深度支撑的目标必须拿到更弱的约束
    assert bottle.pose.covariance[0] > chair.pose.covariance[0], (
        "疑似可穿透目标的地标方差应大于不透明目标"
    )


def test_uniform_mode_still_falls_back_to_rtabmap_defaults(ros):
    """消融对照模式必须保留原行为（全零 -> RTAB-Map 全局参数）。"""
    node, rec = _build_node("uniform")
    try:
        _feed(node, 5)
    finally:
        node.destroy_node()

    lands = rec.msgs[-1].landmarks
    assert len(lands) == 2
    for d in lands:
        cov = list(d.pose.covariance)
        assert cov[0] == 0.0, "uniform 模式应保持全零协方差"
        assert all(v == 0.0 for v in cov)
