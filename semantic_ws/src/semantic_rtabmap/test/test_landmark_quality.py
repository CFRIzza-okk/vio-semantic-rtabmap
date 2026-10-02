"""landmark_quality 的单元测试。

重点锁定两个**会被静默破坏**的不变量：
  1. 角方差 >= 9999 —— 否则 RTAB-Map 改用 BetweenFactor<Pose3> 并要求地标朝向，
     YOLO 的 yaw 不可靠，会污染优化（且不会报错，只是结果变差）。
  2. 质量 q=1 时方差必须等于基线 0.04 —— 保证改动相对 tag_linear_variance:=0.04
     是可对照的，而不是另起一套不可比的东西。
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from semantic_rtabmap.landmark_quality import (  # noqa: E402
    DEFAULT_CONFIG,
    MATERIAL_OPAQUE,
    MATERIAL_PENETRABLE,
    MATERIAL_UNCERTAIN,
    covariance_from_sigma,
    infer_material,
    landmark_covariance,
    landmark_sigma,
    semantic_quality,
)

# RTAB-Map OptimizerGTSAM.cpp:362-364 的硬阈值
RTABMAP_POINT_ONLY_THRESHOLD = 9999.0


# --------------------------------------------------------------------------
# 角方差不变量
# --------------------------------------------------------------------------
def test_angular_variance_stays_point_only():
    """角方差必须 >= 9999，否则 RTAB-Map 会要求地标朝向。"""
    assert DEFAULT_CONFIG.angular_variance >= RTABMAP_POINT_ONLY_THRESHOLD


def test_covariance_diagonal_layout():
    """协方差是 6x6 行主序，对角项 = [x,y,z,roll,pitch,yaw]。"""
    cov = covariance_from_sigma(0.2)
    assert len(cov) == 36
    diag = [cov[i * 6 + i] for i in range(6)]

    assert diag[0] == pytest.approx(0.04)   # 0.2^2，与基线 tag_linear_variance 一致
    assert diag[1] == pytest.approx(0.04)
    assert diag[2] == pytest.approx(0.04)
    for i in (3, 4, 5):
        assert diag[i] >= RTABMAP_POINT_ONLY_THRESHOLD

    # 非对角项必须为零（RTAB-Map 直接把这个块当信息矩阵用）
    for r in range(6):
        for c in range(6):
            if r != c:
                assert cov[r * 6 + c] == 0.0


def test_covariance_first_entry_positive():
    """RTAB-Map 只看 (0,0) 是否为正有限值来决定是否采用我们的协方差。

    若为 0，我们的自适应协方差会被**静默丢弃**并回退到全局参数。
    """
    for sigma in (0.05, 0.2, 1.0, 5.0):
        cov = covariance_from_sigma(sigma)
        assert cov[0] > 0.0
        assert math.isfinite(cov[0])


# --------------------------------------------------------------------------
# 质量模型单调性
# --------------------------------------------------------------------------
def test_quality_monotonic_in_confidence():
    qs = [semantic_quality(c, 10.0, 0.9) for c in (0.1, 0.3, 0.5, 0.7, 0.95)]
    assert qs == sorted(qs)
    assert qs[-1] > qs[0]


def test_quality_monotonic_and_saturating_in_evidence():
    qs = [semantic_quality(0.8, n, 0.9) for n in (0.0, 1.0, 3.0, 10.0, 50.0)]
    assert qs == sorted(qs)
    # 饱和：50 次与 500 次几乎无差别，避免"看得久"被当成"看得准"
    assert semantic_quality(0.8, 50.0, 0.9) == pytest.approx(
        semantic_quality(0.8, 500.0, 0.9), abs=1e-3
    )


def test_quality_monotonic_in_density():
    qs = [semantic_quality(0.8, 10.0, d) for d in (0.05, 0.2, 0.4, 0.6, 0.95)]
    assert qs == sorted(qs)


def test_quality_geometric_mean_penalises_zero_factor():
    """几何平均：深度完全无效时，高置信度不能救回质量。"""
    q = semantic_quality(0.95, 100.0, 0.0)
    assert q == pytest.approx(DEFAULT_CONFIG.q_min)
    assert q < 0.2


def test_quality_bounds():
    assert semantic_quality(1.0, 1e6, 1.0) <= 1.0
    assert semantic_quality(0.0, 0.0, 0.0) >= DEFAULT_CONFIG.q_min


# --------------------------------------------------------------------------
# 材质判别
# --------------------------------------------------------------------------
def test_material_classification():
    assert infer_material(0.10) == MATERIAL_PENETRABLE   # 透明瓶量级
    assert infer_material(0.96) == MATERIAL_OPAQUE       # 不透明标定板量级
    assert infer_material(0.45) == MATERIAL_UNCERTAIN    # 过渡带


def test_penetrable_gets_weaker_constraint():
    """同等质量下，疑似可穿透目标必须得到更大的方差（更弱的约束）。"""
    s_pen, _, m_pen = landmark_sigma(0.8, 10.0, 0.10)
    s_opq, _, m_opq = landmark_sigma(0.8, 10.0, 0.90)
    assert m_pen == MATERIAL_PENETRABLE
    assert m_opq == MATERIAL_OPAQUE
    assert s_pen > s_opq


# --------------------------------------------------------------------------
# 基线可比性
# --------------------------------------------------------------------------
def test_quality_one_reproduces_baseline_variance():
    """q=1 的不透明地标必须精确复现基线方差 0.04（sigma 0.2）。"""
    cfg = DEFAULT_CONFIG
    # 构造一个质量=1 的输入
    sigma, q, material = landmark_sigma(1.0, 1e6, cfg.density_ref, cfg)
    assert material == MATERIAL_OPAQUE
    assert q == pytest.approx(1.0)
    assert sigma == pytest.approx(cfg.base_sigma)


def test_sigma_respects_bounds():
    cfg = DEFAULT_CONFIG
    for conf, n_ind, dens in [(0.0, 0.0, 0.0), (1.0, 1e6, 1.0), (0.5, 2.0, 0.2)]:
        sigma, _, _ = landmark_sigma(conf, n_ind, dens, cfg)
        assert cfg.min_sigma <= sigma <= cfg.max_sigma


def test_sigma_never_zero_or_negative():
    """方差为 0 会让 RTAB-Map 回退到全局参数，也意味着无穷大的信息量。"""
    for conf, n_ind, dens in [(0.0, 0.0, 0.0), (1e-9, 0.0, 0.0)]:
        sigma, _, _ = landmark_sigma(conf, n_ind, dens)
        assert sigma > 0.0


# --------------------------------------------------------------------------
# 一站式接口
# --------------------------------------------------------------------------
def test_landmark_covariance_end_to_end():
    cov, sigma, q, material = landmark_covariance(0.9, 12.0, 0.85)
    assert len(cov) == 36
    assert material == MATERIAL_OPAQUE
    assert 0.0 < q <= 1.0
    assert cov[0] == pytest.approx(sigma * sigma)
    assert cov[35] >= RTABMAP_POINT_ONLY_THRESHOLD


def test_transparent_landmark_is_weaker_than_opaque_end_to_end():
    cov_t, s_t, _, _ = landmark_covariance(0.9, 12.0, 0.10)
    cov_o, s_o, _, _ = landmark_covariance(0.9, 12.0, 0.90)
    assert cov_t[0] > cov_o[0]
    assert s_t > s_o
