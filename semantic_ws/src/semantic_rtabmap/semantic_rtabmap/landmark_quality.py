"""语义地标约束质量模型 —— 语义质量自适应的 RTAB-Map 位姿图约束。

问题背景
--------
RTAB-Map 0.23.7 在 ``rtabmap_conversions/src/MsgConversion.cpp:1996-2003`` 里
这样消费地标协方差::

    cv::Mat covariance = ...msg.pose.covariance...;
    if(covariance.empty() || !uIsFinite(covariance(0,0)) || covariance(0,0) <= 0.0f)
    {
        covariance = cv::Mat::eye(6,6, CV_64FC1);
        covariance(Range(0,3),Range(0,3)) *= defaultLinVariance;   // 全局参数
        covariance(Range(3,6),Range(3,6)) *= defaultAngVariance;   // 全局参数
    }

注意判据只检查 ``(0,0)`` 这一项。于是有两条路：

* **发全零协方差**（本项目原有做法）：RTAB-Map 用两个*全局*参数
  （``landmark_linear_variance`` / ``landmark_angular_variance``）填充**所有**地标。
  后果——只被看到 3 次、深度支撑很差的透明瓶子，和被稳定观测上百次的
  不透明柜子，在位姿图里是**同等强度**的约束。

* **发非零协方差**：RTAB-Map **原样采用**我们给的协方差。这是本模块的着力点。

本模块按每个地标**自身的语义质量与材质类别**计算协方差，使 RTAB-Map 直接采用：

    质量高                      -> 方差小 -> 强约束
    质量低 / 疑似红外可穿透      -> 方差大 -> 弱约束

三个质量因子
------------
1. **检测置信度** ``confidence``
   YOLO 给出的单帧质量，经多帧置信度融合（见 association.ConfidenceEstimator）。

2. **等效独立观测数** ``n_independent``
   相机 20 Hz 连续观测静止物体时，相邻帧高度相关，本质是同一份证据被重复
   计入几百次。本模块用等效独立观测数而非原始帧数，避免"看得久"被误当成
   "看得准"。

3. **深度支撑密度** ``density = points_used / 检测框面积``
   低密度意味着该物体的深度返回稀疏——红外被穿透（透明）、被吸收（深色哑光）、
   被遮挡或距离过远。这正是实验 E4/E5 中透明物体的特征签名：
   瓶身有效深度像素仅占检测框的 26.8%，而深度信息一旦稀疏，
   **横向位置**就最不可靠（E4：横向误差 25.6 mm，而距离误差仅 4.45 mm）。

关于各向异性（重要限制，也是后续内核改进的入口）
------------------------------------------------
E4 的结论是"距离可信、横向不可信"，直觉上应该给 x/y 大方差、z 小方差。
但**在当前 RTAB-Map 通道里这样发是无效的**，有两个原因：

* 协方差**不会被旋转**。``landmarksFromROS`` 把位姿从相机光学系变换到
  base_link，却把协方差原样透传（源码中无任何旋转操作）。因此协方差各轴的
  物理含义会随相机安装姿态而错位。
* 优化器把它喂给 ``BearingRangeFactor``，而该因子的噪声模型定义在
  **(方位, 距离)** 坐标上，RTAB-Map 却直接拷入**笛卡尔** x-y-z 的信息块
  （``OptimizerGTSAM.cpp:632-641``），坐标语义不匹配。

因此本模块**只做各向同性但强度自适应**的缩放——这是当前通道下唯一语义明确的
自由度，也是可被严格验证的部分。轴级各向异性需要修改内核，见后续工作。

关于材质判别的混淆风险（进入第二阶段前必须知道）
------------------------------------------------
``density = points_used / 检测框面积`` 偏低，并不只由"透明"造成，也可能来自：

* 距离过远（超出 D435 的有效深度量程）
* 被遮挡（物体只露出一部分）
* 深色哑光表面（红外被吸收）
* 运动模糊 / 曝光不当

也就是说，本判据把「红外被穿透」和「深度本来就不可靠」混在了一起。

在**当前**实现里这是可以接受的：我们只用它整体缩放约束强度，
而"远处/被遮挡/深度差"的物体本来也应该拿到更弱的约束，方向是一致的。

但一旦进入第二阶段（按材质**切换约束类型**，例如透明目标只注入距离约束、
丢弃方位约束），这个混淆就会变成**真实故障**：一个只是站得远的椅子会被
误判成"可穿透"，进而被错误地丢掉方位约束。

届时必须换用真正区分「穿透」与「反射/吸收」的判别量。E5 提出的
``r = 中心区域有效占比 / 整框有效占比`` 就是为此设计的——它对
"中心比边缘更缺深度"这一**透明性特有**的空间分布敏感，
而不是像 density 那样只对深度总量敏感。
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# 材质类别
# ---------------------------------------------------------------------------
MATERIAL_OPAQUE = "opaque"            # 不可穿透：深度支撑充分
MATERIAL_PENETRABLE = "penetrable"    # 疑似红外可穿透（透明/镜面）：深度支撑稀疏
MATERIAL_UNCERTAIN = "uncertain"      # 不确定：落在两者之间的过渡带


@dataclass
class LandmarkQualityConfig:
    """语义地标质量与协方差的配置。

    默认值经标定，使 ``quality == 1.0`` 时**恰好复现**本项目原有的
    ``tag_linear_variance:=0.04``（即 sigma = 0.20 m）行为，
    从而保证本改动相对于基线是可对照的、不是另起一套。
    """

    # ---- 位置方差模型 ----
    base_sigma: float = 0.20      # 质量 q=1 时的位置标准差（m）-> 方差 0.04
    min_sigma: float = 0.05       # 下限：再好的地标也不当作硬约束
    max_sigma: float = 5.00       # 上限：再差的地标也不至于完全瘫掉优化

    # ---- 角方差 ----
    # 【关键不变量】必须 >= 9999。
    # OptimizerGTSAM.cpp:362-364 用
    #     1/infMatrix(3,3) >= 9999 || 1/infMatrix(4,4) >= 9999 || 1/infMatrix(5,5) >= 9999
    # 判定"该地标只当点、不带姿态"。一旦角方差小于 9999，RTAB-Map 会改用
    # BetweenFactor<Pose3> 并要求地标朝向可信——而 YOLO 估的 yaw 并不可靠。
    # 注意：只要我们发了非零协方差，全局参数 tag_angular_variance 就**不再生效**，
    # 所以这个值必须由我们自己显式给出。
    angular_variance: float = 1.0e6

    # ---- 质量因子权重（加权几何平均）----
    w_confidence: float = 0.45
    w_evidence: float = 0.25
    w_depth: float = 0.30

    # ---- 各因子的归一化尺度 ----
    evidence_tau: float = 3.0     # 等效独立观测数的饱和尺度
    density_ref: float = 0.60     # 深度支撑密度达到此值即视为满分

    # ---- 材质判别阈值（初步值，待补实验后重新标定）----
    # 依据 E5：透明瓶（深度支撑密度约 0.20）与不透明标定板（约 0.96）完全分离。
    # ⚠ 不透明类目前 n=1，阈值属于**初步标定**，补齐椅子/书样本后应重新复核。
    penetrable_density: float = 0.35
    opaque_density: float = 0.60

    # ---- 质量下限 ----
    q_min: float = 0.05

    # ---- 材质带来的额外方差增益 ----
    # 稀疏深度支撑意味着横向位置不可靠，整体弱化该约束。
    gain_opaque: float = 1.0
    gain_uncertain: float = 1.5
    gain_penetrable: float = 2.5


DEFAULT_CONFIG = LandmarkQualityConfig()


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def infer_material(
    density: Optional[float], cfg: LandmarkQualityConfig = DEFAULT_CONFIG
) -> str:
    """按深度支撑密度判别材质类别。

    只用一个从现有数据就能算出的量：``points_used / 检测框面积``。
    它对应实验 E5 中「整框有效深度占比」这一判别特征的工程代理。

    ``density`` 为 None 表示无法计算（检测框退化、没有深度样本）。此时
    **不做任何猜测**：返回 UNCERTAIN 并给温和的方差增益。绝不能因为
    "密度算不出来"就把目标误判成可穿透而弱化约束。
    """
    if density is None:
        return MATERIAL_UNCERTAIN
    if density < cfg.penetrable_density:
        return MATERIAL_PENETRABLE
    if density >= cfg.opaque_density:
        return MATERIAL_OPAQUE
    return MATERIAL_UNCERTAIN


def material_gain(material: str, cfg: LandmarkQualityConfig = DEFAULT_CONFIG) -> float:
    return {
        MATERIAL_OPAQUE: cfg.gain_opaque,
        MATERIAL_UNCERTAIN: cfg.gain_uncertain,
        MATERIAL_PENETRABLE: cfg.gain_penetrable,
    }.get(material, cfg.gain_uncertain)


def semantic_quality(
    confidence: float,
    n_independent: float,
    density: Optional[float],
    cfg: LandmarkQualityConfig = DEFAULT_CONFIG,
) -> float:
    """把三个质量因子合成一个 (0, 1] 的语义质量分。

    用**加权几何平均**而非算术平均：任一因子趋于 0 时质量应当趋于 0。
    （算术平均下"置信度 0.9 但深度完全无效"会得到 0.6，明显偏高。）
    """
    q_conf = _clamp(float(confidence), 0.0, 1.0)

    n_indep = max(float(n_independent), 0.0)
    q_evid = 1.0 - math.exp(-n_indep / max(cfg.evidence_tau, 1e-6))

    if density is None:
        # 深度支撑密度未知时不额外惩罚：置信度与证据两个因子仍然生效。
        q_depth = 1.0
    else:
        q_depth = _clamp(float(density) / max(cfg.density_ref, 1e-6), 0.0, 1.0)

    w_sum = cfg.w_confidence + cfg.w_evidence + cfg.w_depth
    if w_sum <= 0.0:
        return cfg.q_min

    # 用 exp(sum(w*ln(q))) 实现加权几何平均；q=0 时 log 会发散，单独处理。
    for q in (q_conf, q_evid, q_depth):
        if q <= 0.0:
            return cfg.q_min

    log_q = (
        cfg.w_confidence * math.log(q_conf)
        + cfg.w_evidence * math.log(q_evid)
        + cfg.w_depth * math.log(q_depth)
    ) / w_sum

    return _clamp(math.exp(log_q), cfg.q_min, 1.0)


def landmark_sigma(
    confidence: float,
    n_independent: float,
    density: Optional[float],
    cfg: LandmarkQualityConfig = DEFAULT_CONFIG,
) -> Tuple[float, float, str]:
    """返回 (sigma_m, quality, material)。"""
    q = semantic_quality(confidence, n_independent, density, cfg)
    material = infer_material(density, cfg)
    sigma = cfg.base_sigma / math.sqrt(max(q, 1e-6)) * material_gain(material, cfg)
    sigma = _clamp(sigma, cfg.min_sigma, cfg.max_sigma)
    return sigma, q, material


def covariance_from_sigma(
    sigma: float, cfg: LandmarkQualityConfig = DEFAULT_CONFIG
) -> List[float]:
    """构造 RTAB-Map 期望的 6x6 行主序协方差（[x,y,z,roll,pitch,yaw]）。

    RTAB-Map 只检查 ``(0,0)`` 是否为正有限值来决定是否采用我们的协方差，
    所以线性块必须显式给出正数；角块必须显式给出 ``angular_variance``
    （此时全局 ``tag_angular_variance`` 已被旁路）。
    """
    var_lin = float(sigma) * float(sigma)
    var_ang = float(cfg.angular_variance)
    diag = [var_lin, var_lin, var_lin, var_ang, var_ang, var_ang]

    cov = [0.0] * 36
    for i in range(6):
        cov[i * 6 + i] = diag[i]
    return cov


def landmark_covariance(
    confidence: float,
    n_independent: float,
    density: Optional[float],
    cfg: LandmarkQualityConfig = DEFAULT_CONFIG,
) -> Tuple[List[float], float, float, str]:
    """一站式接口：返回 (6x6 协方差, sigma, quality, material)。"""
    sigma, q, material = landmark_sigma(confidence, n_independent, density, cfg)
    return covariance_from_sigma(sigma, cfg), sigma, q, material
