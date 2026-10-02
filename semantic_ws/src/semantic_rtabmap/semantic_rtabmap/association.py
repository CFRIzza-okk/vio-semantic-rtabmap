"""多帧语义物体数据关联（data association）。

这是语义 SLAM 里最核心也最容易出问题的一环：同一个物体在多帧里被反复
检测到（杯子、椅子、人……），系统必须判断"这一帧看到的杯子，和上一帧
那个杯子，是不是同一个物体"。判断错了就会出现两种典型故障：

  - 过度分裂 (over-segmentation)：同一个杯子因为视角变化被当成 5 个不同
    物体，语义地图上出现 5 个杯子。
  - 错误合并 (under-segmentation)：两个相邻的同类物体被合并成一个，
    "去左边的杯子那里"就会导航到错误目标。

本模块采用「类别门控 + 尺寸自适应距离门限 + 全局最优分配」的经典组合：

  1. 类别门控：只有 class_name 相同的检测和轨迹才可能关联。这一条就排除了
     绝大部分误关联。
  2. 尺寸自适应门限：允许的关联距离 = base_gate + size_factor * 物体尺度。
     杯子(0.1m)和汽车(4m)显然不能共用 1 米的门限。
  3. 全局最优分配：用匈牙利算法(scipy)在所有"检测-轨迹"对上求总代价最小的
     一一匹配，而不是贪心地逐个挑最近的。贪心在物体密集时会出错。
  4. 置信度用对数几率(log-odds)累加融合，多次弱检测可以累积成高置信度，
     这比直接取平均更符合贝叶斯更新的语义。

术语：本模块里一个被跟踪的物体叫 Track（轨迹/航迹）。
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment

    _HAS_SCIPY = True
except ImportError:  # 回退到贪心匹配
    _HAS_SCIPY = False


# 数值保护：避免除零和 log(0)
_EPS = 1e-6


class ConfidenceEstimator:
    """把一串检测置信度融合成"这个物体确实存在"的置信度。

    为什么不直接用对数几率(log-odds)累加
    ------------------------------------
    这是本项目实际踩到的坑：朴素贝叶斯累加要求各次观测**相互独立**。但
    相机以 20 Hz 连续观测同一个静止物体，相邻帧的检测高度相关——本质上是
    同一份证据被重复计入几百次。后果是：

      - 一个置信度稳定在 0.35 的真物体，因为 0.35 < 0.5 对应的对数几率为负，
        每帧都在"投反对票"，几百帧后融合置信度被压到 1e-6，物体从地图上消失。
      - 反过来，若单帧置信度是 0.7，几百帧后又会被推到 ~1.0，把偶然的误检
        也变成"铁证"。

    正确做法
    --------
    取其期望，用**置信度加权平均**，并做一个很小的收缩(shrinkage)：

        s = Σ c_i
        conf = (Σ c_i² + c_prior * k) / (s + k)

    把 s 理解为"证据总量"（第 i 帧按 c_i 的权重计入），k 相当于预先加入
    k 份置信度为 c_prior 的观测：
      - 样本很少时结果被先验拉住，避免"一帧高分就确认一个物体"
      - 样本足够多时收敛到观测均值本身

    注意这个量的含义是"**平均检测质量**"，不是"物体存在的概率"。重复观测
    不会把它推向 1.0——推高它的是检测器给出更高的置信度。物体"是否存在"由
    is_confirmed() 的最少观测次数门槛负责判断。把这两件事分开是有意为之：
    混成一个数会让"看到 600 帧但每帧只有 0.35"和"看到 3 帧但每帧 0.95"
    得到同一种结果，而那两种情况该有完全不同的处理。

    同时输出**独立观测数**：把观测按时间间隔折算成等效独立样本数，
    用来给下游（如 RTAB-Map 的地标协方差）一个合理的可信度度量。
    """

    # 同一物体相邻观测之间的等效间隔（秒）。30 Hz 相机取 0.5s，
    # 即认为半秒内的多次观测算作"同一份证据"。
    INDEPENDENT_INTERVAL = 0.5

    def __init__(self, prior: float = 0.35, prior_weight: float = 1.0) -> None:
        self.prior = float(prior)
        self.k = max(float(prior_weight), 1e-6)
        self.weighted_sum = 0.0    # Σ c_i
        self.square_sum = 0.0      # Σ c_i²
        self.independent = 0.0     # 等效独立观测数
        self._last_indep_time: Optional[float] = None

    @property
    def confidence(self) -> float:
        denom = self.weighted_sum + self.k
        if denom <= 0.0:
            return self.prior
        return (self.square_sum + self.prior * self.k) / denom

    def update(self, detection_confidence: float, stamp: Optional[float] = None) -> None:
        c = min(max(float(detection_confidence), 0.0), 1.0)
        self.weighted_sum += c
        self.square_sum += c * c
        if stamp is not None:
            if (
                self._last_indep_time is None
                or (stamp - self._last_indep_time) >= self.INDEPENDENT_INTERVAL
            ):
                self.independent += 1.0
                self._last_indep_time = stamp

    def to_dict(self) -> dict:
        return {
            "weighted_sum": self.weighted_sum,
            "square_sum": self.square_sum,
            "independent": self.independent,
            "prior": self.prior,
            "prior_weight": self.k,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "ConfidenceEstimator":
        est = cls(
            prior=float(d.get("prior", 0.35)),
            prior_weight=float(d.get("prior_weight", 1.0)),
        )
        est.weighted_sum = float(d.get("weighted_sum", 0.0))
        est.square_sum = float(d.get("square_sum", 0.0))
        est.independent = float(d.get("independent", 0.0))
        return est


def circular_mean_2x(angles: np.ndarray, weights: np.ndarray) -> float:
    """轴向(axial)角度的加权圆均值。

    这里的"朝向"是**轴向量**：一个杯子朝 10 度还是朝 190 度是同一条轴线，
    两者不可区分。所以不能直接对角度求算术平均——对 {10°, 170°} 做算术平均
    会得到 90°，而正确答案是 0°（因为 170° 等价于 -10°，与 10° 关于 0° 对称）。

    标准做法是把角度加倍映射到圆周（θ -> 2θ，把轴向歧义折叠掉），在圆周上
    求加权平均，再除以 2 还原。结果落在 [-90°, 90°] 区间，与轴向语义一致。
    """
    if angles.size == 0:
        return 0.0
    s = float(np.sum(np.sin(2.0 * angles) * weights))
    c = float(np.sum(np.cos(2.0 * angles) * weights))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        # 两个互相垂直的轴（比如 0° 和 90°）均值本身就是病态的，返回 0
        return 0.0
    return float(0.5 * np.arctan2(s, c))


@dataclass
class Detection:
    """一次单帧观测（来自 detector_node）。位置/尺寸在统一的目标坐标系下。"""

    class_name: str
    class_id: int
    confidence: float
    position: np.ndarray
    size: np.ndarray
    yaw: float
    stamp: float
    bbox: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    points_used: int = 0
    distance: float = 0.0
    # 深度支撑密度 = points_used / 检测框面积。
    # None 表示算不出来（检测框退化），此时下游不得据此推断材质。
    density: Optional[float] = None


@dataclass
class Track:
    """一个被持续跟踪的语义物体实例，也是语义地图里的一条记录。"""

    instance_id: int
    class_name: str
    class_id: int

    position: np.ndarray
    size: np.ndarray
    yaw: float

    observation_count: int = 0
    # 置信度融合器（见 ConfidenceEstimator 的说明，这里不能用朴素对数几率累加）
    confidence_est: ConfidenceEstimator = field(default_factory=ConfidenceEstimator)
    weight_sum: float = 0.0      # 位置/尺寸融合的权重累计
    yaw_sin_sum: float = 0.0
    yaw_cos_sum: float = 0.0

    first_seen: float = 0.0
    last_seen: float = 0.0
    best_confidence: float = 0.0

    rtabmap_node_id: int = -1
    # 深度支撑密度的置信度加权均值（只累加算得出密度的那些观测）
    density_sum: float = 0.0
    density_weight_sum: float = 0.0
    # 用于离线评估关联稳定性（写论文时画轨迹抖动曲线）
    position_history: List[Tuple[float, np.ndarray]] = field(default_factory=list)

    @property
    def density(self) -> Optional[float]:
        """该实例的深度支撑密度（置信度加权均值）。

        返回 None 表示从未拿到过可用的密度样本——下游必须把它当作
        "未知"处理，而不能默认成"不透明"或"可穿透"。
        """
        if self.density_weight_sum <= 0.0:
            return None
        return self.density_sum / self.density_weight_sum

    @property
    def confidence(self) -> float:
        """融合后的置信度：这个物体真实存在的概率估计。"""
        return self.confidence_est.confidence

    @property
    def independent_observations(self) -> float:
        """等效独立观测数，反映证据的"份数"而非帧数。"""
        return self.confidence_est.independent

    @property
    def age(self) -> float:
        return max(0.0, self.last_seen - self.first_seen)

    def is_confirmed(self, min_observations: int) -> bool:
        return self.observation_count >= min_observations

    def gate_radius(self, base_gate: float, size_factor: float) -> float:
        scale = float(np.mean(self.size[:2])) if self.size.size >= 2 else 0.0
        return base_gate + size_factor * max(scale, 0.05)

    def predict_position(self) -> np.ndarray:
        """静态物体不做运动预测，直接用融合位置作为预测。

        如果将来要处理行人这类动态目标，这里应该换成匀速模型
        x' = x + v * dt（这也是论文里可以展开的改进点）。
        """
        return self.position

    def update(self, det: Detection) -> None:
        """用一次新观测更新轨迹（置信度加权累加平均）。"""
        w = max(float(det.confidence), _EPS)

        # --- 位置 / 尺寸：置信度加权的累加平均 ---
        # 物体是静止的，累加平均能有效压低单帧深度噪声；
        # 用置信度做权重则让"看得清楚的那几帧"说话更有分量。
        total = self.weight_sum + w
        self.position = (self.position * self.weight_sum + det.position * w) / total
        self.size = (self.size * self.weight_sum + det.size * w) / total
        self.weight_sum = total

        # --- 朝向：圆均值 ---
        self.yaw_sin_sum += np.sin(2.0 * det.yaw) * w
        self.yaw_cos_sum += np.cos(2.0 * det.yaw) * w
        if abs(self.yaw_sin_sum) > 1e-12 or abs(self.yaw_cos_sum) > 1e-12:
            self.yaw = float(0.5 * np.arctan2(self.yaw_sin_sum, self.yaw_cos_sum))

        # --- 置信度：加权平均 + 收缩（不要用朴素对数几率累加，见类文档） ---
        self.confidence_est.update(det.confidence, det.stamp)

        # --- 深度支撑密度：同样按置信度加权累加 ---
        # 它是材质判别（可穿透 / 不可穿透）的输入，也是地标协方差的因子。
        if det.density is not None:
            self.density_sum += float(det.density) * w
            self.density_weight_sum += w

        self.observation_count += 1
        self.best_confidence = max(self.best_confidence, float(det.confidence))
        self.last_seen = det.stamp

        # 轨迹历史保留最近 200 条，避免长时间运行内存膨胀
        self.position_history.append((det.stamp, det.position.copy()))
        if len(self.position_history) > 200:
            del self.position_history[: len(self.position_history) - 200]


@dataclass
class AssociatorConfig:
    # 关联距离门限（米）：gate = base_gate + size_factor * 物体尺度
    base_gate: float = 0.6
    size_factor: float = 0.75
    # 门限的绝对上限，防止超大物体（比如整面墙被误检）吞并一切
    max_gate: float = 3.0

    # 轨迹转正所需的最少观测次数
    min_observations: int = 3
    # 超过这个时间没被观测到就认为丢失（秒）
    max_age: float = 3.0
    # 丢失后保留多久再删除（秒），给重观测留出恢复的机会
    prune_age: float = 30.0
    # 是否使用匈牙利算法做全局最优分配
    use_hungarian: bool = True

    # 融合置信度的合理范围：低于此值的轨迹不进入语义地图
    min_confidence: float = 0.3

    # 置信度融合的先验（见 ConfidenceEstimator）
    # 没有观测时先验设为 0.35：略低于默认 detector conf 阈值 0.35，
    # 意味着一帧很弱的观测不足以马上把物体塞进地图
    confidence_prior: float = 0.35
    # 先验权重（等效先验观测次数）；小权重让 3-6 次观测能较快收敛
    confidence_prior_weight: float = 1.0


class Associator:
    """维护全部轨迹，并把每帧检测关联到轨迹上。"""

    def __init__(self, config: Optional[AssociatorConfig] = None) -> None:
        self.cfg = config or AssociatorConfig()
        self.tracks: Dict[int, Track] = {}
        self._next_id = 1
        # 统计量，写论文的实验章节能直接用
        self.stats = {
            "frames": 0,
            "detections": 0,
            "matched": 0,
            "created": 0,
            "pruned": 0,
            "unmatched": 0,
        }

    # ------------------------------------------------------------------
    def update(self, detections: List[Detection], stamp: float) -> List[Track]:
        """处理一帧检测，返回本帧被更新或新建的轨迹列表。

        未被本帧看到的轨迹不会被删除，只是"变老"，由 prune 负责清理。
        """
        self.stats["frames"] += 1
        self.stats["detections"] += len(detections)

        candidates = list(self.tracks.values())
        matches, unmatched_dets = self._associate(detections, candidates)

        touched: List[Track] = []

        for di, ti in matches:
            det = detections[di]
            # 注意用 candidates[ti] 而不是 self.tracks[ti]：
            # ti 是 candidates 列表里的下标，不是 instance_id（字典键）。
            # 两者在第一个轨迹被删除后会不一致，用错会抛 KeyError。
            track = candidates[ti]
            track.update(det)
            touched.append(track)
            self.stats["matched"] += 1

        for di in unmatched_dets:
            det = detections[di]
            track = self._create(det)
            touched.append(track)
            self.stats["created"] += 1

        self.stats["unmatched"] = len(unmatched_dets)
        self._prune(stamp)
        return touched

    # ------------------------------------------------------------------
    def _associate(
        self, detections: List[Detection], tracks: List[Track]
    ) -> Tuple[List[Tuple[int, int]], List[int]]:
        """返回 (匹配对 (检测下标, 轨迹下标), 未匹配的检测下标)。"""
        if not detections or not tracks:
            return [], list(range(len(detections)))

        nd, nt = len(detections), len(tracks)
        # 代价矩阵：不可关联的位置填一个很大的代价
        BIG = 1e6
        cost = np.full((nd, nt), BIG, dtype=np.float64)

        for i, det in enumerate(detections):
            for j, track in enumerate(tracks):
                # --- 类别门控：不同类别绝不关联 ---
                if det.class_name != track.class_name:
                    continue

                d = float(np.linalg.norm(det.position - track.predict_position()))
                gate = min(
                    track.gate_radius(self.cfg.base_gate, self.cfg.size_factor),
                    self.cfg.max_gate,
                )
                # 尺寸差异过大的同类物体（比如远处的杯子和近处的桶）
                # 也拦掉，避免只靠位置就把它们合并
                ds = float(np.linalg.norm(det.size - track.size))
                if ds > max(1.0, 1.2 * float(np.mean(track.size))):
                    continue

                if d <= gate:
                    # 代价同时考虑距离和尺寸差异，位置为主
                    cost[i, j] = d + 0.3 * ds

        matches: List[Tuple[int, int]] = []
        if self.cfg.use_hungarian and _HAS_SCIPY:
            # 匈牙利算法求全局最优一一匹配
            row, col = linear_sum_assignment(cost)
            used_d, used_t = set(), set()
            for r, c in zip(row, col):
                if cost[r, c] < BIG * 0.5:
                    matches.append((int(r), int(c)))
                    used_d.add(int(r))
                    used_t.add(int(c))
        else:
            # 贪心：把所有可行对按代价从小到大排序，依次占用
            pairs = [
                (cost[i, j], i, j)
                for i in range(nd)
                for j in range(nt)
                if cost[i, j] < BIG * 0.5
            ]
            pairs.sort()
            used_d, used_t = set(), set()
            for _, i, j in pairs:
                if i in used_d or j in used_t:
                    continue
                matches.append((i, j))
                used_d.add(i)
                used_t.add(j)

        unmatched = [i for i in range(nd) if i not in used_d]
        return matches, unmatched

    # ------------------------------------------------------------------
    def _create(self, det: Detection) -> Track:
        est = ConfidenceEstimator(
            prior=self.cfg.confidence_prior,
            prior_weight=self.cfg.confidence_prior_weight,
        )
        est.update(det.confidence, det.stamp)
        tr = Track(
            instance_id=self._next_id,
            class_name=det.class_name,
            class_id=det.class_id,
            position=det.position.copy(),
            size=det.size.copy(),
            yaw=float(det.yaw),
            observation_count=1,
            confidence_est=est,
            weight_sum=max(float(det.confidence), _EPS),
            yaw_sin_sum=float(np.sin(2.0 * det.yaw)) * max(det.confidence, _EPS),
            yaw_cos_sum=float(np.cos(2.0 * det.yaw)) * max(det.confidence, _EPS),
            first_seen=det.stamp,
            last_seen=det.stamp,
            best_confidence=float(det.confidence),
        )
        tr.position_history.append((det.stamp, det.position.copy()))
        self.tracks[self._next_id] = tr
        self._next_id += 1
        return tr

    # ------------------------------------------------------------------
    def _prune(self, stamp: float) -> None:
        """删除长时间未观测且置信度不足的轨迹。"""
        dead = [
            tid
            for tid, tr in self.tracks.items()
            if (stamp - tr.last_seen) > self.cfg.prune_age
            and not tr.is_confirmed(self.cfg.min_observations)
        ]
        for tid in dead:
            del self.tracks[tid]
            self.stats["pruned"] += 1

    # ------------------------------------------------------------------
    def confirmed_tracks(self) -> List[Track]:
        """满足进入语义地图条件的轨迹。"""
        return [
            tr
            for tr in self.tracks.values()
            if tr.is_confirmed(self.cfg.min_observations)
            and tr.confidence >= self.cfg.min_confidence
        ]

    def to_dict(self) -> dict:
        return {
            "next_id": self._next_id,
            "stats": self.stats,
            "tracks": [
                {
                    "instance_id": tr.instance_id,
                    "class_name": tr.class_name,
                    "class_id": tr.class_id,
                    "position": tr.position.tolist(),
                    "size": tr.size.tolist(),
                    "yaw": tr.yaw,
                    "observation_count": tr.observation_count,
                    "confidence": tr.confidence,
                    "confidence_est": tr.confidence_est.to_dict(),
                    "best_confidence": tr.best_confidence,
                    "first_seen": tr.first_seen,
                    "last_seen": tr.last_seen,
                    "rtabmap_node_id": tr.rtabmap_node_id,
                    "density_sum": tr.density_sum,
                    "density_weight_sum": tr.density_weight_sum,
                }
                for tr in self.tracks.values()
            ],
        }

    def load_dict(self, data: dict) -> None:
        for t in data.get("tracks", []):
            ce = t.get("confidence_est")
            if ce is not None:
                est = ConfidenceEstimator.from_dict(ce)
            else:
                # 兼容旧格式：没有融合器状态时，用已融合置信度近似重建
                est = ConfidenceEstimator()
                est.update(float(t["confidence"]), 0.0)
            tr = Track(
                instance_id=int(t["instance_id"]),
                class_name=t["class_name"],
                class_id=int(t["class_id"]),
                position=np.asarray(t["position"], dtype=np.float64),
                size=np.asarray(t["size"], dtype=np.float64),
                yaw=float(t["yaw"]),
                observation_count=int(t["observation_count"]),
                confidence_est=est,
                weight_sum=max(float(t["observation_count"]), _EPS),
                first_seen=float(t["first_seen"]),
                last_seen=float(t["last_seen"]),
                best_confidence=float(t.get("best_confidence", t["confidence"])),
                rtabmap_node_id=int(t.get("rtabmap_node_id", -1)),
                density_sum=float(t.get("density_sum", 0.0)),
                density_weight_sum=float(t.get("density_weight_sum", 0.0)),
            )
            tr.yaw_sin_sum = float(np.sin(2.0 * tr.yaw)) * tr.weight_sum
            tr.yaw_cos_sum = float(np.cos(2.0 * tr.yaw)) * tr.weight_sum
            self.tracks[tr.instance_id] = tr
        self._next_id = max(
            [int(data.get("next_id", 1))] + [t + 1 for t in self.tracks.keys()]
        )
