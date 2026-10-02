#!/usr/bin/env python3
"""association 模块单元测试。

重点验证两类典型故障不会发生：
  - 过度分裂：同一物体被当成多个
  - 错误合并：多个物体被当成一个
运行: python3 /home/jetson/VIO/tools/test_association.py
"""

import sys

import numpy as np

sys.path.insert(0, "/home/jetson/VIO/semantic_ws/src/semantic_rtabmap")

from semantic_rtabmap.association import (  # noqa: E402
    Associator,
    AssociatorConfig,
    ConfidenceEstimator,
    Detection,
    circular_mean_2x,
)

failures = []


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + ("" if cond else f"  {detail}"))
    if not cond:
        failures.append(name)


def det(cls, pos, conf=0.8, size=(0.3, 0.3, 0.3), yaw=0.0, stamp=0.0, cid=0):
    return Detection(
        class_name=cls,
        class_id=cid,
        confidence=conf,
        position=np.array(pos, dtype=np.float64),
        size=np.array(size, dtype=np.float64),
        yaw=yaw,
        stamp=stamp,
    )


print("=== 1. 置信度融合器 ===")
# 先验：没有任何观测时应回到先验值附近
e0 = ConfidenceEstimator(prior=0.35, prior_weight=1.0)
check("无观测时等于先验", abs(e0.confidence - 0.35) < 1e-9, f"得到 {e0.confidence}")

# 一批高置信度观测应把置信度推高
e1 = ConfidenceEstimator()
for _ in range(30):
    e1.update(0.9, 0.0)
check("高置信度观测推高融合置信度", e1.confidence > 0.75, f"得到 {e1.confidence:.4f}")

# 一批低置信度观测应把置信度压低但仍接近先验
e2 = ConfidenceEstimator()
for _ in range(30):
    e2.update(0.3, 0.0)
check("低置信度观测压低置信度", e2.confidence < 0.35, f"得到 {e2.confidence:.4f}")
check("但不应塌陷到 0", e2.confidence > 0.25, f"得到 {e2.confidence:.4f}")

check("置信度始终在 [0,1]", all(0.0 <= v <= 1.0 for v in [e0.confidence, e1.confidence, e2.confidence]))

# 边界值不应产生 NaN
e3 = ConfidenceEstimator()
e3.update(0.0, 0.0)
e3.update(1.0, 0.0)
check("0/1 边界不产生 NaN", np.isfinite(e3.confidence), f"得到 {e3.confidence}")

# 独立观测数：密集采样只算一份证据
e4 = ConfidenceEstimator()
for k in range(100):
    e4.update(0.8, k * 0.033)  # 30 Hz，共 3.3 秒
check("等效独立观测数远小于帧数", 4 <= e4.independent <= 9,
      f"得到 {e4.independent} (共 100 帧)")

print("\n=== 1b. 回归测试: 长时间观测真物体不能把置信度压没 ===")
# 这是本项目实际踩到的 bug：相机 20Hz 连续观测同一物体，朴素对数几率累加
# 会把 0.35 置信度的真物体在几百帧后压到 1e-6，物体从地图上消失。
a = Associator(AssociatorConfig(min_observations=3, min_confidence=0.3))
for k in range(600):
    a.update([det("apple", [1.0, 0.0, 0.0], conf=0.35, stamp=k * 0.05)], k * 0.05)
tr = list(a.tracks.values())[0]
check("600 帧后置信度未塌陷", tr.confidence > 0.3,
      f"得到 {tr.confidence:.6f}")
check("物体仍在地图中", len(a.confirmed_tracks()) == 1,
      f"得到 {len(a.confirmed_tracks())}")
check("等效独立观测数合理", 10 <= tr.independent_observations <= 65,
      f"得到 {tr.independent_observations} (共 600 帧 / 30 秒)")

print("\n=== 1c. 高置信度长时间观测 ===")
a = Associator(AssociatorConfig())
for k in range(400):
    a.update([det("cup", [1.0, 0.0, 0.0], conf=0.85, stamp=k * 0.05)], k * 0.05)
tr = list(a.tracks.values())[0]
# 这个量的语义是"平均检测质量"，所以应收敛到观测均值 0.85，而不是被推到 1.0
check("高置信度物体收敛到观测均值 0.85", abs(tr.confidence - 0.85) < 0.02, f"得到 {tr.confidence:.4f}")

def axial_close(a_deg, b_deg, tol=1e-6):
    """轴向量比较：角度相差 180 度视为同一个朝向。"""
    d = abs(a_deg - b_deg) % 180.0
    return min(d, 180.0 - d) < tol


print("\n=== 2. 圆均值（朝向是轴向量，有 180 度歧义）===")
# 算术平均对 {10, 170} 会给出 90（错误）；轴向正确答案是 0，
# 因为 170 等价于 -10，与 10 关于 0 对称。
cm = circular_mean_2x(np.radians([10.0, 170.0]), np.ones(2))
check("轴向歧义处理正确 (10,170 -> 0)", axial_close(np.degrees(cm), 0.0),
      f"得到 {np.degrees(cm):.4f} 度")
cm2 = circular_mean_2x(np.radians([88.0, 92.0]), np.ones(2))
check("普通情况接近算术均值 (88,92 -> 90)", axial_close(np.degrees(cm2), 90.0),
      f"得到 {np.degrees(cm2):.4f} 度")
cm3 = circular_mean_2x(np.radians([179.0, -179.0]), np.ones(2))
check("跨 ±180 度正确处理", axial_close(np.degrees(cm3), 0.0),
      f"得到 {np.degrees(cm3):.4f} 度")
cm4 = circular_mean_2x(np.array([]), np.array([]))
check("空输入返回 0", cm4 == 0.0)

print("\n=== 3. 同一物体多帧观测 -> 只应有一条轨迹 ===")
a = Associator(AssociatorConfig())
for k in range(6):
    # 位置带噪声，模拟深度抖动 ±3cm
    p = np.array([1.0, 0.0, 0.0]) + np.array([0.03, -0.02, 0.01]) * ((-1) ** k)
    a.update([det("bottle", p, conf=0.7, stamp=float(k))], float(k))
check("只生成 1 条轨迹", len(a.tracks) == 1, f"得到 {len(a.tracks)}")
tr = list(a.tracks.values())[0]
check("观测计数为 6", tr.observation_count == 6, f"得到 {tr.observation_count}")
check("位置被融合到真值附近",
      np.linalg.norm(tr.position - np.array([1.0, 0.0, 0.0])) < 0.02,
      f"得到 {tr.position}")
# 6 次观测后应收敛到观测均值 0.7 附近（先验权重 1.0，仍有轻微下拉）
check("融合置信度收敛到观测均值 0.7 附近",
      abs(tr.confidence - 0.7) < 0.1, f"得到 {tr.confidence:.4f}")
check("已转为确认轨迹", tr.is_confirmed(3))

print("\n=== 4. 两个相距较远的同类物体 -> 两条轨迹 ===")
a = Associator(AssociatorConfig())
for k in range(5):
    a.update(
        [
            det("cup", [1.0, 0.0, 0.0], stamp=float(k)),
            det("cup", [4.0, 0.0, 0.0], stamp=float(k)),
        ],
        float(k),
    )
check("生成 2 条轨迹", len(a.tracks) == 2, f"得到 {len(a.tracks)}")
check("两条都在累积观测",
      all(t.observation_count == 5 for t in a.tracks.values()),
      f"得到 {[t.observation_count for t in a.tracks.values()]}")

print("\n=== 5. 不同类别在同一位置 -> 不能合并 ===")
a = Associator(AssociatorConfig())
for k in range(4):
    a.update(
        [
            det("person", [2.0, 0.0, 0.0], stamp=float(k), cid=0),
            det("chair", [2.0, 0.0, 0.0], stamp=float(k), cid=56),
        ],
        float(k),
    )
check("类别门控: 生成 2 条轨迹", len(a.tracks) == 2, f"得到 {len(a.tracks)}")
names = sorted(t.class_name for t in a.tracks.values())
check("类别正确分离", names == ["chair", "person"], f"得到 {names}")

print("\n=== 6. 两个同类物体靠得很近但仍应分离 ===")
# 相距 1.0m 的两个杯子，尺度 0.15m -> 门限 0.6 + 0.75*0.15 = 0.71m
a = Associator(AssociatorConfig())
for k in range(6):
    a.update(
        [
            det("cup", [2.0, 0.0, 0.0], size=(0.15, 0.15, 0.15), stamp=float(k)),
            det("cup", [3.0, 0.0, 0.0], size=(0.15, 0.15, 0.15), stamp=float(k)),
        ],
        float(k),
    )
check("近距离同类物体未错误合并", len(a.tracks) == 2, f"得到 {len(a.tracks)}")
if len(a.tracks) == 2:
    xs = sorted(t.position[0] for t in a.tracks.values())
    check("两个物体位置分别收敛到 2.0 和 3.0",
          abs(xs[0] - 2.0) < 0.05 and abs(xs[1] - 3.0) < 0.05, f"得到 {xs}")

print("\n=== 7. 匈牙利 vs 贪心 结果应一致 ===")
res = {}
for use_h in (True, False):
    a = Associator(AssociatorConfig(use_hungarian=use_h))
    for k in range(5):
        a.update(
            [
                det("cup", [1.0, 0.0, 0.0], stamp=float(k)),
                det("cup", [2.0, 0.0, 0.0], stamp=float(k)),
                det("cup", [3.0, 0.0, 0.0], stamp=float(k)),
            ],
            float(k),
        )
    res[use_h] = len(a.tracks)
check("两种分配算法都得到 3 条轨迹",
      res[True] == 3 and res[False] == 3, f"得到 {res}")

print("\n=== 8. 确认门限与置信度过滤 ===")
a = Associator(AssociatorConfig(min_observations=3, min_confidence=0.3))
a.update([det("cup", [1.0, 0.0, 0.0], conf=0.9, stamp=0.0)], 0.0)
check("观测 1 次不算确认", len(a.confirmed_tracks()) == 0,
      f"得到 {len(a.confirmed_tracks())}")
a.update([det("cup", [1.0, 0.0, 0.0], conf=0.9, stamp=1.0)], 1.0)
a.update([det("cup", [1.0, 0.0, 0.0], conf=0.9, stamp=2.0)], 2.0)
check("观测 3 次转为确认", len(a.confirmed_tracks()) == 1,
      f"得到 {len(a.confirmed_tracks())}")

a2 = Associator(AssociatorConfig(min_observations=1, min_confidence=0.9))
a2.update([det("cup", [1.0, 0.0, 0.0], conf=0.4, stamp=0.0)], 0.0)
check("低置信度轨迹被过滤", len(a2.confirmed_tracks()) == 0,
      f"得到 {len(a2.confirmed_tracks())}")

print("\n=== 9. 陈旧轨迹清理 ===")
a = Associator(AssociatorConfig(min_observations=3, prune_age=10.0))
a.update([det("cup", [1.0, 0.0, 0.0], stamp=0.0)], 0.0)
check("轨迹存在", len(a.tracks) == 1)
a.update([], 20.0)  # 20 秒没有任何检测
check("未确认的陈旧轨迹被清理", len(a.tracks) == 0, f"得到 {len(a.tracks)}")

a3 = Associator(AssociatorConfig(min_observations=3, prune_age=10.0))
for k in range(4):
    a3.update([det("cup", [1.0, 0.0, 0.0], stamp=float(k))], float(k))
check("已确认轨迹就位", len(a3.confirmed_tracks()) == 1)
a3.update([], 100.0)
check("已确认轨迹不会被自动删除(语义地图应长期保留)",
      len(a3.tracks) == 1, f"得到 {len(a3.tracks)}")

print("\n=== 10. 空帧 / 空输入边界 ===")
a = Associator(AssociatorConfig())
check("空检测不崩溃", a.update([], 0.0) == [])
check("无轨迹时正常", len(a.tracks) == 0)

print("\n=== 11. 序列化往返 ===")
a = Associator(AssociatorConfig())
for k in range(5):
    a.update(
        [
            det("cup", [1.0, 2.0, 0.5], conf=0.8, stamp=float(k)),
            det("car", [8.0, -1.0, 0.0], size=(4.0, 1.8, 1.5), conf=0.9, stamp=float(k)),
        ],
        float(k),
    )
d = a.to_dict()
b = Associator(AssociatorConfig())
b.load_dict(d)
check("轨迹数量一致", len(b.tracks) == len(a.tracks),
      f"{len(b.tracks)} vs {len(a.tracks)}")
same = all(
    abs(b.tracks[i].position - a.tracks[i].position).max() < 1e-9
    for i in a.tracks
)
check("位置精确还原", same)
ids = sorted(b.tracks.keys())
check("next_id 正确恢复", b._next_id > max(ids), f"next_id={b._next_id}")

print("\n=== 12. 位置历史用于评估 ===")
a = Associator(AssociatorConfig())
for k in range(250):
    a.update([det("cup", [1.0, 0.0, 0.0], stamp=float(k))], float(k))
tr = list(a.tracks.values())[0]
check("历史被限制在 200 条以内", len(tr.position_history) <= 200,
      f"得到 {len(tr.position_history)}")

print("\n" + "=" * 48)
if failures:
    print(f"结果: {len(failures)} 项失败 -> {failures}")
    sys.exit(1)
print("结果: 全部通过")
