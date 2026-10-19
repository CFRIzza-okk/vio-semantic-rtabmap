#!/usr/bin/env python3
"""起自主探索:一个进程做完 检查 → 确认 → 设参数 → 启动。

为什么需要它
------------
原来 `r680_explore_start.sh` 在敲完 GO 之后要发起 12 次 `ros2` 命令行进程:
5 次 `param set`、5 次 `param get`、1 次预算回读、1 次 `service call`。
每次冷启动实测 1.0~1.2 s(负载高时更久),叠起来就是十几秒 —— 操作员敲了 GO,
车却纹丝不动,只能盯着屏幕等。这不是体验问题:等待会让人怀疑脚本坏了,
转去手敲 `ros2 param set` 绕过它,而绕过的恰恰是带校验的那条路。

这里改成**一个进程**:一次 ROS 初始化,然后 SetParameters(一次设全 5 个)、
GetParameters(一次读回校验)、Trigger(启动)。全部是毫秒级。

校验一条都不能少
----------------
* 定位无效 / 前方有障碍 → 拒绝启动(和原来一样,exit 1)。
* 参数设完之后**回读比对预算** —— 这是踩过的坑:`param set` 失败只打印一行
  警告,脚本继续跑,于是"我要跑 600 秒"实际变成节点默认的 120 秒就停,车停在
  场地中间。所以设参数的成功返回**不作为证据**,回读值才是。
* /r680_explorer 不在(或者参数服务不响应) → 拒绝启动,不猜。

用法
----
    python3 r680_start_explore.py --go   --budget 600 [--yes]
    python3 r680_start_explore.py --dry-run --budget 600 [--yes]
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

# 前置检查要等的话题。都是 1 Hz 以上发布,3 s 足够;等不到就是没人发,
# 那本身就是必须拒绝启动的理由(而不是"大概是好的")。
TOPIC_WAIT_S = 3.0


def _fail(msg):
    print(msg)
    return 1


class Starter(Node):
    def __init__(self):
        super().__init__("r680_start_explore")
        self.latest = {}
        self.subs = [
            self.create_subscription(
                Bool, "/safety/localization_valid",
                lambda m: self.latest.__setitem__("localization_valid", m.data), 5),
            self.create_subscription(
                Bool, "/safety/obstacle_stop",
                lambda m: self.latest.__setitem__("obstacle_stop", m.data), 5),
            self.create_subscription(
                Float32, "/battery_voltage",
                lambda m: self.latest.__setitem__("battery_voltage", m.data), 5),
        ]

    def collect(self, timeout):
        """等前置话题到手;返回 False 表示等不到(调用方据此拒绝启动)。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and len(self.latest) < 3:
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.latest) >= 3


def _param(name, value):
    p = Parameter()
    p.name = name
    v = ParameterValue()
    if isinstance(value, bool):
        v.type = ParameterType.PARAMETER_BOOL
        v.bool_value = value
    else:
        v.type = ParameterType.PARAMETER_DOUBLE
        v.double_value = float(value)
    p.value = v
    return p


def _disarm(node):
    """取消时把 dry_run 放回 true(尽力而为,失败只提示不阻断取消)。"""
    client = node.create_client(SetParameters, "/r680_explorer/set_parameters")
    if not client.wait_for_service(timeout_sec=2.0):
        print("  (无法把 dry_run 复位:参数服务不响应)")
        return
    req = SetParameters.Request()
    req.parameters = [_param("dry_run", True)]
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if fut.result() is None:
        print("  (无法把 dry_run 复位:没有返回)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--budget", default="300.0")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    if args.go == args.dry_run:
        return _fail("必须且只能给出 --go 或 --dry-run 之一")

    dry = args.dry_run
    try:
        budget = float(args.budget)
    except (TypeError, ValueError):
        return _fail("预算非法: %r (要秒数)" % (args.budget,))

    rclpy.init()
    node = Starter()
    try:
        print("=== 起探索前的前置检查 ===")
        if not node.collect(TOPIC_WAIT_S):
            missing = [t for t in ("localization_valid", "obstacle_stop",
                                   "battery_voltage") if t not in node.latest]
            return _fail("  ❌ 等不到话题: %s —— 栈没起全,拒绝开始运动"
                         % ", ".join(missing))

        lv = node.latest["localization_valid"]
        os_ = node.latest["obstacle_stop"]
        bv = node.latest["battery_voltage"]
        print("  localization_valid: %s  (需 true)" % ("true" if lv else "false"))
        print("  obstacle_stop:      %s  (需 false)" % ("true" if os_ else "false"))
        print("  battery_voltage:    %.2f V" % bv)

        if args.go and (not lv or os_):
            return _fail("  ❌ 前置条件不满足,拒绝开始运动")

        # ---- 设参数:一次 SetParameters 设全 5 个 ----
        # 关键:这一步和下面的回读校验都在**确认之前**做完。
        # 原来它们在敲完 GO 之后,于是 GO 之后还有 12 次 ros2 命令行进程
        # (每次冷启动 1.0~1.2 s),车要等十几秒才动 —— 看起来像卡死。
        # 现在敲 GO 之后只剩一个 Trigger 调用(同进程内,毫秒级)。
        # 这样安排是安全的:dry_run=false 只是解除"演练",真正开始运动必须靠
        # /exploration/start;节点在 IDLE 状态下等这个调用,不会自己走。
        want = {
            "dry_run": dry,
            "max_exploration_time": float(budget),
            "goal_timeout": 60.0,
            "min_battery": 10.5,
            "battery_timeout": 5.0,
        }
        setter = node.create_client(SetParameters, "/r680_explorer/set_parameters")
        if not setter.wait_for_service(timeout_sec=5.0):
            return _fail("  ❌ /r680_explorer 不在(参数服务无人应答),拒绝启动")

        req = SetParameters.Request()
        req.parameters = [_param(k, v) for k, v in want.items()]
        future = setter.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if future.result() is None:
            return _fail("  ❌ 设参数没有返回,拒绝启动")
        rejected = [r.reason for r in future.result().results if not r.successful]
        if rejected:
            return _fail("  ❌ 参数被拒绝: %s,拒绝启动" % "; ".join(rejected))

        # ---- 回读校验:设参数的成功返回不算证据 ----
        # `ros2 param set` 失败只打印一行警告,脚本照常往下走,于是"我要跑 600 秒"
        # 实际变成节点默认的 120 秒就停。同理 SetParameters 返回 successful 也可能
        # 因为类型不匹配而没生效。回读值才是证据。
        getter = node.create_client(GetParameters, "/r680_explorer/get_parameters")
        if not getter.wait_for_service(timeout_sec=5.0):
            return _fail("  ❌ 参数服务不响应,无法回读校验,拒绝启动")
        greq = GetParameters.Request()
        greq.names = ["max_exploration_time", "dry_run"]
        gfut = getter.call_async(greq)
        rclpy.spin_until_future_complete(node, gfut, timeout_sec=10.0)
        if gfut.result() is None:
            return _fail("  ❌ 回读参数没有返回,拒绝启动")

        values = gfut.result().values
        applied = values[0].double_value
        applied_dry = values[1].bool_value
        # 预算用容差判,不比对字符串:2.0 与 2 是同一个值。
        if abs(applied - budget) > 1e-3:
            print()
            print("  ✗ 预算没有生效:想要 %.1fs,节点上是 %.1fs" % (budget, applied))
            print("    探索未启动。先查 /r680_explorer 是否还活着,再重试。")
            return 1
        if applied_dry != dry:
            print()
            print("  ✗ dry_run 没有生效:想要 %s,节点上是 %s" % (dry, applied_dry))
            print("    探索未启动(否则会以错误的模式驱动底盘)。")
            return 1

        print("  ✅ 参数已就绪:预算 %.1f s,dry_run=%s" % (applied, applied_dry))

        if args.go:
            print()
            print("  ⚠️  即将开始**真实自主运动**")
            print("      限速 0.10 / 0.0 / 0.35,预算 %.1f s" % budget)
            print("      只有满足以下全部条件才继续:")
            print("        - 现场有人,手能碰到物理断电开关")
            print("        - 前方至少 2 m 净空")
            print("        - 另一个终端已经开着: bash tools/r680_explore_watch.sh")
            print("      急停随时可用: bash tools/r680_estop.sh")
            print()
            if args.yes:
                print("  --yes 已给出,跳过人工确认(5 s 后开始,可 Ctrl-C 中止)")
                time.sleep(5)
            else:
                # 提示语单独成行(带换行),输入位只用 "> ":
                # `input("提示")` 的提示是没有换行的,readline 一类的读法永远等不到
                # 它结束 —— 测试里就因此挂过一轮。分开写对人和对机器都更清楚。
                print("  确认继续?输入 go 开始(大小写都行):")
                try:
                    ans = input("  > ")
                except EOFError:
                    ans = ""
                # 大小写都认:旧写法只认全大写 GO,敲小写的人看到的是
                # "已取消(车未动)",而他明明输入了 go。
                if ans.strip().upper() != "GO":
                    # 参数已经设过了,取消时要把它放回演练态,
                    # 免得留下一个 dry_run=false 却没人管的节点。
                    _disarm(node)
                    print("  已取消(车未动)。收到的是 %r;要开始请再跑一次并输入 go"
                          % ans)
                    return 0

        # ---- 启动 ----
        print()
        print("=== 启动探索(预算 %.1f s)%s ===" % (budget, ",演练模式" if dry else ""))
        starter_client = node.create_client(Trigger, "/exploration/start")
        if not starter_client.wait_for_service(timeout_sec=10.0):
            return _fail("  ❌ /exploration/start 不在,探索没有启动")
        sfut = starter_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, sfut, timeout_sec=15.0)
        if sfut.result() is None:
            return _fail("  ❌ 启动服务没有返回(不知道车有没有动),立刻检查车")
        if not sfut.result().success:
            return _fail("  ❌ 启动被拒绝: %s" % sfut.result().message)

        print("  ✅ 已启动:%s" % sfut.result().message)
        print()
        print("  现在开始监视: bash tools/r680_explore_watch.sh")
        print("  急停:         bash tools/r680_estop.sh")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
