#!/usr/bin/env python3
# ============================================================================
# vio_divergence_monitor.py —— 实时监测 RGB-D 视觉里程计是否发散
#
# 用途：
#   在录制 rosbag 期间，把本节点输出直接打印在录制终端里。一旦视觉里程计
#   持续发散，就打印醒目的 “VIO发散” 提示，避免录完一整段才发现 bag 里的
#   /odom 全程是无效零位姿。
#
# 主要信号（优先级从高到低）：
#   1. /rtabmap/odom_info  (rtabmap_msgs/msg/OdomInfo)
#        lost=true 或 inliers < 阈值  → 该帧判定为“坏帧”
#      这是最准确的信号，直接来自 RTAB-Map 的里程计内部状态。
#   2. /odom  (nav_msgs/msg/Odometry) 降级备用
#      仅当收不到 odom_info 时使用：协方差 >= 9999 且四元数为零 → 无效位姿。
#
# 判定逻辑：
#   连续 sustain 秒没有出现任何“好帧”才算发散，避免快速转身、短暂丢帧造成的
#   误报。启动后先静默 grace 秒，因为 RTAB-Map 初始化阶段本身就会丢帧。
#
# 用法：
#   python3 tools/vio_divergence_monitor.py
#   python3 tools/vio_divergence_monitor.py --grace 20 --sustain 3.0
#   python3 tools/vio_divergence_monitor.py --min-inliers 12 --heartbeat 10
# ============================================================================

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo

# 终端颜色；非 tty（例如重定向到日志）时自动关闭，避免日志里出现转义字符。
_TTY = sys.stdout.isatty()


def _c(code: str, text: str) -> str:
    if not _TTY:
        return text
    return f"\033[{code}m{text}\033[0m"


def red(text: str) -> str:
    return _c("1;31", text)


def yellow(text: str) -> str:
    return _c("1;33", text)


def green(text: str) -> str:
    return _c("1;32", text)


def cyan(text: str) -> str:
    return _c("1;36", text)


def stamp() -> str:
    return time.strftime("%H:%M:%S")


class _Tee:
    """把 stdout 同时写到终端和日志文件，便于录完后复查。"""

    def __init__(self, stream, path: str) -> None:
        self.stream = stream
        self.file = open(path, "a", buffering=1, encoding="utf-8")

    def write(self, data: str) -> int:
        self.stream.write(data)
        self.file.write(data)
        return len(data)

    def flush(self) -> None:
        self.stream.flush()
        self.file.flush()

    def close(self) -> None:
        try:
            self.file.close()
        except Exception:
            pass


class VioDivergenceMonitor(Node):
    """订阅里程计状态，判定视觉里程计是否持续发散。"""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("vio_divergence_monitor")
        self.args = args

        self.start_time = time.monotonic()

        # 状态跟踪
        self.last_good_time = None      # 最近一次好帧（单调时钟）
        self.last_msg_time = None       # 最近一次收到任何里程计消息
        self.last_bad_time = None
        self.bad_streak = 0
        self.source = None              # "odom_info" 或 "odom"
        self.state = "STARTING"         # STARTING/DIVERGED/WEAK/HEALTHY/NODATA
        self.last_report_time = 0.0
        self.last_repeat_time = 0.0
        self.diverged_since = None

        # 统计
        self.frames = 0
        self.bad_frames = 0
        self.episodes = 0
        self.latest_inliers = 0
        self.latest_matches = 0
        self.latest_features = 0
        self.latest_lost = False
        self.distance = 0.0
        self.ever_healthy = False

        # odom_info 使用传感器数据 QoS（BEST_EFFORT）更稳妥，同时兼容 RELIABLE。
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(OdomInfo, "/rtabmap/odom_info", self.on_odom_info, qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)

        # 0.5 秒评估一次状态，与消息到达频率解耦。
        self.create_timer(0.5, self.evaluate)

        self.get_logger().info(
            f"VIO 监测已启动（宽限 {args.grace:g}s，连续丢帧 {args.sustain:g}s 判定发散）"
        )

    # -- 回调 ---------------------------------------------------------------
    def on_odom_info(self, msg: OdomInfo) -> None:
        now = time.monotonic()
        self.source = "odom_info"
        self.last_msg_time = now
        self.frames += 1

        self.latest_lost = bool(msg.lost)
        self.latest_inliers = int(msg.inliers)
        self.latest_matches = int(msg.matches)
        self.latest_features = int(msg.features)
        self.distance = float(msg.distance_travelled)

        # 好帧：没有丢失，且内点数达到阈值。
        good = (not msg.lost) and msg.inliers >= self.args.min_inliers
        if good:
            self.last_good_time = now
            self.bad_streak = 0
        else:
            self.bad_frames += 1
            self.bad_streak += 1
            self.last_bad_time = now

    def on_odom(self, msg: Odometry) -> None:
        # 仅在完全没有 odom_info 时才用它作为降级信号。
        if self.source == "odom_info":
            return
        now = time.monotonic()
        self.source = "odom"
        self.last_msg_time = now
        self.frames += 1

        cov = msg.pose.covariance
        q = msg.pose.pose.orientation
        valid = not (cov[0] >= 9999.0 or (q.w == 0.0 and q.z == 0.0))
        if valid:
            self.last_good_time = now
            self.bad_streak = 0
        else:
            self.bad_frames += 1
            self.bad_streak += 1
            self.last_bad_time = now

    # -- 判定 ---------------------------------------------------------------
    def evaluate(self) -> None:
        now = time.monotonic()
        elapsed = now - self.start_time

        # 启动宽限期：RTAB-Map 初始化阶段本来就会丢帧，不做判定。
        if elapsed < self.args.grace:
            self.state = "STARTING"
            return

        if self.last_msg_time is None or (now - self.last_msg_time) > self.args.no_data_timeout:
            self.enter(now, "NODATA")
            self.report(
                now,
                red("VIO无数据"),
                "收不到 /rtabmap/odom_info，也无法从 /odom 判断位姿有效性。"
                "请检查视觉里程计节点是否在运行。",
            )
            return

        if self.last_good_time is None:
            # 从未出现过好帧：宽限期结束后，累计足够坏帧即判定发散。
            bad_run = elapsed - self.args.grace
            if bad_run >= self.args.sustain and self.bad_streak >= self.args.min_bad_frames:
                self.enter(now, "DIVERGED")
                self.report_diverged(now, bad_run)
            else:
                self.enter(now, "WEAK")
            return

        gap = now - self.last_good_time
        # 必须同时满足“持续时间”和“实际坏帧数量”，否则消息短暂中断也会被
        # 误报成发散，并显示上一帧好帧的数值。
        if gap >= self.args.sustain and self.bad_streak >= self.args.min_bad_frames:
            self.enter(now, "DIVERGED")
            self.report_diverged(now, gap)
            return

        # 恢复判定
        if self.state == "DIVERGED":
            self.ever_healthy = True
            held = now - (self.diverged_since or now)
            self.diverged_since = None
            self.state = "HEALTHY"
            self.report(
                now,
                green("VIO已恢复"),
                f"重新获得有效位姿（本次发散持续约 {held:.1f}s，"
                f"内点={self.latest_inliers} 特征={self.latest_features}）",
            )
            return

        if self.last_bad_time is not None and (now - self.last_bad_time) <= 1.0:
            # 最近仍有坏帧但未达到发散阈值。
            self.enter(now, "WEAK")
            self.report(
                now,
                yellow("VIO不稳定"),
                f"lost={self.latest_lost} 内点={self.latest_inliers} "
                f"特征={self.latest_features}（尚未判定为发散）",
            )
            return

        self.enter(now, "HEALTHY")
        self.ever_healthy = True
        self.report(
            now,
            green("VIO正常"),
            f"内点={self.latest_inliers} 匹配={self.latest_matches} "
            f"特征={self.latest_features} 累计里程={self.distance:.2f}m",
        )

    def enter(self, now: float, state: str) -> None:
        """记录状态切换；进入发散状态时计数一次。"""
        if state == "DIVERGED" and self.state != "DIVERGED":
            self.episodes += 1
            self.diverged_since = now
            # 事件首次出现必须立即打印，不受心跳节流影响。
            self.last_report_time = 0.0
            self.last_repeat_time = 0.0
        self.state = state

    def report_diverged(self, now: float, gap: float) -> None:
        """首次发散打印完整提示，之后按较长间隔打印精简行，避免刷屏。"""
        if (now - self.last_report_time) < 1.0:
            return
        detail = (
            f"lost={self.latest_lost} 内点={self.latest_inliers} "
            f"匹配={self.latest_matches} 特征={self.latest_features} "
            f"累计里程={self.distance:.2f}m"
        )
        first = (now - self.last_repeat_time) > self.args.repeat * 100
        if first:
            self.last_report_time = now
            print(f"[{stamp()}] {red('⚠️  VIO发散')}", flush=True)
            print(f"          已连续 {gap:.1f}s 没有有效位姿（{detail}）", flush=True)
            print(
                "          相机几乎没有可用位姿。请放慢移动、避免长时间对着白墙/窗户/地面，"
                "并确认画面有纹理和光照；",
                flush=True,
            )
            print(
                "          若持续发散，这段 rosbag 无法用于离线建图。", flush=True
            )
            self.last_repeat_time = now
            return
        if (now - self.last_repeat_time) >= self.args.repeat:
            self.last_repeat_time = now
            self.last_report_time = now
            print(
                f"[{stamp()}] {red('⚠️  VIO发散')} "
                f"仍在发散 {gap:.0f}s（{detail}）",
                flush=True,
            )

    def report(self, now: float, headline: str, body: str) -> None:
        """按心跳间隔节流输出常规状态。"""
        if (now - self.last_report_time) < self.args.heartbeat:
            return
        self.last_report_time = now
        print(f"[{stamp()}] {headline}", flush=True)
        for line in body.splitlines():
            print(f"          {line}", flush=True)

    def summary(self) -> None:
        print("", flush=True)
        print(cyan("---- VIO 监测汇总 ----"), flush=True)
        print(f"  信号来源    : {self.source or '无'}", flush=True)
        print(f"  收到帧数    : {self.frames}", flush=True)
        print(f"  坏帧数      : {self.bad_frames}", flush=True)
        print(f"  发散次数    : {self.episodes}", flush=True)
        print(f"  累计里程    : {self.distance:.2f} m", flush=True)
        if self.state == "DIVERGED":
            print(f"  结束状态    : {red('发散中')}", flush=True)
        elif not self.ever_healthy:
            print(f"  结束状态    : {red('从未获得有效位姿')}", flush=True)
        else:
            print(f"  结束状态    : {green('曾正常跟踪')}", flush=True)
        if self.distance < 0.5 and self.frames > 0:
            print(
                yellow(
                    "  警告        : 累计里程不足 0.5m，该 rosbag 可能无法用于离线建图。"
                ),
                flush=True,
            )
        print(cyan("----------------------"), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实时监测 RGB-D 视觉里程计是否发散。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--grace", type=float, default=20.0,
                        help="启动后的静默宽限时间（秒）")
    parser.add_argument("--sustain", type=float, default=3.0,
                        help="连续多少秒没有有效位姿才判定发散")
    parser.add_argument("--min-inliers", type=int, default=10,
                        help="判定为有效位姿所需的最少内点数")
    parser.add_argument("--min-bad-frames", type=int, default=10,
                        help="判定发散前需要累计的连续坏帧数")
    parser.add_argument("--no-data-timeout", type=float, default=5.0,
                        help="多久收不到里程计消息就报告无数据（秒）")
    parser.add_argument("--heartbeat", type=float, default=15.0,
                        help="正常状态下的心跳打印间隔（秒）")
    parser.add_argument("--repeat", type=float, default=10.0,
                        help="持续发散时的精简提醒间隔（秒）")
    parser.add_argument("--log", type=str, default="",
                        help="同时把监测输出写入该日志文件")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    tee = None
    if args.log:
        tee = _Tee(sys.stdout, args.log)
        sys.stdout = tee
    rclpy.init()
    node = VioDivergenceMonitor(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM 退出属于正常停止，不打印堆栈。
        pass
    finally:
        try:
            node.summary()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if tee is not None:
            sys.stdout = tee.stream
            tee.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
