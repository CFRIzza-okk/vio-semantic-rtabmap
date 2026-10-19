#!/usr/bin/env python3
"""`go` 必须一句话就把车开动,而且不能靠"跳过检查"来省时间。

要防的两个缺陷
--------------
1. **省时间省掉了安全的那个版本。** 最省事的写法是让 --fast 直接跳到起跑,
   不管栈在不在。那样在栈只起了一半时会走到 r680_explore_start.sh,它对
   `localization_valid` / `obstacle_stop` 拿不到值就判前置条件不满足,
   表现是"敲了 go,什么也没发生,只有一行拒绝" —— 恰恰是这次要修的体感。
   更糟的分支是闸门还关着:车看着起来了,/cmd_vel 却没有发布者,一步不动。

2. **`--fast` 其实还在重建整栈。** 那样"敲了 go 立刻动"就没实现,
   而且浪费 3~4 分钟 —— 用户会绕过脚本手敲 ros2 param set,比什么都危险。

怎么测
------
不查字符串定义,而是**真的把脚本跑起来**:PATH 前面放一个假的 `pgrep`,
让它报告"一个节点都没找到"(等价于栈没起)。于是:

    期望: 走到"栈不完整"分支 -> 非零退出 -> 且从未打印"整栈启动"、从未碰闸门

这两条断言分别盯住上面两个缺陷:
    * 若 --fast 仍调 up.sh  -> 出现"1/3 整栈启动" -> 失败
    * 若省掉了完整性校验     -> 继续往下开闸 -> 出现"闸门"字样/退出码 0 -> 失败

用假 pgrep 而不是限制真实环境,是因为脚本用的就是 pgrep:桩在这里是
"让真实分支可达",不是替身实现。全程不驱动底盘、不需要真栈。
"""
import os
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOLS = REPO_ROOT / "tools"
ONEKEY = TOOLS / "r680_explore_300.sh"
GO = TOOLS / "go.sh"
ROS_SETUP = "/opt/ros/humble/setup.bash"

# 栈没起来:任何 pgrep 都找不到东西。
PGREP_STUB = """#!/usr/bin/env bash
echo "pgrep $@" >> "$STUB_DIR/pgrep.log"
exit 1
"""


def _run(script, args, tmp_path):
    """跑真脚本,pgrep 桩报告"栈不完整"。返回 (rc, 输出, pgrep 调用记录)。"""
    stub_dir = tmp_path / "stub"
    stub_dir.mkdir()
    pgrep = stub_dir / "pgrep"
    pgrep.write_text(PGREP_STUB)
    pgrep.chmod(0o755)
    log = stub_dir / "pgrep.log"

    env = dict(os.environ)
    env["STUB_DIR"] = str(stub_dir)
    env["VIO"] = str(REPO_ROOT)
    # 干净域:万一某个分支真的走到 ros2,也不会碰到在跑的那套栈。
    env["ROS_DOMAIN_ID"] = "77"

    inner = (
        'export PATH="%s:$PATH"; bash "%s" %s' % (stub_dir, script, " ".join(args))
    )
    proc = subprocess.run(
        ["bash", "-c", inner],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )
    return proc.returncode, proc.stdout + proc.stderr, log.read_text() if log.exists() else ""


@pytest.mark.skipif(not ONEKEY.exists(), reason="缺少一键脚本")
def test_fast_skips_the_bring_up_and_checks_the_stack_first(tmp_path):
    """--fast 不重建整栈,但必须校验;校验不过就退出,绝不碰闸门。"""
    rc, out, pgrep_log = _run(str(ONEKEY), ["--fast", "--budget", "60"], tmp_path)

    assert "1/3 整栈启动" not in out, (
        "--fast 仍然在重建整栈,'敲了 go 立刻动'就没实现:\n%s" % out
    )
    assert rc != 0, "栈不完整时必须以非零码退出,实际 rc=%d:\n%s" % (rc, out)
    assert "栈不完整" in out, "没有明确告诉操作员缺什么:\n%s" % out
    # 缺的节点要逐个点名,否则操作员不知道该跑 up.sh 还是查别处。
    for node in ("rtabmap", "controller_server", "bt_navigator", "exploration_node"):
        assert node in out, "缺失清单里没有 %s:\n%s" % (node, out)
    # 校验不过就绝不能开闸 —— 开了闸却起不来的车,是最糟的中间态。
    assert "闸门" not in out, "校验失败却已经去碰闸门了:\n%s" % out
    assert pgrep_log.strip(), "根本没做完整性检查(pgrep 一次都没被调用)"


@pytest.mark.skipif(not GO.exists(), reason="缺少 go.sh")
def test_go_routes_through_the_fast_path_with_a_budget(tmp_path):
    """`go` 必须走 --fast,并把预算传下去。"""
    rc, out, _ = _run(str(GO), ["240"], tmp_path)

    assert "跳过整栈重建" in out, (
        "go.sh 没有走 --fast 分支(r680_explore_300.sh 的 fast 模式没被启用):\n%s" % out
    )
    assert "1/3 整栈启动" not in out, "go.sh 竟然重建了整栈:\n%s" % out
    # 预算要真的传到了编排层,不是停在 go.sh 里。
    assert "240" in out, "预算没有传下去:\n%s" % out
    # 栈不完整时同样必须退出,不能"起了再说"。
    assert rc != 0, "栈不完整时 go.sh 必须以非零码退出,实际 rc=%d" % rc


# ---------------------------------------------------------------------------
# 交互确认那部分搬到 test_r680_go_starts_immediately.py 了。
#
# 原来这里用假的 `ros2` 命令行桩去验证"敲 go 会不会开动"。2026-10-18 起,
# 确认之后的设参数/回读/启动由 tools/r680_start_explore.py **一个进程**完成
# (原来 12 次命令行进程,每次 1 s 以上,敲了 GO 车要等十几秒),命令行的桩
# 就再也碰不到那段逻辑了 —— 留着只会是一组永远"通过"的假守卫。现在的同一批
# 行为由那个文件用真 rclpy 节点覆盖:真参数服务、真 start 服务、真计时。
