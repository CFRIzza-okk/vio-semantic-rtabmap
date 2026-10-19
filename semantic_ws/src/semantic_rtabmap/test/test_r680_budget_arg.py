#!/usr/bin/env python3
"""预算必须真的生效,而且不能靠"静默沿用节点默认值"蒙混过去。

这个守卫要防的两个缺陷
----------------------
1. **整数写法被 DOUBLE 参数拒掉,失败只有一行警告。** 实测:

       bash tools/r680_explore_300.sh --yes --budget 300
         -> Setting parameter failed: Wrong parameter type,
            expected 'Type.DOUBLE' got 'Type.INTEGER'
         -> 脚本继续跑,车真的动了,但预算是节点默认的 120.0

   而 `r680_explore_300.sh --help` 当时推荐的正是 `--budget 600` 这种坏写法。
   一次自主探索带着没人知道的预算开跑,后果是车停在场地中间。

2. **校验自己会被空值绕过。** 第一版修法是"设完 param 再回读比对",写法是

       case "$APPLIED" in *"$EXPLORE_TIME"*) ;; *) 报错退出 ;; esac

   但 `EXPLORE_TIME` 一旦为空,`*""*` 匹配**任何**字符串,校验永远通过。
   而它真的会为空:`cd` 之后再 `source "$(dirname "$0")/r680_budget.sh"` 解析到
   新工作目录,source 静默失败(脚本没有 set -e),函数不存在 -> 预算变成空串。
   实测就是这样漏的。所以现在的校验先判空/判非法,再比对回读值。

怎么测
------
不查字符串,而是**真的把脚本跑起来**:用 PATH 前面放一个假的 `ros2` 记录它被问了
什么、并报告节点上的值。先 source ROS 的 setup.bash 再插桩,是因为 setup.bash
只在"尚未存在"时才前插自己的 bin,这样桩才能压住真的 ros2。

干跑模式(--dry-run)不驱动底盘、不需要真栈。
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOLS = REPO_ROOT / "tools"
START = TOOLS / "r680_explore_start.sh"
ONEKEY = TOOLS / "r680_explore_300.sh"
HELPER = TOOLS / "r680_budget.sh"
ROS_SETUP = "/opt/ros/humble/setup.bash"

STUB = r"""#!/usr/bin/env bash
echo "$@" >> "$STUB_DIR/calls.log"
case "$1 $2" in
  "topic echo") case "$3" in
      */localization_valid) echo "data: true" ;;
      */obstacle_stop)      echo "data: false" ;;
      */battery_voltage)    echo "data: 12.1" ;;
      *) echo "data: true" ;;
    esac ;;
  "param set") echo "Set parameter successful" ;;
  "param get") case "$4" in
      max_exploration_time) echo "Double value is: $STUB_BUDGET_REPORT" ;;
      *) echo "Boolean value is: true" ;;
    esac ;;
  "service call") echo "success=True" ;;
  "node list")    echo "/r680_explorer" ;;
  *) echo "(stub: $*)" ;;
esac
"""

pytestmark = pytest.mark.skipif(
    not Path(ROS_SETUP).exists(), reason="needs the ROS 2 environment to source"
)


def _prepare_stub(tmp_path):
    stub_dir = tmp_path / "stub"
    stub_dir.mkdir()
    ros2 = stub_dir / "ros2"
    ros2.write_text(STUB)
    ros2.chmod(0o755)
    return stub_dir


def _run_script(script, args, tmp_path, budget_report, explore_time=None):
    """Run the real script against the stub ros2. Returns (rc, output, calls)."""
    stub_dir = _prepare_stub(tmp_path)
    calls = stub_dir / "calls.log"
    if calls.exists():
        calls.unlink()

    env = dict(os.environ)
    env["STUB_DIR"] = str(stub_dir)
    env["STUB_BUDGET_REPORT"] = budget_report
    if explore_time is not None:
        env["EXPLORE_TIME"] = explore_time

    # 先 source ROS(标记 ROS 的 bin "已存在"),再把桩插到最前面。
    inner = (
        'source "%s" >/dev/null 2>&1; export PATH="%s:$PATH"; '
        'bash "%s" %s' % (ROS_SETUP, stub_dir, script, " ".join(args))
    )
    proc = subprocess.run(["bash", "-c", inner], cwd=str(REPO_ROOT), env=env,
                          capture_output=True, text=True, timeout=180)
    recorded = calls.read_text() if calls.exists() else ""
    return proc.returncode, proc.stdout + proc.stderr, recorded


# --------------------------------------------------------------------------
# 1. 规范化函数本身
# --------------------------------------------------------------------------
@pytest.mark.parametrize("given,expected", [
    ("300", "300.0"),
    ("600", "600.0"),
    ("300.0", "300.0"),
    ("0.5", "0.5"),
    ("120", "120.0"),
])
def test_helper_makes_the_budget_float_shaped(given, expected):
    out = subprocess.run(
        ["bash", "-c", 'source "%s"; r680_float_arg "%s"' % (HELPER, given)],
        capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    assert out.stdout == expected, (
        "%s 应该被规范成 %s,实际是 %s —— 整数字面量会被 DOUBLE 参数拒掉"
        % (given, expected, out.stdout)
    )


# --------------------------------------------------------------------------
# 2. 脚本真的把浮点形态发出去了
# --------------------------------------------------------------------------
# 2026-10-18:起点脚本把"设参数 + 回读校验 + 启动"交给了
# tools/r680_start_explore.py 一个进程完成。原因:原来 GO 之后要发起 12 次
# `ros2` 命令行(5 次 param set、5 次 param get、1 次回读、1 次 service call),
# 每次冷启动实测 1.0~1.2 s,叠起来十几秒 —— 敲了 GO 车却不动。
# 所以这里桩的对象从 `ros2` 换成了 `python3`:断言脚本仍把预算**规范化成浮点**
# 再传下去。回读校验本身在辅助程序里,由 test_r680_go_starts_immediately.py
# 用真节点覆盖(包括"参数被拒就不启动")。
PYTHON_STUB = r"""#!/usr/bin/env bash
echo "$@" >> "$STUB_DIR/py.log"
exit ${STUB_PY_RC:-0}
"""


def _prepare_py_stub(tmp_path):
    d = tmp_path / "pystub"
    d.mkdir()
    exe = d / "python3"
    exe.write_text(PYTHON_STUB)
    exe.chmod(0o755)
    return d


def _run_start_with_py_stub(args, tmp_path, explore_time, py_rc="0", cwd=None):
    d = _prepare_py_stub(tmp_path)
    env = dict(os.environ)
    env["STUB_DIR"] = str(d)
    env["EXPLORE_TIME"] = explore_time
    env["STUB_PY_RC"] = py_rc
    inner = ('source "%s" >/dev/null 2>&1; export PATH="%s:$PATH"; '
             'bash "%s" %s' % (ROS_SETUP, d, START, " ".join(args)))
    proc = subprocess.run(["bash", "-c", inner], cwd=cwd or str(REPO_ROOT), env=env,
                          capture_output=True, text=True, timeout=180)
    log = d / "py.log"
    return proc.returncode, proc.stdout + proc.stderr, (log.read_text() if log.exists() else "")


def test_start_script_asks_the_node_for_a_float_budget(tmp_path):
    rc, out, calls = _run_start_with_py_stub(["--dry-run"], tmp_path,
                                            explore_time="600")
    assert "--budget 600.0" in calls, (
        "脚本没有把预算规范化成浮点再传下去;它发出的调用是:\n%s" % calls
    )
    assert rc == 0, "预算生效时脚本不该失败:\n%s" % out


def test_start_script_fails_when_the_helper_fails(tmp_path):
    """辅助程序说"预算没生效"时,脚本必须把失败传出去,而不是吞掉。

    回读比对本身在 r680_start_explore.py 里(那里拿真节点的值跟请求值比,
    不相等就 exit 1);这里守住的是脚本侧:**退出码必须传出去**。
    如果脚本吞掉失败返回 0,操作员会以为车已经在跑。
    """
    rc, out, _ = _run_start_with_py_stub(["--dry-run"], tmp_path,
                                         explore_time="600", py_rc="1")
    assert rc != 0, (
        "辅助程序失败了(预算没生效),脚本却返回 0 —— 操作员会以为车在跑:\n%s" % out
    )


def test_start_script_rejects_a_garbage_budget(tmp_path):
    """垃圾预算必须当场拒绝:空/非法值会让回读比对永远通过。"""
    rc, out, _ = _run_script(START, ["--dry-run"], tmp_path,
                             budget_report="120.0", explore_time="abc")
    assert rc != 0, "非法的预算值被放行了:\n%s" % out
    assert "非法" in out or "预算" in out, out


def test_start_script_runs_from_any_directory(tmp_path):
    """相对路径 source 过 helper 的写法,换目录就会静默失效(本缺陷的成因)。"""
    rc, out, recorded = _run_start_with_py_stub(["--dry-run"], tmp_path,
                                                explore_time="300", cwd="/tmp")
    assert "--budget 300.0" in recorded, (
        "从别的目录调用时预算没传下去(相对路径 source 失效):\n%s" % recorded
    )
    assert rc == 0, out


# --------------------------------------------------------------------------
# 3. 一键脚本也走同一套
# --------------------------------------------------------------------------
def test_onekey_script_normalises_and_validates():
    source = ONEKEY.read_text()
    assert "r680_float_arg" in source, "一键脚本没走预算规范化"
    assert "r680_budget.sh" in source, "一键脚本没引用预算 helper"
    # 必须是绝对路径算出来的目录:cd 之后 $(dirname "$0") 会指错地方。
    assert 'TOOLS_DIR="$(cd "$(dirname "$0")" && pwd)"' in source, (
        "一键脚本用相对路径引用 helper:一旦先 cd 过,source 会静默失败"
    )
