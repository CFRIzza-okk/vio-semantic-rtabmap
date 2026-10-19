#!/usr/bin/env python3
"""不要在 rtabmap 活着的时候读它的库 —— 那会把车上的建图进程读死。

2026-10-17 实测的现场(代价是一个 rtabmap 进程):
    [FATAL] DBDriverSqlite3.cpp:5034::addStatisticsQuery()
            Condition (rc == SQLITE_DONE) not met! [DB error: database is locked]
    terminate called after throwing an instance of 'UException'
    [ERROR] [rtabmap-1]: process has died [pid 74847, exit code -6]

成因:盘点工具对每个库跑完整性检查,会长时间持有读锁;而 rtabmap 每插一个节点都要
写库,SQLite 回滚日志模式下写需要 EXCLUSIVE 锁 —— 读锁不放,它的写就拿到
SQLITE_BUSY,RTAB-Map 把这个当致命错误直接 abort。

地图数据本身没坏(逐项对比:164 节点 / 194 链接 / 164 份深度数据全在,integrity ok,
也没有残留 journal),但"我只是看一眼库"不该把建图进程看死。所以 scan_rtabmap_dbs.py
现在会先看 /proc,发现某个待扫的库正被活着的 rtabmap 打开就高声警告。

这两个测试都真的起一个进程来验,不查字符串。
"""
import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
TOOL = REPO_ROOT / "tools" / "scan_rtabmap_dbs.py"


def _load_tool():
    spec = importlib.util.spec_from_file_location("scan_rtabmap_dbs", TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fake_rtabmap_holding(db_path):
    """起一个进程,它的 argv 看起来就像 rtabmap 打开着这个库。"""
    return subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)",
         "rtabmap", str(db_path)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def test_detects_a_live_rtabmap_holding_the_db(tmp_path):
    db = tmp_path / "held.db"
    db.write_bytes(b"")
    proc = _fake_rtabmap_holding(db)
    try:
        time.sleep(1.5)
        live = _load_tool().live_rtabmap_dbs()
        assert os.path.realpath(str(db)) in live, (
            "没认出正被活着的 rtabmap 打开的库;读它就会把它写崩。"
            "识别到的: %r" % (live,)
        )
    finally:
        proc.kill()
        proc.wait()


def test_silent_when_no_rtabmap_is_running(tmp_path):
    db = tmp_path / "idle.db"
    db.write_bytes(b"")
    live = _load_tool().live_rtabmap_dbs()
    assert os.path.realpath(str(db)) not in live, (
        "没有 rtabmap 在跑,却把它当成「活的库」了 —— 误报会让人无视警告"
    )


def test_scan_warns_loudly_about_a_live_db(tmp_path, capsys):
    """用户看得到的那句话必须出现:光有检测不算防护。"""
    db = tmp_path / "held.db"
    db.write_bytes(b"")
    proc = _fake_rtabmap_holding(db)
    try:
        time.sleep(1.5)
        module = _load_tool()
        sys.argv = ["scan_rtabmap_dbs.py", "--dirs", str(tmp_path)]
        module.main()
        out = capsys.readouterr().out
    finally:
        proc.kill()
        proc.wait()
    assert "正在运行" in out, "扫描活库时没有发出警告:\n%s" % out
    assert "database is locked" in out, "警告没说清后果"
    assert "r680_down.sh" in out, "警告没给出该怎么做"
