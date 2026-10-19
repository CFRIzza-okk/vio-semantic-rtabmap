#!/usr/bin/env python3
# ============================================================================
# scan_rtabmap_dbs.py —— 盘点本机所有 RTAB-Map 数据库
#
# 用途：
#   建图数据经常散落在 ~/.ros/、备份目录、以及 rtabmap_*.db.bak.* 这类
#   自动备份里。本工具把它们全部找出来，逐个体检并打印一张表，回答
#   “我到底存了哪些数据、哪些还能用”。
#
# 每个库会检查：
#   大小、节点数、轨迹范围、累计里程、回环数、地标数
#   Data / Feature 表是否完好（决定有没有图像和 3D 点云）
#   SQLite 完整性
#
# 注意：`ls ~/.ros/*.db` 匹配不到 `.db.bak.*`，容易误判成“没有数据”。
#
# 用法：
#   python3 tools/scan_rtabmap_dbs.py
#   python3 tools/scan_rtabmap_dbs.py --dirs ~/.ros ~/VIO_backup_20260929_220415/.ros
#   python3 tools/scan_rtabmap_dbs.py --all        # 额外扫描整个家目录（较慢）
# ============================================================================

import argparse
import glob
import os
import sqlite3
import struct
import sys

DEFAULT_DIRS = [
    "~/.ros",
    "~/VIO/data/bags",
    "~/VIO_backup_20260929_220415/.ros",
]

# 无意义的系统数据库
SKIP_PARTS = (".cache", ".config", ".local/share", ".pki", ".var", "node_modules")


def decode_translation(blob):
    if not blob or len(blob) < 48:
        return None
    v = struct.unpack("<12f", blob[:48])
    return v[3], v[7], v[11]


def table_status(conn, table, nodes):
    """判断 Data/Feature 是否可用。

    不能只用 `select * from T limit 1`：在空表上它不触碰损坏页就返回成功，
    会把没有点云的库误报成“有”。必须走 count(*)，并按节点数判断是否缺失。
    """
    try:
        n = conn.execute(f'select count(*) from "{table}"').fetchone()[0]
    except sqlite3.DatabaseError:
        return "损坏", 0
    try:
        conn.execute(f'select * from "{table}"').fetchall()
    except sqlite3.DatabaseError:
        return "损坏", n
    if nodes > 0 and n == 0:
        return "缺失", n
    return ("有" if n > 0 else "空"), n


def inspect(path):
    info = {"path": path, "size_mb": os.path.getsize(path) / 1048576}
    try:
        conn = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    except sqlite3.DatabaseError as exc:
        info["error"] = f"无法打开: {exc}"
        return info
    try:
        info["nodes"] = conn.execute("select count(*) from Node").fetchone()[0]
    except sqlite3.DatabaseError as exc:
        info["error"] = f"Node 表不可读: {exc}"
        conn.close()
        return info

    if info["nodes"] == 0:
        conn.close()
        return info

    try:
        xs, ys, zs = [], [], []
        for (blob,) in conn.execute("select pose from Node"):
            t = decode_translation(blob)
            if t:
                xs.append(t[0]); ys.append(t[1]); zs.append(t[2])
        if xs:
            info["extent"] = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            info["path_len"] = sum(
                ((xs[i + 1] - xs[i]) ** 2 + (ys[i + 1] - ys[i]) ** 2) ** 0.5
                for i in range(len(xs) - 1)
            )
    except sqlite3.DatabaseError:
        pass

    try:
        info["links"] = conn.execute("select count(*) from Link").fetchone()[0]
        info["closures"] = conn.execute(
            "select count(*) from Link where type in (1,2)"
        ).fetchone()[0]
        info["landmarks"] = conn.execute(
            "select count(distinct to_id) from Link where to_id<0"
        ).fetchone()[0]
    except sqlite3.DatabaseError:
        pass

    info["data_status"], info["data_rows"] = table_status(conn, "Data", info["nodes"])
    info["feature_status"], info["feature_rows"] = table_status(
        conn, "Feature", info["nodes"]
    )
    try:
        info["integrity"] = conn.execute("pragma quick_check").fetchone()[0]
    except sqlite3.DatabaseError as exc:
        info["integrity"] = f"损坏: {exc}"
    conn.close()
    return info


def live_rtabmap_dbs():
    """返回"正被活着的 rtabmap 打开"的库路径集合。

    为什么需要它(2026-10-17 实测,代价是车上一个 rtabmap 进程):
    本工具会对每个库跑完整性检查,也就是长时间持有读锁。而 rtabmap 每个节点都要
    写库,SQLite 回滚日志模式下写操作需要 EXCLUSIVE 锁 —— 读锁不放手,它的写就
    拿到 SQLITE_BUSY,RTAB-Map 把 `addStatisticsQuery()` 的这个错误当致命错误,
    直接 abort(exit code -6)。日志原话:

        [FATAL] DBDriverSqlite3.cpp:5034::addStatisticsQuery()
                Condition (rc == SQLITE_DONE) not met! [DB error: database is locked]
        terminate called after throwing an instance of 'UException'

    地图数据本身没事(出事前的事务都已提交,逐项对比 164 节点 / 194 链接 / 164 份
    深度数据全在),但"我只是看一眼库"不该把车上的建图进程看死。
    """
    open_dbs = set()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as fh:
                cmd = fh.read().decode("utf-8", "replace")
        except OSError:
            continue
        if "rtabmap" not in cmd:
            continue
        for tok in cmd.split("\0"):
            if tok.endswith(".db") or ".db?" in tok:
                open_dbs.add(os.path.realpath(os.path.expanduser(tok)))
    return open_dbs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="盘点本机所有 RTAB-Map 数据库。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--dirs", nargs="*", default=None,
                        help="要扫描的目录（默认若干常用位置）")
    parser.add_argument("--all", action="store_true",
                        help="额外扫描整个家目录（较慢）")
    args = parser.parse_args()

    dirs = args.dirs if args.dirs else list(DEFAULT_DIRS)
    patterns = []
    for d in dirs:
        d = os.path.expanduser(d)
        patterns.append(os.path.join(d, "**", "*.db"))
        patterns.append(os.path.join(d, "**", "*.db.bak.*"))
    if args.all:
        patterns = [os.path.expanduser("~/**/*.db"), os.path.expanduser("~/**/*.db.bak.*")]

    found = set()
    for pat in patterns:
        for p in glob.glob(pat, recursive=True):
            if any(part in p for part in SKIP_PARTS):
                continue
            if os.path.isfile(p):
                found.add(os.path.realpath(p))

    if not found:
        print("没有找到任何 .db 文件。")
        return 0

    live = [p for p in sorted(found) if p in live_rtabmap_dbs()]
    if live:
        print("!! 有 rtabmap 正在运行,而且正打开着下面这些库:")
        for p in live:
            print("     %s" % p)
        print("   现在读它会给它加读锁,而它每插一个节点都要写库(SQLite 回滚日志模式下")
        print("   写需要 EXCLUSIVE 锁)。读锁没放,它的写就会拿到 SQLITE_BUSY,而")
        print("   RTAB-Map 把这个当成致命错误直接 abort —— 实测就是这样:")
        print("     [FATAL] DBDriverSqlite3.cpp addStatisticsQuery() database is locked")
        print("     terminate called after throwing an instance of 'UException'")
        print("     process has died [exit code -6]")
        print("   地图数据不会坏(崩溃前已提交),但车上的建图进程会当场没掉。")
        print("   请先停栈(bash tools/r680_down.sh),或改为扫描一份快照。\n")

    rows = [inspect(p) for p in sorted(found)]
    useful = [r for r in rows if r.get("nodes", 0) > 0]
    empty = [r for r in rows if r.get("nodes", 0) == 0 and "error" not in r]
    broken = [r for r in rows if "error" in r]

    print(f"扫描到 {len(rows)} 个数据库：有内容 {len(useful)}，空库 {len(empty)}，"
          f"无法读取 {len(broken)}\n")

    if useful:
        print("=" * 118)
        print("有内容的数据库（按节点数排序）")
        print("=" * 118)
        header = (f"{'节点':>6s} {'大小MB':>8s} {'范围(m)':>18s} {'里程m':>7s} "
                  f"{'回环':>6s} {'地标':>5s} {'图像/点云':>10s} {'完整性':>8s}  文件")
        print(header)
        print("-" * 118)
        for r in sorted(useful, key=lambda x: -x["nodes"]):
            ext = r.get("extent")
            ext_s = f"{ext[0]:.2f}x{ext[1]:.2f}x{ext[2]:.2f}" if ext else "?"
            if r.get("data_status") == "有" and r.get("feature_status") == "有":
                cloud = "有"
            elif "损坏" in (r.get("data_status"), r.get("feature_status")):
                cloud = "无(损坏)"
            else:
                cloud = "无(缺失)"
            integrity = r.get("integrity", "?")
            integrity = "ok" if integrity == "ok" else "损坏"
            print(f"{r['nodes']:>6d} {r['size_mb']:>8.1f} {ext_s:>18s} "
                  f"{r.get('path_len', 0):>7.1f} {r.get('closures', 0):>6d} "
                  f"{r.get('landmarks', 0):>5d} {cloud:>10s} {integrity:>8s}  "
                  f"{os.path.basename(r['path'])}")
            print(f"{'':>7s}{'':>9s}{'':>19s}{'':>8s}{'':>7s}{'':>6s}{'':>11s}{'':>9s}  "
                  f"  {r['path']}")
        print()

    if broken:
        print("无法读取：")
        for r in broken:
            print(f"  {r['path']}\n      {r['error']}")
        print()

    if empty:
        print(f"空库 {len(empty)} 个（建图失败或刚开始就结束，可安全删除）：")
        for r in empty[:15]:
            print(f"  {os.path.basename(r['path'])}  ({r['size_mb']:.1f} MB)")
        if len(empty) > 15:
            print(f"  ... 另有 {len(empty)-15} 个")
        print()

    good = [r for r in useful
            if r.get("data_status") == "有" and r.get("feature_status") == "有"
            and r.get("integrity") == "ok"]
    if good:
        print("推荐查看（完整可用）：")
        for r in sorted(good, key=lambda x: -x["nodes"])[:5]:
            print(f"  rtabmap-databaseViewer {r['path']}")
        print(f"\n导出轨迹：python3 tools/export_rtabmap_map.py <db> --out 轨迹.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
