#!/usr/bin/env python3
# ============================================================================
# recover_rtabmap_db.py —— 从损坏的 RTAB-Map 数据库中抢救可读数据
#
# 背景：
#   RTAB-Map 的 .db 如果在写入过程中被复制（例如建图还没结束就 cp/rsync），
#   会出现 “database disk image is malformed”，rtabmap-databaseViewer 直接崩溃。
#   常见情况是只有 Data / Feature / Statistics 这几张大表损坏，而
#   Node（位姿）、Link（回环与地标）、Word（词袋）仍然完好。
#
# 本工具把仍然可读的表复制到一个新数据库，得到一个 integrity_check 通过、
#   可以被 databaseViewer 打开的库。
#
# 能救回什么：
#   Node   每个关键帧的位姿和时间戳  → 轨迹
#   Link   邻接、回环、语义地标链接  → 位姿图、回环、地标观测
#   Word   视觉词袋
#
# 救不回什么：
#   Data        每个节点的图像/深度/标定 → 没有节点图像
#   Feature     每个节点的视觉特征      → 没有 3D 点云
#   Statistics  每个节点的耗时统计
#   因此恢复后的库只能看轨迹、回环和地标，看不到点云。
#
# 用法：
#   python3 tools/recover_rtabmap_db.py 损坏的.db 恢复后的.db
#   python3 tools/recover_rtabmap_db.py 损坏的.db 恢复后的.db --force
# ============================================================================

import argparse
import os
import sqlite3
import sys

# 按重要性排序；只有真正可读的表才会被复制。
CANDIDATE_TABLES = [
    "Node",
    "Link",
    "Word",
    "GlobalDescriptor",
    "Info",
    "Admin",
    "Statistics",
    "Data",
    "Feature",
]


def table_is_readable(conn: sqlite3.Connection, table: str):
    """逐行读取整表内容，判断是否真的可读（count(*) 走索引，可能掩盖损坏）。"""
    try:
        cols = [r[1] for r in conn.execute(f"pragma table_info({table})")]
        if not cols:
            return False, 0, None
        rows = conn.execute(f'select * from "{table}"').fetchall()
        return True, len(rows), None
    except sqlite3.DatabaseError as exc:
        return False, 0, str(exc)


def copy_table(src: sqlite3.Connection, dst: sqlite3.Connection, table: str) -> int:
    cols = [r[1] for r in src.execute(f"pragma table_info({table})")]
    placeholders = ",".join("?" * len(cols))
    names = ",".join(f'"{c}"' for c in cols)
    rows = src.execute(f'select * from "{table}"').fetchall()
    dst.executemany(f'insert into "{table}" ({names}) values ({placeholders})', rows)
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="从损坏的 RTAB-Map 数据库中抢救可读数据。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("source", help="损坏的 .db 文件")
    parser.add_argument("dest", help="恢复后写入的 .db 文件")
    parser.add_argument("--force", action="store_true", help="允许覆盖已存在的目标文件")
    args = parser.parse_args()

    if not os.path.isfile(args.source):
        print(f"错误：找不到源数据库 {args.source}", file=sys.stderr)
        return 1
    if os.path.exists(args.dest) and not args.force:
        print(f"错误：{args.dest} 已存在，加 --force 覆盖。", file=sys.stderr)
        return 1
    if os.path.exists(args.dest):
        os.remove(args.dest)

    src = sqlite3.connect(f"file:{args.source}?mode=ro", uri=True)
    dst = sqlite3.connect(args.dest)

    # 1) 建表结构和索引。先不建触发器，否则 time_enter 会被改成当前时间。
    for typ in ("table", "index"):
        for (sql,) in src.execute(
            "select sql from sqlite_master where type=? and sql is not null", (typ,)
        ):
            try:
                dst.execute(sql)
            except sqlite3.DatabaseError as exc:
                print(f"  [跳过] 建 {typ} 失败: {exc}", file=sys.stderr)
    dst.commit()

    # 2) 逐表探测并复制。
    print(f"源数据库：{args.source}")
    print(f"  大小 {os.path.getsize(args.source)/1048576:.1f} MB\n")
    print("  逐表检查：")
    copied, skipped = [], []
    for table in CANDIDATE_TABLES:
        ok, count, err = table_is_readable(src, table)
        if ok:
            try:
                n = copy_table(src, dst, table)
                copied.append((table, n))
                print(f"    {table:18s} 还原 {n:>7,} 行")
            except sqlite3.DatabaseError as exc:
                skipped.append((table, str(exc)))
                print(f"    {table:18s} 复制失败：{exc}")
        else:
            skipped.append((table, err))
            print(f"    {table:18s} 损坏，跳过（{err}）")

    # 3) 再建触发器，保持与原始库一致的行为。
    for (sql,) in src.execute(
        "select sql from sqlite_master where type='trigger' and sql is not null"
    ):
        try:
            dst.execute(sql)
        except sqlite3.DatabaseError:
            pass
    dst.commit()
    dst.close()
    src.close()

    # 4) 校验新库。
    check = sqlite3.connect(f"file:{args.dest}?mode=ro", uri=True)
    integrity = check.execute("pragma integrity_check").fetchone()[0]
    print(f"\n恢复结果：{args.dest}")
    print(f"  大小 {os.path.getsize(args.dest)/1048576:.2f} MB")
    print(f"  integrity_check：{integrity}")
    for table, _ in copied:
        if table in ("Node", "Link", "Word"):
            n = check.execute(f'select count(*) from "{table}"').fetchone()[0]
            print(f"  {table} = {n:,}")
    check.close()

    if skipped:
        print("\n未能恢复：")
        for table, err in skipped:
            print(f"  {table:18s} {err}")
        print(
            "\n提示：缺少 Data/Feature 意味着恢复后的库没有节点图像和 3D 点云，\n"
            "      但位姿图、回环和语义地标仍然可以查看。"
        )
    print(
        "\n查看方式：\n"
        f"  source /opt/ros/humble/setup.bash\n"
        f"  rtabmap-databaseViewer {args.dest}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
