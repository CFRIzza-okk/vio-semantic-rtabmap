#!/usr/bin/env bash
# 把"人敲的预算"规范成 `ros2 param set` 能接受的形式。
#
# 为什么需要这个函数（2026-10-17 实测）:
#
#   bash tools/r680_explore_300.sh --yes --budget 300
#     -> Setting parameter failed: Wrong parameter type,
#        expected 'Type.DOUBLE' got 'Type.INTEGER'
#     -> 脚本继续往下跑,车也真的动了,但预算是**默认的 120.0**
#
# `/r680_explorer` 的 max_exploration_time 声明为 DOUBLE,`ros2 param set`
# 传整数字面量会被拒。失败信息只有一行,夹在几十行启动输出里滚过去,而脚本
# 当时并不检查回读值 —— 于是"我要跑 600 秒"变成了"跑了 120 秒就停",而且
# 没人知道。`r680_explore_300.sh --help` 推荐的正是 `--budget 600` 这种坏写法。
#
# 所以:预算一律先过这个函数(300 -> 300.0),改完再**回读校验**,不一致就
# 报错退出,而不是让操作者以为自己设了 600。
#
# 走 case 而不是 python3,是为了不在这条冷启动路径上多起一个解释器。
r680_float_arg() {
    case "$1" in
        *.*|*[eE]*) printf '%s' "$1" ;;   # 已经是浮点形态(300.0 / 6e2)
        *)          printf '%s.0' "$1" ;; # 300 -> 300.0
    esac
}
