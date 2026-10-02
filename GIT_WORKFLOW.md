# Git 使用说明

> 面向本项目的日常用法。仓库位置：`/home/jetson/VIO`，主分支 `main`。
> 你不需要学会 Git 的全部功能，**下面 90% 的场景用不到 6 条命令**。

---

## 一、为什么这个项目需要它

你做的事情有一个特点：**同一份代码，跑不同的参数、在不同的时间，产出不同的数据。**

于是答辩时必然被问到、而你必须答得上来的问题是：

> "第三章那个 ATE 结果，是用哪一版代码、哪一份配置跑出来的？"
> "你说改了语义地标约束之后变好了——那没改之前是什么样？"

没有版本管理，这两个问题只能靠回忆。有了 Git，`git log` 就能直接回答。

这就是本项目的用法重点：**不是"备份代码"，而是"把代码版本和实验数据绑起来"。**

---

## 二、日常三步

改完代码，想存一个档：

```bash
cd /home/jetson/VIO
git status                      # ① 先看改了什么
git add -A                      # ② 全部加入暂存区
git commit -m "说明这次改了什么"  # ③ 存盘
```

就这三条。`git status` 显示 `??` 是"新文件还没管"，`M` 是"改过了"，`A` 是"已准备提交"。

**提交信息写清楚"为什么改"，不是"改了什么"**（改了什么 diff 里能看到）：

```text
差：修改 mapper_node.py
好：地标协方差改为按语义质量自适应，修复透明目标被当成强约束的问题
```

---

## 三、实验前打标签（最重要的一条）

跑实验之前，先给当前代码存一个**带名字的档**：

```bash
git add -A
git commit -m "准备跑 quality 模式对照实验"
git tag exp-quality-20261003        # 给这个版本起个名字
```

跑完实验，数据目录里记一笔，或者直接在文档里写：

```text
数据 data/material_exp/ 由 exp-quality-20261003 版本产出
```

以后想找回当时的代码，一条命令：

```bash
git checkout exp-quality-20261003   # 回到那个版本
git checkout main                   # 看完回主线
```

**这一步是整个流程里最有价值的**。它把"论文里的数字"和"产生数字的代码"锁在一起了。建议每跑一次正式实验就打一个 tag。

---

## 四、改坏了怎么退

分三种情况，按严重程度递增：

```bash
# 情况 1：刚改了几个文件，还没 commit，想全丢掉
git restore .                       # 危险：未提交的修改会永久消失

# 情况 2：只丢某一个文件
git restore semantic_ws/src/semantic_rtabmap/semantic_rtabmap/mapper_node.py

# 情况 3：已经 commit 了，想退回到上一次提交
git log --oneline                   # 先找到要回到哪个提交
git restore --source=<提交号> <文件>  # 只退某个文件
```

**情况 1 之前先想一下**：`git restore .` 不可撤销。拿不准就先把改动提交掉（哪怕提交信息写得潦草），至少能退回来。

看历史：

```bash
git log --oneline                   # 一行一条，简洁
git log --oneline --graph --all     # 带分支图
git show <提交号>                    # 看那次提交具体改了什么
git diff <提交A> <提交B>             # 比较两个版本
```

---

## 五、强烈建议：加一个远程备份

**现在这个仓库只在你 Jetson 本地硬盘上，没有任何备份。** 硬盘坏了、系统重装了，全部历史一起没。这对毕业设计来说风险太高。

两条路，选一条：

### 方案 A：Gitee（国内快，可私有仓库）

```bash
# 1. 在 gitee.com 建一个私有仓库，拿到地址
# 2. 关联并推送
cd /home/jetson/VIO
git remote add origin git@gitee.com:你的用户名/vio.git
git push -u origin main
git push origin --tags              # 标签也要推，否则实验版本没备份
```

### 方案 B：就在本机另做一个裸仓库（不联网，最简单）

```bash
# 在另一块盘或另一个目录做镜像
git clone --bare /home/jetson/VIO /media/你的U盘/vio-backup.git
# 以后同步
cd /home/jetson/VIO && git push /media/你的U盘/vio-backup.git main --tags
```

**注意：`data/` 不在版本管理里**（4.3G 太大了），所以**实验数据要单独备份**。代码有 Git 保底，数据没有。

---

## 六、这个仓库的边界

`.gitignore` 里排除了这些东西，`git add -A` **不会**把它们加进来（已逐个验证过）：

| 排除项 | 为什么 |
|---|---|
| `data/` | 4.3G 实验数据、rosbag。太大，且可重新采集 |
| `build/ install/ log/` | colcon 构建产物，`colcon build` 能重建 |
| `rtabmap_cuda_ws/src/rtabmap{, _ros}` | 上游第三方仓库，自带 `.git`，不属于你的项目 |
| `cleanup_backup_*` | 一次性清理归档 |

**如果你的实验数据需要版本化**，不要直接 `git add data/`。大文件进 Git 会永久留在历史里，仓库再也瘦不下来。正确做法是记录"数据是怎么产生的"（用第三节的 tag），而不是把数据本身塞进去。

---

## 七、命令速查

```bash
# 看状态
git status                    # 现在改了什么
git log --oneline             # 提交历史
git diff                      # 未暂存的改动内容
git diff --staged             # 已暂存的改动内容

# 存盘
git add -A                    # 全部加入暂存
git commit -m "说明"           # 提交
git tag 名字                   # 给当前版本起名（实验前用）

# 反悔
git restore <文件>             # 丢弃某文件的未提交修改
git restore --staged <文件>    # 撤销暂存（改动还在）
git restore --source=<提交> <文件>  # 把某文件退回到指定版本

# 回到过去
git checkout <tag或提交号>     # 切到某个版本（看代码用）
git checkout main             # 回主线

# 远程
git remote -v                 # 看有没有远程
git push origin main --tags   # 推送提交和标签
```

---

## 八、当前的提交身份

```text
user.name  = jetson
user.email = jetson@localhost
```

这是**本机占位符**，不是你的真实身份。纯本地用没问题；要推到 Gitee/GitHub，或者想让提交记录显示你的名字，改成真实的：

```bash
cd /home/jetson/VIO
git config user.name  "你的姓名"
git config user.email "你的邮箱"
git commit --amend --reset-author --no-edit   # 改写首次提交的作者信息
```

顺便把全局身份也配上，否则以后新建仓库会再次报"请告诉我你是谁"：

```bash
git config --global user.name  "你的姓名"
git config --global user.email "你的邮箱"
```
