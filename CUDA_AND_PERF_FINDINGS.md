# CUDA 加速调研 + 干净基线测量结论

> 本文记录一轮完整的"查证型"工作：CUDA 版 RTAB-Map 是否真能加速 VO、以及
> 排除编译负载后 VO 的真实性能。**结论是负面的**——CUDA 那条路技术可行但
> 治不了真正的病。所有数字都标注了来源和可信度。

---

## 0. 一句话结论

**CUDA 特征提取编译成功、运行可用，但它不是瓶颈。** 干净环境下实测：
相机稳定产出 30 Hz，VO 只输出 **9.5 Hz**，而 VO 进程仅占 **29% CPU**。
一个不饱和的进程，加 GPU 不会变快——它不是在"算不动"，而是在"等"。

---

## 1. CUDA 版 RTAB-Map：编译成功，且我独立复验过

### 1.1 产物

```
~/VIO/rtabmap_cuda_ws/build/bin/librtabmap_core.so.0.23.7
17,063,656 字节    编译耗时 2353 s（-j1，129 个目标文件）
```

### 1.2 决定性证据：CUDA 分支真的被编进去了

`corelib/src/Features2d.cpp` 里 GPU 分支的守卫是编译期宏 + 运行期探测**双重**：

| 参数 | 编译期宏 | 源码行 |
|---|---|---|
| `ORB/Gpu` | `HAVE_OPENCV_CUDAFEATURES2D` | L1581 |
| `FAST/Gpu` | `HAVE_OPENCV_CUDAFEATURES2D` | L1828 |
| `GFTT/Gpu` | `HAVE_OPENCV_CUDAIMGPROC` | L2117 |

**这两个宏不是 RTAB-Map 生成的**，来自 OpenCV 自己的 `opencv2/opencv_modules.hpp`。
这正是 apt 版 GPU 选项为灰的原因。实测对比（我亲自执行）：

```
/usr/local/include/opencv4/opencv2/opencv_modules.hpp
    → 11 个 HAVE_OPENCV_CUDA*，含 CUDAFEATURES2D / CUDAIMGPROC / CUDAOPTFLOW
/usr/include/opencv4/opencv2/opencv_modules.hpp
    → 0 个
```

新编出的目标文件里能查到真实调用：

```
$ nm -uC Features2d.cpp.o | grep cv::cuda | wc -l
10
  U cv::cuda::createGoodFeaturesToTrackDetector(...)   ← GFTT/Gpu
  U cv::cuda::ORB::create(...)                          ← ORB/Gpu
```

> 注：这些符号在 `librtabmap_core.so` 中**应当**是 `U`（未定义），因为实现位于
> `libopencv_cudafeatures2d.so.410`。`T` 出现在调用方才是异常。

`ldd` 复核：**4.5d 引用 0 条**，21 条解析到 `/usr/local/lib/*.410`；
`readelf -d` 的 DT_NEEDED 含 `libopencv_cudafeatures2d.so.410`、`cudaimgproc`、
`cudaoptflow`，外加 CUDA 12.6 全套 NPP / cublas / cufft。

### 1.3 ⚠️ 必须记住的地雷：`FAST/Gpu` 会崩节点

`corelib/src/Features2d.cpp` 中 **`FAST` 类的 GPU 路径上游从未实现**：

| 行 | 所属函数 | 代码 |
|---|---|---|
| **1852** | `FAST::parseParameters()` | `UFATAL("not implemented");` |
| **1970** | `FAST::generateKeypointsImpl()` | `UFATAL("not implemented");` |
| 2112 | `GFTT::parseParameters()` | `UWARN(...)`，且仅 OpenCV<3，无害 |
| 2133 | GFTT 实现 | `cv::cuda::createGoodFeaturesToTrackDetector(...)` ✅ |

`UFATAL` 抛 `UException`（继承 `std::runtime_error`）。而守卫只在
`getCudaEnabledDeviceCount()==0` 时降级——本机 4.10 返回 **1**，守卫不触发
→ **必然走到 UFATAL → 节点崩**。`FAST_BRIEF`/`FAST_FREAK` 继承自 `FAST`
且不重写该方法，同样会崩。

**而 OpenCV 层面 FAST GPU 是好的**（独立实测：2006 个关键点）。
纯粹是 rtabmap 没接线。所以：

| 参数 | 状态 | 用法 |
|---|---|---|
| `GFTT/Gpu` | ✅ 可用 | 配 `Vis/FeatureType=8`（GFTT/ORB，本就是默认） |
| `ORB/Gpu` | ✅ 可用 | 配 `Vis/FeatureType=2` |
| `FAST/Gpu` | ❌ **会崩，永远设 false** | — |

`WITH_CUDASIFT` 与这三个参数**无关**（只影响 `SIFT/Gpu`，需另外装 CudaSift），
保持 `OFF` 正确。

### 1.4 硬件侧完全正常（独立于 rtabmap 验证）

```
CUDA Driver 12.60, NumDevs = 1
GFTT/Gpu (cudaimgproc)     -> corners=500
FAST/Gpu (cudafeatures2d)  -> keypoints=2006
ORB/Gpu  (cudafeatures2d)  -> keypoints=500, descriptors=500x32
```

---

## 2. 用它的代价：与 ROS 侧的 OpenCV ABI 冲突

`rtabmap_ros` 必需 `cv_bridge`，而 `libcv_bridge.so` 链的是 **`libopencv_core.so.4.5d`**
（无 CUDA）。与新的 4.10 同进程共存：

- 两套 core 库 **2180/2200 个导出符号同名**，`cv` 命名空间**未做 ABI 版本化**
- 实测符号抢占：**先加载者胜**（先 4.5 则 `cv::Mat::Mat()` 解析到 4.5）
- `cv::Mat` 布局兼容（都是 96 字节，成员偏移一致），`sizeof(cv::KeyPoint)=28` 一致

**后果是"静默失效"而非崩溃**——这点反直觉但重要：

| 符号 | 4.5.4 | 4.10 |
|---|---|---|
| `cv::cuda::getCudaEnabledDeviceCount()` | 有（实测返回 **0**） | 有（实测返回 **1**） |
| `GpuMat::upload/download` | 有 | 有 |
| `cuda::ORB::create` | **无** | 有 |
| `createGoodFeaturesToTrackDetector` | **无** | 有 |

若 4.5d 抢到符号 → 守卫拿到 0 → rtabmap 打印
`UWARN("GPU version of ... not available! Using CPU version instead...")`
→ **自动降级回 CPU**。不崩，但你要的加速**静默消失**，且生效与否取决于
链接顺序抽签 → **结论不可复现**。

> 另有一个既存环境不一致：`libopencv-dev` 提供的是 **4.8.0 头文件**，
> 而运行库是 **4.5.4**。本地重编任何包默认会拿到 4.8 头 + 4.5 库的错配，
> 必须显式 `-DOpenCV_DIR=/usr/local/lib/cmake/opencv4`。
>
> ROS 侧链 4.5d 的库共 **31 个**，含 `librealsense2_camera.so`、`libdepth_image_proc.so`、
> `libimage_proc.so`、`libcv_bridge.so`。全量重编到 4.10 按 `-j1` 估计 **6~10 小时**。

**三条路**（按推荐度）：

1. **绕开 ROS**：独立 C++ 程序直接经 librealsense2 取帧、调 corelib 的
   `rtabmap::Odometry` 走 CUDA，完全不碰 cv_bridge。唯一彻底绕开符号冲突的方案。
2. **全一致 workspace**：cv_bridge / image_* / realsense2_camera / rtabmap_ros
   全部源码重编到 4.10。最干净但 6~10 小时且反复触及内存红线。
3. **直接编 rtabmap_ros 到 4.10、不重编 cv_bridge**：最便宜，不崩但结论不可复现，
   只能当一次性可行性验证。

---

## 3. 干净基线测量：VO 不是算力受限（这是本轮最重要的发现）

### 3.1 测量条件

编译已结束（`cc1plus` 残留 0，可用内存从 ~1 GB 回到 3359 MB），
运行 `--slam-only --no-rviz --fresh-db` 默认配置。
**这是全程第一次在无编译负载下测量**——此前所有数字都不可信。

### 3.2 确定数字

| 指标 | 值 | 来源 |
|---|---|---|
| 相机产出 | **30 Hz** | 此前用 header.stamp 与 C++ 录制双重确认 |
| **`/odom` 产出** | **9.4–9.6 Hz** | `ros2 topic hz`，min 53 ms / max 209 ms |
| VO `update time` | **48–96 ms**（中位 ~65 ms） | rtabmap 自身日志 |
| VO `delay` | 0.10–0.20 s | 同上 |

> `/odom` 消息很小，`ros2 topic hz` 的 Python 探针低估问题**不适用**（那个问题
> 只影响 921 KB 的整帧图像）。此处数字可信。
>
> 我自己的 `tools/measure_topic_rate.py` 对 `/odom` 返回 0 条，是脚本 QoS 不匹配
> （发布者 RELIABLE/VOLATILE），**不是话题问题**——该脚本对图像话题仍有效。

### 3.3 关键反证：VO 只占 29% CPU

```
chrome         117.6%   ← 会话界面，不能关
node-Ma…        70.6%   ← DSH 自身（node --import tsx/esm apps/cli/src/bin.ts web）
rgbd_odometry   29.4%   ← 我们的 VO，远未跑满一个核
Xorg            17.6%
nxnode/nxcodec  17.6%   ← NoMachine，不能关
realsense2      17.6%
rtabmap         11.8%
```

**推理**：如果 VO 是计算受限，它应接近 100%（单线程满核）。实测 29%，
说明 65 ms 的 `update time` 里大部分是**等待**（同步/互斥/队列），不是运算。
→ **给特征提取加 GPU 不会提升这个吞吐。**

### 3.4 桌面栈的 CPU 账（必须写进论文的环境章节）

6 核机器上，**我这一侧的软件栈就吃掉 ~190–340%**：
`chrome`(会话界面) + `node`(agent 运行时) + `Xorg` + `nxnode`/`nxcodec`(NoMachine)。

这解释了为什么早期会把"CPU 饱和"误判为瓶颈——**确实饱和，但饱和源是我们
自己的观测工具，不是被测系统**。测量者必须计入自身的开销，否则会把自己的
影子当成被测对象。

### 3.5 副作用警告

运行期间持续出现：

```
The time difference between rgb and depth frames is high (diff=0.033347s)
Dropping image/scan data with stamp ... (delay=0.043)
```

`diff≈0.0333s` ≈ 30 Hz 的一帧，是 D435 彩色与深度流**相差整一帧**的典型特征。
这两条是 **VO 落后的症状而非原因**。但"RGB 与深度差整帧"本身值得单独查——
它可能是真实的同步问题，也可能是 VO 慢的**结果**。此前测 `enable_sync:=true`
无改善，但那是在编译负载下测的，**需要重测**。

---

## 4. 下一步的判据（待决策）

按价值排序，**都要求在无编译负载下进行**：

1. **先弄清 VO 在等什么**。它只占 29% CPU，却要 65 ms/帧。用 `strace -c` /
   `perf` 看时间花在哪：等锁、等同步、还是等 DDS。这决定了后续所有优化方向。
2. **重测 RGB/深度同步**（`enable_sync`、`approx_sync_max_interval`）。
   如果 33 ms 的整帧偏移是真的，修它可能比任何 GPU 优化都有效。
3. **只有当 1、2 都排除后**，才考虑 CUDA。届时的正确形态是 §2 的**路线 1
   （绕开 ROS）**，而不是让 4.10 和 4.5 在同一进程里抽签。
4. 论文中 CUDA 部分应写成**已验证的负结果 + 可复现的方法论**：
   证明编译成功、证明运行可用、证明它不解决瓶颈、并解释为什么。
   这比一个说不清收益的"加速比"更站得住。
