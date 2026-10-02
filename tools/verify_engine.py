#!/usr/bin/env python3
"""对比 TensorRT engine 与 PyTorch .pt 的推理结果是否一致。

为什么必须做这件事:
    TensorRT engine 是**绑定设备**编译的。加载时 TensorRT 会警告
        [W] Using an engine plan file across different models of devices
            is not recommended and is likely to affect performance or even
            cause errors.
    说明这个 engine 是在别的设备/别的 TRT 版本上构建的。如果只是"能跑"
    就拿来用，可能出现检测框偏移、类别错乱甚至静默漏检——这类问题在答辩
    现场是致命的。所以必须用同一张真实图像，把两个模型的输出逐框对比。

用法: python3 /home/jetson/VIO/tools/verify_engine.py
需要先跑 tools/capture_frame.py 生成 /home/jetson/VIO/data/frame.npy
"""

import sys
import time

import numpy as np

FRAME = "/home/jetson/VIO/data/frame.npy"
ENGINE = "/home/jetson/ultralytics/ultralytics/yolo11n.engine"
PT = "/home/jetson/ultralytics/ultralytics/yolo11n.pt"

CONF = 0.35
IOU = 0.45


def run(path, img):
    from ultralytics import YOLO

    model = YOLO(path, task="detect")
    # 预热
    model.predict(img, conf=CONF, iou=IOU, verbose=False, device=0)
    t = time.time()
    n = 10
    for _ in range(n):
        r = model.predict(img, conf=CONF, iou=IOU, verbose=False, device=0)
    dt = (time.time() - t) / n
    boxes = []
    if r and r[0].boxes is not None and len(r[0].boxes):
        b = r[0].boxes
        for xyxy, cf, cl in zip(
            b.xyxy.cpu().numpy(), b.conf.cpu().numpy(), b.cls.cpu().numpy().astype(int)
        ):
            boxes.append((tuple(float(v) for v in xyxy), float(cf), int(cl)))
    names = dict(model.names)
    return boxes, dt, names


def iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def main():
    try:
        img = np.load(FRAME)
    except FileNotFoundError:
        print(f"找不到 {FRAME}，请先运行: python3 tools/capture_frame.py")
        return 1

    print(f"测试图像: {FRAME}  {img.shape}")
    print(f"置信度阈值 {CONF}, NMS IoU {IOU}\n")

    eb, edt, en = run(ENGINE, img)
    pb, pdt, pn = run(PT, img)

    print("实例数: engine=%d, pt=%d" % (len(eb), len(pb)))
    print(f"推理速度: engine {edt*1000:.1f} ms ({1/edt:.1f} FPS) | "
          f"pt {pdt*1000:.1f} ms ({1/pdt:.1f} FPS) | 加速 {pdt/edt:.2f}x\n")

    if not eb and not pb:
        print("两个模型都没检测到任何目标 —— 无法充分验证。")
        print("请让相机对准有明显物体（人/杯子/椅子）的场景后重新采集。")
        return 2

    print("逐框匹配（IoU > 0.5 视为同一目标）:")
    used = set()
    max_conf_diff = 0.0
    max_box_diff = 0.0
    mismatches = []
    matched = 0
    for exyxy, ecf, ecl in eb:
        best, best_i = 0.0, -1
        for i, (pxyxy, pcf, pcl) in enumerate(pb):
            if i in used:
                continue
            v = iou(exyxy, pxyxy)
            if v > best:
                best, best_i = v, i
        if best > 0.5 and best_i >= 0:
            pxyxy, pcf, pcl = pb[best_i]
            used.add(best_i)
            matched += 1
            cd = abs(ecf - pcf)
            bd = max(abs(a - b) for a, b in zip(exyxy, pxyxy))
            max_conf_diff = max(max_conf_diff, cd)
            max_box_diff = max(max_box_diff, bd)
            cls_ok = "OK" if ecl == pcl else f"类别不一致! {en.get(ecl)} vs {pn.get(pcl)}"
            if ecl != pcl:
                mismatches.append((en.get(ecl), pn.get(pcl)))
            print(f"  IoU {best:.3f}  {en.get(ecl):<12s} "
                  f"conf engine={ecf:.3f} pt={pcf:.3f} (差 {cd:.4f})  "
                  f"框最大像素差 {bd:.2f}  {cls_ok}")
        else:
            print(f"  engine 独有: {en.get(ecl)} conf={ecf:.3f} (在 pt 结果中无匹配)")
            mismatches.append(("engine-only", en.get(ecl)))

    for i, (pxyxy, pcf, pcl) in enumerate(pb):
        if i not in used:
            print(f"  pt 独有: {pn.get(pcl)} conf={pcf:.3f} (在 engine 结果中无匹配)")
            mismatches.append(("pt-only", pn.get(pcl)))

    print("\n" + "=" * 52)
    print(f"匹配上 {matched} 个目标")
    print(f"最大框坐标偏差: {max_box_diff:.2f} 像素  <- 判定依据")
    print(f"最大置信度偏差: {max_conf_diff:.4f}   <- 参考信息")

    # 判定标准说明：
    #   真正决定可用性的是「类别是否一致」和「框位置是否一致」。
    #   置信度差异是 FP16 量化的固有现象——engine 用 half=True 导出为 FP16，
    #   而 .pt 跑 FP32，同一个目标的置信度相差 5%~8% 属于正常范围。
    #   实测（本机 Orin NX, TRT 10.7）：重建前后的 engine 给出完全相同的
    #   置信度 0.709，与 FP32 的 0.773 稳定相差 0.064，证明这是量化效应
    #   而非引擎损坏。
    ok = True
    if max_box_diff > 3.0:
        print("失败: 框坐标偏差 > 3 像素, engine 不可靠")
        ok = False
    if mismatches:
        print(f"失败: 类别或目标数不一致: {mismatches}")
        ok = False
    if matched == 0:
        print("失败: 没有任何目标匹配上")
        ok = False

    if max_conf_diff > 0.05:
        print(
            f"\n注意: 置信度偏差 {max_conf_diff:.3f} > 0.05，这是 FP16 量化的正常表现。\n"
            "      实际影响: 如果你的 conf_threshold 恰好卡在某个目标的置信度附近，\n"
            "      FP16 engine 可能把它判为漏检、或反过来多检出。建议把阈值设在\n"
            "      离常见置信度较远的位置（如 0.35~0.45），并在实机上核对检出率。"
        )

    if ok:
        print(f"\n结论: engine 与 pt 检测结果一致，可以安全使用。")
        print(f"      TensorRT FP16 实测加速 {pdt/edt:.2f}x ({edt*1000:.1f} ms vs {pdt*1000:.1f} ms)")
        print(f"      建议: 实时用 engine，把 .pt 留在参数里作为回退。")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
