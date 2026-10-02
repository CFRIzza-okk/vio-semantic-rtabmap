import time, sys, numpy as np
from ultralytics import YOLO

def bench(path, task="detect", n=30):
    try:
        m = YOLO(path, task=task)
    except Exception as e:
        print(f"[SKIP] {path}: {type(e).__name__}: {str(e)[:120]}")
        return
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    for _ in range(5):
        m.predict(img, verbose=False, device=0)
    t = time.time()
    for _ in range(n):
        m.predict(img, verbose=False, device=0)
    dt = (time.time() - t) / n
    print(f"[OK] {path.split('/')[-1]:24s} {dt*1000:7.1f} ms/frame  {1/dt:6.1f} FPS")

for p in ["/home/jetson/ultralytics/ultralytics/yolo11n.engine",
          "/home/jetson/ultralytics/ultralytics/yolo26n.engine",
          "/home/jetson/ultralytics/ultralytics/yolo11n.pt"]:
    bench(p)
