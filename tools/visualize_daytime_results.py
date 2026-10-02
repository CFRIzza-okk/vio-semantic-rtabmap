#!/usr/bin/env python3
"""Build publication-oriented visualizations and a concise analysis from existing S01/S05 CSVs."""
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path.home() / "VIO" / "data"
OUT = ROOT / "daytime_analysis"
OUT.mkdir(parents=True, exist_ok=True)
FONT = Path.home() / "VIO/tools/fonts/NotoSansCJKsc-Regular.otf"
if FONT.exists():
    font_manager.fontManager.addfont(str(FONT))
    family = font_manager.FontProperties(fname=str(FONT)).get_name()
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = [family, "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 140
plt.rcParams["savefig.dpi"] = 180
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = .22

mat = pd.read_csv(ROOT / "material_exp/bottle_150/features.csv")
abl = pd.read_csv(ROOT / "ablation_s01_fixed/summary.csv")
off = pd.read_csv(ROOT / "bottle_offset/bottle_offset_per_frame.csv")
board = pd.read_csv(ROOT / "board_align/board_alignment_summary.csv")
trans = pd.read_csv(ROOT / "transparency_probe/transparency_features.csv")

# Aggregate frame-level material features; use frame means, not averaging ratios of pooled pixels.
metrics = {
    "整框有效深度占比": (mat.valid_ratio.mean()*100, mat.valid_ratio.std(ddof=1)*100),
    "中心 ROI 有效占比": (mat.roi_valid_ratio.mean()*100, mat.roi_valid_ratio.std(ddof=1)*100),
    "中心/整框比 r": (mat.ratio_r.mean(), mat.ratio_r.std(ddof=1)),
    "前景占比": (mat.fg_frac.mean()*100, mat.fg_frac.std(ddof=1)*100),
}
# Figure 1: material feature comparison + sample points
fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.4))
labels = ["透明塑料瓶\nS01 (n=33)", "不透明标定板\nS05 (单次汇总)"]
colors = ["#2c7fb8", "#41ab5d"]
vals = [mat.valid_ratio.mean()*100, 87.41058655221745]
errs = [mat.valid_ratio.std(ddof=1)*100, 0]
ax[0].bar(labels, vals, yerr=errs, capsize=5, color=colors, alpha=.88)
ax[0].scatter(np.repeat(0, len(mat))*0.08 + np.random.default_rng(8).normal(0,.035,len(mat)), mat.valid_ratio*100,
              color="#084081", s=12, alpha=.5, label="瓶子逐帧")
ax[0].set_ylabel("整框有效深度 (%)"); ax[0].set_ylim(0,105); ax[0].set_title("整框深度完整性")
for i,v in enumerate(vals): ax[0].text(i, v+3, f"{v:.1f}%", ha="center", fontweight="bold")
rv=[mat.ratio_r.mean(),1.1132358836854432]
re=[mat.ratio_r.std(ddof=1),0]
ax[1].bar(labels,rv,yerr=re,capsize=5,color=colors,alpha=.88)
ax[1].scatter(np.repeat(0,len(mat))*0.08+np.random.default_rng(12).normal(0,.035,len(mat)),mat.ratio_r,
              color="#084081",s=12,alpha=.5,label="瓶子逐帧")
ax[1].axhline(.7,color="#d7301f",ls="--",lw=1.2,label="透明阈值 r=0.7")
ax[1].axhline(1.,color="#636363",ls=":",lw=1.2,label="不透明阈值 r=1.0")
ax[1].set_ylabel("r = 中心 ROI 有效占比 / 整框有效占比"); ax[1].set_ylim(0,1.35); ax[1].set_title("空间分布判据 r")
for i,v in enumerate(rv): ax[1].text(i,v+.035,f"{v:.2f}",ha="center",fontweight="bold")
ax[1].legend(fontsize=8,loc="upper left")
fig.suptitle("S01 / S05 材质特征对照（S05 仅有单次汇总，不代表重复样本）",fontweight="bold")
fig.tight_layout()
fig.savefig(OUT/"fig1_material_features.png",bbox_inches="tight"); plt.close(fig)

# Figure 2: ablation MAE (log scale) and variability
fig, ax=plt.subplots(figsize=(9.2,4.8))
short={"V0_full_median":"V0 全框中位数","V1_full_mean":"V1 全框均值","V2_roi_median":"V2 中心 ROI 中位数","V3_roi_p20":"V3 中心 ROI P20","V4_roi_p20_cluster":"V4 中心 ROI P20+聚类"}
names=[short.get(x,x) for x in abl.method]
mae=abl.mae_z_m.to_numpy()*1000; std=abl.std_z_m.to_numpy()*1000
bars=ax.bar(names,mae,color=["#3182bd","#de2d26","#9ecae1","#6baed6","#31a354"])
ax.errorbar(np.arange(len(abl)),mae,yerr=std,fmt="none",ecolor="#222",capsize=4,lw=1)
ax.set_yscale("log"); ax.set_ylabel("绝对 z 误差 MAE (mm，对数轴)"); ax.set_title("S01 透明瓶：反投影消融（每种方法 n=33）")
ax.tick_params(axis="x",rotation=15)
for b,v in zip(bars,mae): ax.text(b.get_x()+b.get_width()/2,v*1.18,f"{v:.2f}",ha="center",fontsize=9)
ax.text(.99,.96,"误差棒：±1 STD",transform=ax.transAxes,ha="right",va="top",fontsize=9)
fig.tight_layout(); fig.savefig(OUT/"fig2_ablation_mae.png",bbox_inches="tight"); plt.close(fig)

# Figure 3: spatial offset measurements and corrected residuals
fig,ax=plt.subplots(1,2,figsize=(11.5,4.3))
x=np.arange(len(off))
ax[0].plot(x,off.dx_px,"o-",ms=3,lw=1,color="#2c7fb8")
ax[0].axhline(off.dx_px.mean(),color="#d7301f",ls="--",label=f"均值 {off.dx_px.mean():.2f}px")
ax[0].set_xlabel("帧序号");ax[0].set_ylabel("深度质心 − 彩色质心 (px)");ax[0].set_title("透明瓶横向偏移（逐帧）");ax[0].legend()
ax[1].boxplot([off.dx_pos_mm,off.dz_pos_mm],tick_labels=["横向修正量 Δx","深度修正量 Δz"],showmeans=True)
ax[1].axhline(0,color="#636363",lw=1);ax[1].set_ylabel("修正前后位置差 (mm)");ax[1].set_title("应用彩色框中心后的位移量")
fig.suptitle("S01 透明瓶：横向偏移稳定，修正主要作用于横向",fontweight="bold")
fig.tight_layout();fig.savefig(OUT/"fig3_lateral_offset.png",bbox_inches="tight");plt.close(fig)

# Figure 4: S05 alignment geometry summary, explicitly one board run
fig,ax=plt.subplots(figsize=(7.3,4.4))
row=board.iloc[0]
vals=[row.width_diff_px,row.center_diff_px]
ax.bar(["深度框宽 − ArUco 框宽","深度框中心 − ArUco 框中心"],vals,color=["#756bb1","#31a354"],width=.55)
ax.axhline(0,color="black",lw=.8);ax.set_ylabel("差值 (px)");ax.set_title("S05 标定板对齐检查（16 行；4 个标记）")
for i,v in enumerate(vals):ax.text(i,v+(0.5 if v>=0 else -1.2),f"{v:+.2f}px",ha="center",fontweight="bold")
ax.text(.98,.96,f"中心差 σ={row.center_diff_std_px:.2f}px\n等效横向={row.center_diff_mm:.2f}mm",transform=ax.transAxes,ha="right",va="top",bbox=dict(boxstyle="round",fc="white",alpha=.85))
fig.tight_layout();fig.savefig(OUT/"fig4_board_alignment.png",bbox_inches="tight");plt.close(fig)

# Stats and interpretations for report

def mean_sd(x, scale=1):
    a=np.asarray(x,dtype=float)*scale
    return float(np.mean(a)),float(np.std(a,ddof=1))

def quant(x,scale=1):
    a=np.asarray(x,dtype=float)*scale
    return np.percentile(a,[5,50,95]).tolist()

vmean,vstd=mean_sd(mat.valid_ratio,100)
rmean,rstd=mean_sd(mat.ratio_r)
roi_mean,roi_std=mean_sd(mat.roi_valid_ratio,100)
fg_mean,fg_std=mean_sd(mat.fg_frac,100)
offmean,offstd=mean_sd(off.dx_px)
dx_corr=mean_sd(off.dx_pos_mm);dz_corr=mean_sd(off.dz_pos_mm)
methods=[]
for _,r in abl.iterrows():
    methods.append(f"| {short.get(r.method,r.method)} | {int(r.detections)} | {r.mae_z_m*1000:.2f} | {r.std_z_m*1000:.2f} | {r.max_error_z_m*1000:.2f} |")
report=f'''# 白天测试数据可视化与分析（S01 / S05）

> 分析范围：当前磁盘上已有的透明瓶 S01、标定板 S05 与对应逐帧/汇总 CSV。**没有新增实验数据**；M2（金属/镜面）尚未采集。
> 图表目录：`~/VIO/data/daytime_analysis/`。

## 结论摘要

1. **S01 透明瓶的中心深度缺失特征稳定。** n={len(mat)} 帧，整框有效深度 **{vmean:.2f}% ± {vstd:.2f}%**，中心 ROI **{roi_mean:.2f}% ± {roi_std:.2f}%**，r **{rmean:.3f} ± {rstd:.3f}**；逐帧 r 的 P5/P50/P95 为 {quant(mat.ratio_r)[0]:.3f}/{quant(mat.ratio_r)[1]:.3f}/{quant(mat.ratio_r)[2]:.3f}。每帧均落在 r<0.7，支持此样本上的透明分支判定。
2. **稳健深度统计比全框均值明显可靠。** V0 全框中位数 MAE={abl.loc[abl.method=='V0_full_median','mae_z_m'].iloc[0]*1000:.2f} mm；V4 中心 ROI P20+聚类 MAE={abl.loc[abl.method=='V4_roi_p20_cluster','mae_z_m'].iloc[0]*1000:.2f} mm。V1 全框均值 MAE={abl.loc[abl.method=='V1_full_mean','mae_z_m'].iloc[0]*1000:.1f} mm，主要受背景深度污染。
3. **S01 横向偏移稳定、方向一致。** 深度质心相对彩色分割质心平均偏 {offmean:.2f} ± {offstd:.2f} px（n={len(off)}）；彩色框中心修正的横向位移平均 {dx_corr[0]:.2f} ± {dx_corr[1]:.2f} mm。此量与「中心投影后横向误差」是不同评价量，不能直接把位移量当作残余误差。
4. **S05 对齐检查显示主要是边缘宽度差，不是整体中心平移。** 深度框宽度差 {row.width_diff_px:+.2f}px，中心差 {row.center_diff_px:+.2f} ± {row.center_diff_std_px:.2f}px（16 行；4 个 ArUco 标记）。这支持「S01 横向偏移并非简单 RGB-D 全局错位」的解释，但单次、有限行数仍不是强泛化证明。

## 图表

![材质特征](fig1_material_features.png)

![消融结果](fig2_ablation_mae.png)

![横向偏移](fig3_lateral_offset.png)

![标定板对齐检查](fig4_board_alignment.png)

## S01 消融详细数值

| 方法 | 检测数 | MAE-z (mm) | STD-z (mm) | 最大误差 (mm) |
|---|---:|---:|---:|---:|
{chr(10).join(methods)}

## 解读与边界

- V0/V4 的 MAE 是**距离真值误差**；这不能说明所有位置分量都准确。透明瓶仍有明显横向伪影，故材质自适应分流的动机成立。
- V1 的平均误差巨大，符合全框均值混入远背景的机制；它不是一个合理的稳健估计器基线，但保留它能展示背景污染的后果。
- S05 数据是一个标定板 capture 的汇总（16 行统计、4 个标记），S01 是 33 帧；图 1 把 S05 画成无误差线的单次基准，**不能据此声称类别级统计显著性**。
- S01 每帧来自同一静态场景，帧间高度相关；n=33 是时间重复，不等于 33 个独立物体/场景样本。论文中宜称「33 帧重复观测」，不要当独立样本做过度推断。
- S05 的平面拟合 RMS 为 16.0 mm、倾角约 10.9°（见原始汇总）；不应用它替代横向位移统计，也需在解释中指出平面拟合受可见区域/姿态影响。
- 目前只能支持**这两个具体样本、当前距离与采集条件下**的现象。阈值泛化、尺寸混淆、距离变化和材质复杂性仍待 M1–M7 验证；明早再做 M2 金属/镜面采集。

## 可复现的数据源

- `data/material_exp/bottle_150/features.csv`、`per_frame.csv`
- `data/ablation_s01_fixed/summary.csv`
- `data/bottle_offset/bottle_offset_per_frame.csv`
- `data/board_align/board_alignment_summary.csv`
- `data/transparency_probe/transparency_features.csv`

生成命令：

```bash
python3 ~/VIO/tools/visualize_daytime_results.py
```
'''
(OUT/"分析报告.md").write_text(report,encoding="utf-8")

# Export a compact metrics table for reuse in slides/thesis.
summary=pd.DataFrame([
 {"样本":"S01 透明瓶","n":len(mat),"整框有效均值_%":vmean,"整框有效STD_%":vstd,"中心有效均值_%":roi_mean,"中心有效STD_%":roi_std,"r均值":rmean,"r_STD":rstd,"前景占比均值_%":fg_mean,"偏移均值_px":offmean,"偏移STD_px":offstd},
 {"样本":"S05 标定板","n":1,"整框有效均值_%":87.41058655221745,"整框有效STD_%":np.nan,"中心有效均值_%":97.30860156392172,"中心有效STD_%":np.nan,"r均值":1.1132358836854432,"r_STD":np.nan,"前景占比均值_%":96.35402240966889,"偏移均值_px":row.center_diff_px,"偏移STD_px":row.center_diff_std_px},
])
summary.to_csv(OUT/"核心指标汇总.csv",index=False,encoding="utf-8-sig")
print("输出目录:",OUT)
print("S01 n=",len(mat),"valid=%.2f%% ± %.2f%%; ROI=%.2f%% ± %.2f%%; r=%.3f ± %.3f"%(vmean,vstd,roi_mean,roi_std,rmean,rstd))
print("offset=%.2f ± %.2f px; correction displacement x=%.2f ± %.2f mm"%(offmean,offstd,dx_corr[0],dx_corr[1]))
print("S05 width diff=%.2f px; center diff=%.2f ± %.2f px"%(row.width_diff_px,row.center_diff_px,row.center_diff_std_px))
