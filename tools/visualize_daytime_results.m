%% 白天测试数据可视化：S01 透明瓶 / S05 标定板
% 用途：基于现有 CSV 生成论文分析初稿图表。
% 输出目录：~/VIO/data/daytime_analysis_matlab/
%
% 重要约束：
%   - S01 的 33 帧来自同一静态序列，帧间相关；
%   - S05 是单次采集汇总；
%   - 因此这些图是初步现象与方法动机证据，不应解释为跨物体统计显著性。
%   - 图 3 左栏 dx_px 与右栏 dx_pos_mm 口径不同；
%     右栏是修正前后的位置位移量，不应误读为修正后的残余误差。
%
% 运行：直接在项目根目录或其子目录下执行 visualize_daytime_results.m。

clear; close all; clc;

%% 1. 路径与样式设置（统一从脚本位置推导，便于任意 CWD 运行）
scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);
dataDir = fullfile(projectDir, 'data');
outDir = fullfile(dataDir, 'daytime_analysis_matlab');
if ~exist(outDir, 'dir'), mkdir(outDir); end

applyReportStyle();   % 统一字体、轴线宽、色彩等

%% 2. 输入数据加载与存在性检查
inputFiles = { ...
    fullfile(dataDir, 'material_exp',    'bottle_150', 'features.csv'), ...
    fullfile(dataDir, 'material_exp',    'bottle_150', 'per_frame.csv'), ...
    fullfile(dataDir, 'ablation_s01_fixed',              'summary.csv'), ...
    fullfile(dataDir, 'bottle_offset',                   'bottle_offset_per_frame.csv'), ...
    fullfile(dataDir, 'board_align',                     'board_alignment_summary.csv'), ...
    fullfile(dataDir, 'transparency_probe',              'transparency_features.csv')};
for k = 1:numel(inputFiles)
    if ~isfile(inputFiles{k})
        error('缺少输入文件：%s', inputFiles{k});
    end
end

% 保留 CSV 原列名，减少 MATLAB 版本差异对变量名的自动改写
mat    = readtable(inputFiles{1}, 'VariableNamingRule','preserve');
frames = readtable(inputFiles{2}, 'VariableNamingRule','preserve');
abl    = readtable(inputFiles{3}, 'VariableNamingRule','preserve');
off    = readtable(inputFiles{4}, 'VariableNamingRule','preserve');
board  = readtable(inputFiles{5}, 'VariableNamingRule','preserve');
trans  = readtable(inputFiles{6}, 'VariableNamingRule','preserve');

% 从对照表读 S05 单一汇总值，避免在脚本里硬编码数字
s05Valid = trans.valid_ratio(1) * 100;
s05R     = trans.valid_ratio_ratio(1);

rng(8, 'twister');   % 散点抖动可复现

%% 3. 准备四张图共用的数据
s01Valid = mat.valid_ratio    * 100;
s01Roi   = mat.roi_valid_ratio* 100;
s01R     = mat.ratio_r;
s01FG    = mat.fg_frac        * 100;

methodNames = string(abl.method);
shortNames  = cellstr(methodNames);
shortNames = strrep(shortNames, 'V0_full_median',       'V0 全框中位数');
shortNames = strrep(shortNames, 'V1_full_mean',         'V1 全框均值');
shortNames = strrep(shortNames, 'V2_roi_median',        'V2 中心 ROI 中位数');
shortNames = strrep(shortNames, 'V3_roi_p20',           'V3 中心 ROI P20');
shortNames = strrep(shortNames, 'V4_roi_p20_cluster',   'V4 中心 ROI P20+聚类');

maeMm = abl.mae_z_m * 1000;
stdMm = abl.std_z_m * 1000;

boardRow = board(1, :);

%% 4. 渲染并导出各图
renderFig1(s01Valid, s01R, s05Valid, s05R, outDir);
renderFig2(shortNames, maeMm, stdMm, outDir);
renderFig3(off, outDir);
renderFig4(boardRow, outDir);

%% 5. 终端摘要
fprintf('\n已生成图表，输出目录：%s\n', outDir);
fprintf('S01 n=%d：整框有效 %.2f%% ± %.2f%%；中心 ROI %.2f%% ± %.2f%%；r = %.3f ± %.3f\n', ...
    height(mat), mean(s01Valid), std(s01Valid), ...
    mean(s01Roi),   std(s01Roi),   mean(s01R), std(s01R));
fprintf('S01 横向偏移 %.2f ± %.2f px（n=%d）\n', ...
    mean(off.dx_px), std(off.dx_px), height(off));
fprintf('S05 宽度差 %+.2f px；中心差 %+.2f ± %.2f px（单次汇总）\n', ...
    boardRow.width_diff_px, boardRow.center_diff_px, boardRow.center_diff_std_px);
fprintf(['提醒：S01 的 33 帧是同一静态场景的时间重复，S05 是单次汇总；' ...
    '不应将其解释为跨物体统计显著性。\n']);

%% ======================= 局部函数定义 =======================

function applyReportStyle()
% 设定统一的论文风格：白底、深色字体、克制色板、细网格
    set(groot, 'DefaultFigureColor',      'w');
    set(groot, 'DefaultAxesColor',        'w');
    set(groot, 'DefaultAxesXColor',       [0.13 0.13 0.13]);
    set(groot, 'DefaultAxesYColor',       [0.13 0.13 0.13]);
    set(groot, 'DefaultAxesLineWidth',    1.0);
    set(groot, 'DefaultAxesBox',          'off');
    set(groot, 'DefaultAxesTickDir',      'out');
    set(groot, 'DefaultAxesTickLength',   [0.012 0.012]);
    set(groot, 'DefaultAxesGridAlpha',    0.18);
    set(groot, 'DefaultAxesFontName',     'Helvetica');
    set(groot, 'DefaultAxesFontSize',     10);
    set(groot, 'DefaultTextFontName',     'Helvetica');
    set(groot, 'DefaultTextFontSize',     10);
    set(groot, 'DefaultLegendBox',        'off');
    set(groot, 'DefaultLineLineWidth',    1.1);
end

function renderFig1(s01Valid, s01R, s05Valid, s05R, outDir)
% 图 1：S01 / S05 材质特征对比
    COLOR_BLUE  = [0.18 0.51 0.74];
    COLOR_GREEN = [0.27 0.69 0.39];
    COLOR_DOT   = [0.05 0.30 0.55];

    fig = figure('Color','w','Position',[100 100 1100 460]);
    tl  = tiledlayout(fig, 1, 2, 'TileSpacing','compact', 'Padding','compact');

    % ---- 左图：整框有效深度比例
    ax = nexttile;
    means = [mean(s01Valid), s05Valid];
    sds   = [std(s01Valid, 0), 0];
    b = bar(ax, [1 2], means, 0.62, 'FaceColor','flat');
    b.CData = [COLOR_BLUE; COLOR_GREEN];
    hold(ax, 'on');
    errorbar(ax, [1 2], means, sds, 'k.', 'LineWidth',1.0, 'CapSize',7);
    jitter = (rand(size(s01Valid))-0.5)*0.12;
    scatter(ax, 1+jitter, s01Valid, 22, COLOR_DOT, 'filled', ...
        'MarkerFaceAlpha',0.55, 'MarkerEdgeAlpha',0.0);
    text(ax, 1, means(1)+3.5, sprintf('%.1f%%',means(1)), ...
        'HorizontalAlignment','center', 'FontWeight','bold');
    text(ax, 2, means(2)+3.5, sprintf('%.1f%%',means(2)), ...
        'HorizontalAlignment','center', 'FontWeight','bold');
    set(ax,'XTick',1:2,'XTickLabel',{'透明塑料瓶 S01 (n=33)', '不透明标定板 S05 (单次汇总)'});
    ylim(ax, [0 105]);
    ylabel(ax, '整框有效深度 (%)');
    title(ax, '(a) 整框深度完整性');
    grid(ax, 'on');

    % ---- 右图：r = 中心 ROI / 整框
    ax = nexttile;
    meansR = [mean(s01R), s05R];
    sdsR   = [std(s01R, 0), 0];
    b = bar(ax, [1 2], meansR, 0.62, 'FaceColor','flat');
    b.CData = [COLOR_BLUE; COLOR_GREEN];
    hold(ax, 'on');
    errorbar(ax, [1 2], meansR, sdsR, 'k.', 'LineWidth',1.0, 'CapSize',7);
    jitter = (rand(size(s01R))-0.5)*0.12;
    scatter(ax, 1+jitter, s01R, 22, COLOR_DOT, 'filled', ...
        'MarkerFaceAlpha',0.55, 'MarkerEdgeAlpha',0.0);
    yline(ax, 0.7, '--', '透明判据 r = 0.7', ...
        'Color',[0.82 0.16 0.13],'LineWidth',1.2, 'LabelHorizontalAlignment','left');
    yline(ax, 1.0, ':',  '不透明判据 r = 1.0', ...
        'Color',[0.40 0.40 0.40],'LineWidth',1.2, 'LabelHorizontalAlignment','left');
    text(ax, 1, meansR(1)+0.06, sprintf('%.2f',meansR(1)), ...
        'HorizontalAlignment','center', 'FontWeight','bold');
    text(ax, 2, meansR(2)+0.06, sprintf('%.2f',meansR(2)), ...
        'HorizontalAlignment','center', 'FontWeight','bold');
    set(ax,'XTick',1:2,'XTickLabel',{'透明塑料瓶 S01 (n=33)', '不透明标定板 S05 (单次汇总)'});
    ylim(ax, [0 1.35]);
    ylabel(ax, 'r = 中心 ROI 有效占比 / 整框有效占比');
    title(ax, '(b) 空间分布判据 r');
    grid(ax, 'on');

    sgtitle(tl, '图 1  S01 / S05 材质特征对照（S05 为单次基准，非重复样本统计）', 'FontWeight','bold');
    exportgraphics(fig, fullfile(outDir,'fig1_material_features.png'), 'Resolution',240);
end

function renderFig2(shortNames, maeMm, stdMm, outDir)
% 图 2：S01 深度反投影消融（对数刻度）
    palette = [ ...
        0.19 0.51 0.74;  ...  V0 蓝
        0.87 0.18 0.16;  ...  V1 红
        0.62 0.78 0.89;  ...  V2 浅蓝
        0.42 0.67 0.83;  ...  V3 中蓝
        0.19 0.64 0.33];     V4 绿

    fig = figure('Color','w','Position',[100 100 1000 520]);
    ax  = axes(fig); hold(ax,'on');

    b = bar(ax, maeMm, 0.66, 'FaceColor','flat');
    b.CData = palette;
    errorbar(ax, 1:numel(maeMm), maeMm, stdMm, ...
        'k.', 'LineWidth',1.0, 'CapSize',7);
    set(ax, 'YScale','log', ...
            'XTick',1:numel(maeMm), 'XTickLabel',shortNames);
    xtickangle(ax, 12);
    ylabel(ax, 'z 误差 MAE (mm，对数轴)');
    title(ax, '图 2  S01 透明瓶：深度统计方法消融（每种方法 n=33）', 'FontWeight','bold');
    grid(ax, 'on');

    % 在柱顶标数值（对数轴下用乘性偏移）
    for i = 1:numel(maeMm)
        text(ax, i, maeMm(i)*1.28, sprintf('%.2f', maeMm(i)), ...
            'HorizontalAlignment','center', 'FontSize',9);
    end
    text(ax, 0.99, 0.96, '误差棒：±1 STD', ...
        'Units','normalized', 'HorizontalAlignment','right', 'VerticalAlignment','top');
    exportgraphics(fig, fullfile(outDir,'fig2_ablation_mae.png'), 'Resolution',240);
end

function renderFig3(off, outDir)
% 图 3：逐帧横向偏移 + 修正前后位移量
    fig = figure('Color','w','Position',[100 100 1120 460]);
    tl  = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

    % ---- 左：逐帧 dx_px
    ax = nexttile;
    plot(ax, 1:height(off), off.dx_px, 'o-', ...
        'Color',[0.18 0.51 0.74], 'MarkerSize',4, 'LineWidth',1.0);
    hold(ax,'on');
    yline(ax, mean(off.dx_px), '--', ...
        sprintf('均值 %.2f px', mean(off.dx_px)), ...
        'Color',[0.82 0.16 0.13], 'LineWidth',1.2, 'LabelHorizontalAlignment','left');
    xlabel(ax, '帧序号');
    ylabel(ax, '深度质心 - 彩色分割质心 (px)');
    title(ax, '(a) 逐帧横向偏移');
    grid(ax,'on');

    % ---- 右：修正前后位移量箱线图
    ax = nexttile;
    data = [off.dx_pos_mm, off.dz_pos_mm];
    boxplot(ax, data, 'Labels', {'横向位移 \Deltax','深度位移 \Deltaz'}, ...
        'Whisker', 1.5, 'Widths', 0.55);
    yline(ax, 0, '-', 'Color',[0.40 0.40 0.40], 'LineWidth',1.0);
    ylabel(ax, '修正前后位置差 (mm)');
    title(ax, '(b) 修正位移量（注意非残余误差）');
    grid(ax,'on');

    sgtitle(tl, '图 3  S01 透明瓶：横向偏移稳定；位移量 ≠ 修正后残余误差', 'FontWeight','bold');
    exportgraphics(fig, fullfile(outDir,'fig3_lateral_offset.png'), 'Resolution',240);
end

function renderFig4(boardRow, outDir)
% 图 4：S05 标定板中心差和宽度差
    vals = [boardRow.width_diff_px, boardRow.center_diff_px];
    fig = figure('Color','w','Position',[100 100 820 460]);
    ax  = axes(fig); hold(ax,'on');

    b = bar(ax, [1 2], vals, 0.55, 'FaceColor','flat');
    b.CData = [0.46 0.42 0.68; 0.19 0.64 0.33];
    yline(ax, 0, 'k-', 'LineWidth',0.8);

    set(ax, 'XTick',1:2, 'XTickLabel', ...
        {'深度框宽 - ArUco 框宽', '深度框中心 - ArUco 框中心'});
    ylabel(ax, '差值 (px)');
    title(ax, '图 4  S05 标定板对齐检查（16 行统计，4 个标记，单次汇总）', 'FontWeight','bold');
    grid(ax,'on');

    text(ax, 1, vals(1)+0.9, sprintf('%+.2f px', vals(1)), ...
        'HorizontalAlignment','center', 'FontWeight','bold');
    text(ax, 2, vals(2)+0.9, sprintf('%+.2f px', vals(2)), ...
        'HorizontalAlignment','center', 'FontWeight','bold');

    annotation(fig, 'textbox', [0.66 0.70 0.28 0.18], ...
        'String', sprintf( ...
            '中心差 STD = %.2f px\n等效横向 = %.2f mm', ...
            boardRow.center_diff_std_px, boardRow.center_diff_mm), ...
        'FitBoxToText','on', 'BackgroundColor','white', ...
        'EdgeColor',[0.6 0.6 0.6], 'Margin',5);
    exportgraphics(fig, fullfile(outDir,'fig4_board_alignment.png'), 'Resolution',240);
end
