function plot_compute_heatmaps()
%PLOT_COMPUTE_HEATMAPS Raw AGX Orin and RP2040 computation latency.

cfg = analysis_config();
S = load(cfg.AnalysisMat,'ConditionSummary','ModeMap');
C = S.ConditionSummary;

fig = figure('Color','w','Visible',cfg.FigureVisible,'Units','inches', ...
    'Position',[1 1 12.5 8.0]);
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');

plotSimulationPanel(nexttile(layout),C,S.ModeMap,cfg.PSMissionName,cfg, ...
    "(a) "+cfg.PSFigureName+" AGX Orin");
plotSimulationPanel(nexttile(layout),C,S.ModeMap,cfg.CVMissionName,cfg, ...
    "(b) "+cfg.CVFigureName+" AGX Orin");
plotHILPanel(nexttile(layout),C,S.ModeMap,cfg.PSMissionName,cfg, ...
    "(c) "+cfg.PSFigureName+" RP2040");
plotHILPanel(nexttile(layout),C,S.ModeMap,cfg.CVMissionName,cfg, ...
    "(d) "+cfg.CVFigureName+" RP2040");

sgtitle(layout,'Computational Effects of Value-Ranked Candidate Restriction', ...
    'FontWeight','bold');
exportFigure(fig,cfg,'compute_effects_heatmaps');
end

function plotSimulationPanel(ax,C,ModeMap,mission,cfg,panelTitle)
if mission == cfg.PSMissionName
    modes = ModeMap.Mode(isfinite(ModeMap.PSRate));
else
    modes = cfg.CVFigureModes;
end
algorithms = cfg.AlgorithmOrder;
Mms = NaN(numel(algorithms),numel(modes));
labels = strings(size(Mms));

for i = 1:numel(algorithms)
    for j = 1:numel(modes)
        row = C(C.Mission == mission & C.Platform == "Simulation" & ...
            C.Algorithm == algorithms(i) & C.Mode == modes(j),:);
        if height(row) == 1
            Mms(i,j) = row.CombinedPerCallMsMean;
            labels(i,j) = formatMilliseconds(Mms(i,j));
        end
    end
end

soft = log10(1 + Mms/cfg.AGXOrinSoftLogPivotMs);
maxSoft = log10(1 + cfg.AGXOrinComputeMaxMs/cfg.AGXOrinSoftLogPivotMs);
imagesc(ax,soft,'AlphaData',isfinite(soft));
ax.Color = [0.87 0.87 0.87];
colormap(ax,greenYellowRed(256));
clim(ax,[0 maxSoft]);
set(ax,'YDir','normal');
xticks(ax,1:numel(modes)); xticklabels(ax,modes);
yticks(ax,1:numel(algorithms)); yticklabels(ax,algorithms);
xlabel(ax,'Restriction mode');
title(ax,panelTitle);
addCellText(ax,soft,labels,[0 maxSoft]);
cb = colorbar(ax);
tickMs = [0 1 10 100 1000];
cb.Ticks = log10(1 + tickMs/cfg.AGXOrinSoftLogPivotMs);
cb.TickLabels = compose('%g',tickMs);
cb.Label.String = 'Mean combined time per call (ms)';
end

function plotHILPanel(ax,C,ModeMap,mission,cfg,panelTitle)
if mission == cfg.PSMissionName
    modes = ModeMap.Mode(isfinite(ModeMap.PSRate));
else
    modes = cfg.CVFigureModes;
end
algorithms = cfg.AlgorithmOrder;
Mms = NaN(numel(algorithms),numel(modes));
labels = strings(size(Mms));

for i = 1:numel(algorithms)
    for j = 1:numel(modes)
        row = C(C.Mission == mission & C.Platform == "RP2040 HIL" & ...
            C.Algorithm == algorithms(i) & C.Mode == modes(j),:);
        if height(row) ~= 1
            labels(i,j) = "---";
            continue;
        end
        if row.CapStopped
            Mms(i,j) = cfg.HILCallLimitMs;
            labels(i,j) = "CAP";
        elseif isfinite(row.CombinedPerCallMsMean)
            Mms(i,j) = row.CombinedPerCallMsMean;
            labels(i,j) = formatMilliseconds(Mms(i,j));
        end
    end
end

soft = log10(1 + Mms/cfg.HILSoftLogPivotMs);
capSoft = log10(1 + cfg.HILCallLimitMs/cfg.HILSoftLogPivotMs);
imagesc(ax,soft,'AlphaData',isfinite(soft));
ax.Color = [0.87 0.87 0.87];
colormap(ax,greenYellowRed(256));
clim(ax,[0 capSoft]);
set(ax,'YDir','normal');
xticks(ax,1:numel(modes)); xticklabels(ax,modes);
yticks(ax,1:numel(algorithms)); yticklabels(ax,algorithms);
xlabel(ax,'Restriction mode');
title(ax,panelTitle);
addCellText(ax,soft,labels,[0 capSoft]);

cb = colorbar(ax);
tickMs = [0 100 1000 10000 30000];
cb.Ticks = log10(1 + tickMs/cfg.HILSoftLogPivotMs);
cb.TickLabels = compose('%g',tickMs);
cb.Label.String = 'Mean combined time per call (ms)';
end

function addCellText(ax,M,labels,colorLimits)
for i = 1:size(M,1)
    for j = 1:size(M,2)
        if strlength(labels(i,j)) == 0
            continue;
        end
        value = M(i,j);
        if ~isfinite(value)
            textColor = [0.2 0.2 0.2];
        else
            normalized = (value-colorLimits(1))/diff(colorLimits);
            textColor = [0 0 0];
            if normalized < 0.14 || normalized > 0.86
                textColor = [1 1 1];
            end
        end
        text(ax,j,i,labels(i,j),'HorizontalAlignment','center', ...
            'VerticalAlignment','middle','FontSize',8,'FontWeight','bold', ...
            'Color',textColor);
    end
end
end

function label = formatMilliseconds(value)
if value >= 10000
    label = sprintf('%.0fk',value/1000);
elseif value >= 1000
    label = sprintf('%.1fk',value/1000);
elseif value >= 100
    label = sprintf('%.0f',value);
else
    label = sprintf('%.1f',value);
end
end

function cmap = greenYellowRed(n)
anchors = [0.10 0.55 0.20; 0.70 0.84 0.38; 1.00 0.95 0.55; ...
    0.94 0.55 0.35; 0.70 0.10 0.12];
x = linspace(0,1,size(anchors,1));
xi = linspace(0,1,n);
cmap = interp1(x,anchors,xi,'linear');
end

function exportFigure(fig,cfg,stem)
if ~isfolder(cfg.FigureDir); mkdir(cfg.FigureDir); end
exportgraphics(fig,fullfile(cfg.FigureDir,stem+".png"), ...
    'Resolution',cfg.ExportResolution,'BackgroundColor','white');
if cfg.ExportPDF
    exportgraphics(fig,fullfile(cfg.FigureDir,stem+".pdf"), ...
        'ContentType','vector','BackgroundColor','white');
end
savefig(fig,fullfile(cfg.FigureDir,stem+".fig"));
end
