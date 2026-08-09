function plot_failure_rates()
%PLOT_FAILURE_RATES Trial failure/noncompletion heatmaps for every condition.
%
% Cell color uses noncompletion rate so timing-cap stops remain visible.
% The analysis workbook retains separate failed-trial and missing-trial rates.

cfg = analysis_config();
S = load(cfg.AnalysisMat,'FailureSummary','ModeMap');
F = S.FailureSummary;

fig = figure('Color','w','Visible',cfg.FigureVisible,'Units','inches', ...
    'Position',[1 1 12.5 8.0]);
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
plotPanel(nexttile(layout),F,S.ModeMap,cfg.PSMissionName,"Simulation",cfg, ...
    "(a) "+cfg.PSFigureName+" simulation");
plotPanel(nexttile(layout),F,S.ModeMap,cfg.CVMissionName,"Simulation",cfg, ...
    "(b) "+cfg.CVFigureName+" simulation");
plotPanel(nexttile(layout),F,S.ModeMap,cfg.PSMissionName,"RP2040 HIL",cfg, ...
    "(c) "+cfg.PSFigureName+" RP2040 HIL");
plotPanel(nexttile(layout),F,S.ModeMap,cfg.CVMissionName,"RP2040 HIL",cfg, ...
    "(d) "+cfg.CVFigureName+" RP2040 HIL");
sgtitle(layout,'Condition Failure and Noncompletion Rates','FontWeight','bold');

exportFigure(fig,cfg,'condition_failure_rates');
end

function plotPanel(ax,F,ModeMap,mission,platform,cfg,panelTitle)
if mission == cfg.PSMissionName
    modes = ModeMap.Mode(isfinite(ModeMap.PSRate));
else
    modes = cfg.CVFigureModes;
end
conditionLabels = modes;
if cfg.IncludeSpecialK1InFailureFigure
    conditionLabels(end+1) = "K=1";
end
algorithms = cfg.AlgorithmOrder;
M = NaN(numel(algorithms),numel(conditionLabels));
labels = strings(size(M));

for i = 1:numel(algorithms)
    for j = 1:numel(conditionLabels)
        row = F(F.Mission == mission & F.Platform == platform & ...
            F.Algorithm == algorithms(i) & F.DisplayCondition == conditionLabels(j),:);
        if height(row) ~= 1
            labels(i,j) = "---";
            continue;
        end
        if row.CapStopped
            M(i,j) = 100;
            labels(i,j) = "CAP";
        elseif isfinite(row.NoncompletionRatePct)
            M(i,j) = row.NoncompletionRatePct;
            labels(i,j) = sprintf('%.0f%%',M(i,j));
        else
            labels(i,j) = "?";
        end
    end
end

imagesc(ax,M,'AlphaData',isfinite(M));
ax.Color = [0.87 0.87 0.87];
colormap(ax,greenYellowRed(256));
clim(ax,[0 100]);
set(ax,'YDir','normal');
xticks(ax,1:numel(conditionLabels)); xticklabels(ax,conditionLabels);
yticks(ax,1:numel(algorithms)); yticklabels(ax,algorithms);
xlabel(ax,'Restriction condition');
title(ax,panelTitle);

for i = 1:size(M,1)
    for j = 1:size(M,2)
        if strlength(labels(i,j)) == 0; continue; end
        textColor = [0 0 0];
        if isfinite(M(i,j)) && (M(i,j) < 12 || M(i,j) > 88)
            textColor = [1 1 1];
        end
        text(ax,j,i,labels(i,j),'HorizontalAlignment','center', ...
            'VerticalAlignment','middle','FontSize',8,'FontWeight','bold', ...
            'Color',textColor);
    end
end
cb = colorbar(ax);
cb.Label.String = 'Trial noncompletion rate (%)';
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
