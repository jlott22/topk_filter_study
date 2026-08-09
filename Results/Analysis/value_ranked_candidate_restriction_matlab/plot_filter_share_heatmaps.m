function plot_filter_share_heatmaps()
%PLOT_FILTER_SHARE_HEATMAPS RP2040 filter share of combined computation.

cfg = analysis_config();
S = load(cfg.AnalysisMat,'ConditionSummary','ModeMap');
C = S.ConditionSummary;

fig = figure('Color','w','Visible',cfg.FigureVisible,'Units','inches', ...
    'Position',[1 1 11.5 4.2]);
layout = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
plotPanel(nexttile(layout),C,S.ModeMap,cfg.PSMissionName,cfg,"(a) "+cfg.PSFigureName);
plotPanel(nexttile(layout),C,S.ModeMap,cfg.CVMissionName,cfg,"(b) "+cfg.CVFigureName);
sgtitle(layout,'Candidate-Filtering Share of RP2040 Combined Computation', ...
    'FontWeight','bold');

exportFigure(fig,cfg,'hil_filter_share_heatmaps');
end

function plotPanel(ax,C,ModeMap,mission,cfg,panelTitle)
if mission == cfg.PSMissionName
    modes = ModeMap.Mode(isfinite(ModeMap.PSRate));
else
    modes = cfg.CVFigureModes;
end
algorithms = cfg.AlgorithmOrder;
M = NaN(numel(algorithms),numel(modes));
labels = strings(size(M));

for i = 1:numel(algorithms)
    for j = 1:numel(modes)
        row = C(C.Mission == mission & C.Platform == "RP2040 HIL" & ...
            C.Algorithm == algorithms(i) & C.Mode == modes(j),:);
        if height(row) ~= 1
            labels(i,j) = "---";
        elseif row.CapStopped
            M(i,j) = 100;
            labels(i,j) = "CAP";
        elseif isfinite(row.FilterShareAggregatePct)
            M(i,j) = row.FilterShareAggregatePct;
            labels(i,j) = sprintf('%.0f%%',M(i,j));
        end
    end
end

imagesc(ax,M,'AlphaData',isfinite(M));
ax.Color = [0.87 0.87 0.87];
colormap(ax,greenYellowRed(256));
clim(ax,[0 100]);
set(ax,'YDir','normal');
xticks(ax,1:numel(modes)); xticklabels(ax,modes);
yticks(ax,1:numel(algorithms)); yticklabels(ax,algorithms);
xlabel(ax,'Restriction mode');
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
cb.Label.String = 'Filter share of combined computation (%)';
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
