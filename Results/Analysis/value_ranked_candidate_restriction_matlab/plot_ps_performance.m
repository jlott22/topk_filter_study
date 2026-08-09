function plot_ps_performance()
%PLOT_PS_PERFORMANCE Plot paired maximum-step change for Probabilistic Search.

cfg = analysis_config();
S = load(cfg.AnalysisMat,'PairedChangeSummary','ModeMap');
T = S.PairedChangeSummary;

T = T(T.Mission == cfg.PSMissionName & T.Platform == "Simulation" & ...
    T.Metric == "Max Robot Steps",:);
modeOrder = S.ModeMap.Mode;
x = 1:numel(modeOrder);
colors = algorithmColors(numel(cfg.AlgorithmOrder));

fig = figure('Color','w','Visible',cfg.FigureVisible,'Units','inches', ...
    'Position',[1 1 7.1 4.3]);
ax = axes(fig); hold(ax,'on');

for a = 1:numel(cfg.AlgorithmOrder)
    algorithm = cfg.AlgorithmOrder(a);
    values = NaN(size(modeOrder));
    for m = 1:numel(modeOrder)
        row = T(T.Algorithm == algorithm & T.Mode == modeOrder(m),:);
        if height(row) == 1
            values(m) = row.MeanPctChange;
        end
    end
    plot(ax,x,values,'-o','LineWidth',1.8,'MarkerSize',5, ...
        'Color',colors(a,:),'MarkerFaceColor',colors(a,:), ...
        'DisplayName',algorithm);
end

yline(ax,0,'--','Color',[0.2 0.2 0.2],'LineWidth',0.9,'HandleVisibility','off');
xticks(ax,x); xticklabels(ax,modeOrder);
xlabel(ax,'Restriction mode (categorically spaced)');
ylabel(ax,'Paired maximum-step change from R0 (%)');
title(ax,cfg.PSFigureName+" Maximum-Step Performance Change from R0");
ylim(ax,cfg.PerformanceYLim);
grid(ax,'on'); box(ax,'on');
ax.GridAlpha = 0.18;
legend(ax,'Location','northoutside','Orientation','horizontal','NumColumns',3);

exportFigure(fig,cfg,'ps_max_step_change_by_restriction_mode');
end

function colors = algorithmColors(n)
colors = lines(max(n,7));
colors = colors(1:n,:);
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
