function plot_tradeoff_trajectories()
%PLOT_TRADEOFF_TRAJECTORIES Probabilistic Search performance-compute paths.
%
% X: paired maximum-step percentage change from R0.
% Y: paired allocator-time-per-call percentage change from R0.
% Negative Y values represent computation saved.
% Gold stars identify the modes used for physical validation. Open diamonds
% identify the balanced Pareto selections calculated from HIL maximum steps
% and mean combined allocator time per call.

cfg = analysis_config();
S = load(cfg.AnalysisMat,'PairedChangeSummary','ModeMap','HILFeasibility');
T = S.PairedChangeSummary;
H = S.HILFeasibility;
H = H(H.Mission == cfg.PSMissionName,:);

modes = S.ModeMap.Mode(isfinite(S.ModeMap.PSRate));
algorithms = cfg.AlgorithmOrder;

xValues = NaN(numel(algorithms),numel(modes));
yValues = NaN(numel(algorithms),numel(modes));
hardwareCapStopped = false(numel(algorithms),numel(modes));
for a = 1:numel(algorithms)
    for m = 1:numel(modes)
        xRow = T(T.Mission == cfg.PSMissionName & T.Platform == "Simulation" & ...
            T.Algorithm == algorithms(a) & T.Mode == modes(m) & ...
            T.Metric == "Max Robot Steps",:);
        yRow = T(T.Mission == cfg.PSMissionName & T.Platform == "Simulation" & ...
            T.Algorithm == algorithms(a) & T.Mode == modes(m) & ...
            T.Metric == "Combined Time Per Call",:);
        if height(xRow) == 1; xValues(a,m) = xRow.MeanPctChange; end
        if height(yRow) == 1; yValues(a,m) = yRow.MeanPctChange; end
        hRow = H(H.Algorithm == algorithms(a) & H.Mode == modes(m),:);
        if height(hRow) ~= 1
            error('Expected one HIL-feasibility row for %s %s; found %d.', ...
                algorithms(a),modes(m),height(hRow));
        end
        hardwareCapStopped(a,m) = hRow.CapStopped;
    end
end

finiteX = xValues(isfinite(xValues));
finiteY = yValues(isfinite(yValues));
xLimits = paddedLimits(finiteX,5,3);
yLimits = paddedLimits(finiteY,10,5);

fig = figure('Color','w','Visible',cfg.FigureVisible,'Units','inches', ...
    'Position',[1 1 6.0 7.2]);
layout = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
colors = algorithmColors(numel(algorithms));
legendAxis = gobjects(0);

for a = 1:numel(algorithms)
    ax = nexttile(layout); hold(ax,'on');
    if isempty(legendAxis); legendAxis = ax; end
    x = xValues(a,:);
    y = yValues(a,:);
    plot(ax,x,y,'-','Color',colors(a,:),'LineWidth',1.2, ...
        'HandleVisibility','off');

    valid = isfinite(x) & isfinite(y);
    feasible = valid & ~hardwareCapStopped(a,:);
    infeasible = valid & hardwareCapStopped(a,:);
    scatter(ax,x(feasible),y(feasible),28,'o', ...
        'MarkerFaceColor',colors(a,:),'MarkerEdgeColor',colors(a,:), ...
        'LineWidth',0.8,'HandleVisibility','off');
    scatter(ax,x(infeasible),y(infeasible),28,'o', ...
        'MarkerFaceColor','none','MarkerEdgeColor',colors(a,:), ...
        'LineWidth',1.2,'HandleVisibility','off');

    for m = 1:numel(modes)-1
        if all(isfinite([x(m),y(m),x(m+1),y(m+1)]))
            quiver(ax,x(m),y(m),x(m+1)-x(m),y(m+1)-y(m),0, ...
                'Color',[0.25 0.25 0.25],'LineWidth',0.55,'MaxHeadSize',0.35, ...
                'HandleVisibility','off');
        end
    end

    for m = 1:numel(modes)
        if ~isfinite(x(m)) || ~isfinite(y(m)); continue; end
        text(ax,x(m),y(m)," "+modes(m),'FontSize',5.8, ...
            'VerticalAlignment','bottom','Clipping','on');
    end

    selectedRow = cfg.PhysicalValidationModes( ...
        cfg.PhysicalValidationModes.Algorithm == algorithms(a),:);
    if ~isempty(selectedRow) && ~ismissing(selectedRow.Mode)
        selectedIndex = find(modes == selectedRow.Mode,1);
        if ~isempty(selectedIndex) && isfinite(x(selectedIndex)) && isfinite(y(selectedIndex))
            scatter(ax,x(selectedIndex),y(selectedIndex),70,'p', ...
                'MarkerFaceColor',[0.95 0.72 0.15],'MarkerEdgeColor','k', ...
                'LineWidth',0.8,'HandleVisibility','off');
        end
    end

    paretoRow = H(H.Algorithm == algorithms(a) & H.SelectedByPareto,:);
    if height(paretoRow) > 1
        error('Expected at most one selected Pareto row for %s; found %d.', ...
            algorithms(a),height(paretoRow));
    elseif height(paretoRow) == 1
        paretoIndex = find(modes == paretoRow.Mode,1);
        if ~isempty(paretoIndex) && isfinite(x(paretoIndex)) && isfinite(y(paretoIndex))
            scatter(ax,x(paretoIndex),y(paretoIndex),92,'d', ...
                'MarkerFaceColor','none','MarkerEdgeColor',[0.05 0.05 0.05], ...
                'LineWidth',1.35,'HandleVisibility','off');
        end
    end

    xline(ax,0,'--','Color',[0.35 0.35 0.35],'LineWidth',0.55,'HandleVisibility','off');
    yline(ax,0,'--','Color',[0.35 0.35 0.35],'LineWidth',0.55,'HandleVisibility','off');
    xlim(ax,xLimits); ylim(ax,yLimits);
    grid(ax,'on'); box(ax,'on');
    ax.GridAlpha = 0.16;
    ax.FontSize = 6.3;
    title(ax,algorithms(a),'FontSize',8.5,'FontWeight','bold');
end

xlabel(layout,'Paired maximum-step change from R0 (%)','FontSize',7.5);
ylabel(layout,'Paired allocator time per call change from R0 (%)','FontSize',7.5);
title(layout,cfg.PSFigureName+" Performance–Compute Tradeoff Trajectories", ...
    'FontSize',11,'FontWeight','bold');

feasibleHandle = scatter(legendAxis,NaN,NaN,28,'o', ...
    'MarkerFaceColor',[0.25 0.25 0.25],'MarkerEdgeColor',[0.25 0.25 0.25], ...
    'LineWidth',0.8);
infeasibleHandle = scatter(legendAxis,NaN,NaN,28,'o', ...
    'MarkerFaceColor','none','MarkerEdgeColor',[0.25 0.25 0.25], ...
    'LineWidth',1.2);
starHandle = scatter(legendAxis,NaN,NaN,70,'p', ...
    'MarkerFaceColor',[0.95 0.72 0.15],'MarkerEdgeColor','k','LineWidth',0.8);
paretoHandle = scatter(legendAxis,NaN,NaN,92,'d', ...
    'MarkerFaceColor','none','MarkerEdgeColor',[0.05 0.05 0.05], ...
    'LineWidth',1.35);
lgd = legend(legendAxis,[feasibleHandle,infeasibleHandle,starHandle,paretoHandle], ...
    {'RP2040 feasible','RP2040 infeasible','Physical-validation mode', ...
    'Per-call Pareto compromise'}, ...
    'Orientation','horizontal','NumColumns',2,'FontSize',6.8);
lgd.Layout.Tile = 'north';

exportFigure(fig,cfg,'ps_performance_compute_trajectories');
end

function limits = paddedLimits(values,roundTo,padding)
low = floor(min(values)/roundTo)*roundTo-padding;
high = ceil(max(values)/roundTo)*roundTo+padding;
if low == high
    low = low-roundTo;
    high = high+roundTo;
end
limits = [low high];
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
