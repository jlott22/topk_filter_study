function run_all()
%RUN_ALL Build every analysis table and publication figure.

scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir);

build_analysis_tables();
plot_ps_performance();
plot_cv_performance();
plot_compute_heatmaps();
plot_filter_share_heatmaps();
plot_tradeoff_trajectories();
plot_failure_rates();

cfg = analysis_config();
fprintf('\nAll analysis outputs are available in:\n  %s\n',cfg.OutputDir);
end
