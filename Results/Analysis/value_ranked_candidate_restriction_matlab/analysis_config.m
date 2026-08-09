function cfg = analysis_config()
%ANALYSIS_CONFIG Paths, terminology, analysis settings, and figure options.
%
% Edit only this file when moving the analysis to another computer.
% All other scripts read their settings from this struct.

resultsRoot = "C:\Users\lottj\Desktop\research\decentralized benchmark\topk_filter_study\Results";

cfg = struct();

%% Terminology used in outputs
cfg.MethodName = "value-ranked candidate restriction";
cfg.PSMissionName = "Probabilistic Search";
cfg.CVMissionName = "Collaborative Visit";
cfg.PSFigureName = "CLIPS";
cfg.CVFigureName = "CV";
cfg.CVFigureModes = ["R0","R1","R2","R3","R4"];
cfg.AlgorithmOrder = ["CBAA","ACBBA","PI","HIPC","DMCHBA","DGA"];

%% Required source files
cfg.Files.PSSimulation = fullfile(resultsRoot, "Simulation", "Bayesian", "Published", ...
    "bayesian_topk_all_k_trial_level_system_and_timing_results.csv");
cfg.Files.CVSimulation = fullfile(resultsRoot, "Simulation", "Collaborative", "Published", ...
    "collaborative_topk_all_k_trial_level_system_and_timing_results.csv");

cfg.Files.PSHIL = fullfile(resultsRoot, "HIL", "Bayesian", "CompletedTopKCampaignV8", ...
    "bayesian_hil_topk_non_k1_combined_trial_level_system_and_timing_results.csv");
cfg.Files.CVHIL = fullfile(resultsRoot, "HIL", "Collaborative", "CompletedTopKCampaignV8", ...
    "collaborative_hil_topk_all_k_combined_trial_level_system_and_timing_results.csv");
cfg.Files.CombinedHIL = fullfile(resultsRoot, "HIL", "CombinedTopKCampaignV8", ...
    "hil_topk_combined_combined_trial_level_system_and_timing_results.csv");

% Condition-level files are required to distinguish ordinary failures from
% conditions stopped by the 30 s hardware limit. If these exact names are not
% found, verify_inputs.m searches the same folders for a file containing
% "condition" and "aggregate" in its name.
cfg.Files.PSHILConditions = fullfile(resultsRoot, "HIL", "Bayesian", "CompletedTopKCampaignV8", ...
    "bayesian_hil_topk_non_k1_condition_aggregate_metrics.csv");
cfg.Files.CVHILConditions = fullfile(resultsRoot, "HIL", "Collaborative", "CompletedTopKCampaignV8", ...
    "collaborative_hil_topk_all_k_condition_aggregate_metrics.csv");

cfg.Files.Sensitivity = fullfile(resultsRoot, "Simulation", "Sensitivity", "reports", ...
    "all_missions", "paired_step_results.csv");
cfg.Files.PSSensitivity = fullfile(resultsRoot, "Simulation", "Sensitivity", "reports", ...
    "bayesian_clue_search", "paired_step_results.csv");
cfg.Files.CVSensitivity = fullfile(resultsRoot, "Simulation", "Sensitivity", "reports", ...
    "collaborative_known_target_visit", "paired_step_results.csv");

cfg.Files.Physical = fullfile(resultsRoot, "Hardware", "PhysicalTrials", "CombinedTrials", ...
    "combined_metrics_per_trial.csv");

%% Outputs
cfg.OutputDir = fullfile(resultsRoot, "Analysis");
cfg.TableDir = fullfile(cfg.OutputDir, "Tables");
cfg.FigureDir = fullfile(cfg.OutputDir, "Figures");
cfg.CacheDir = fullfile(cfg.OutputDir, "Cache");
cfg.AnalysisMat = fullfile(cfg.OutputDir, "value_ranked_candidate_restriction_analysis.mat");
cfg.AnalysisWorkbook = fullfile(cfg.OutputDir, "value_ranked_candidate_restriction_analysis.xlsx");
cfg.SensitivityCache = fullfile(cfg.CacheDir, "sensitivity_permutation_checkpoint.mat");

%% Statistical settings
cfg.Alpha = 0.05;
cfg.NumPermutations = 99999;
cfg.RandomSeed = 20260805;
cfg.UseParallelIfAvailable = false;  % Parallel Computing Toolbox is not required.
cfg.ReuseSensitivityCache = true;
cfg.RunSensitivity = true;

%% Hardware and campaign settings
cfg.HILPlannedTrials = 25;
cfg.HILCallLimitMs = 30000;

%% Figure settings
cfg.ExportResolution = 600;
cfg.ExportPDF = true;
cfg.PerformanceYLim = [-5 40];
cfg.AGXOrinComputeMaxMs = 1000;           % shared raw-time color limit for AGX Orin panels
cfg.AGXOrinSoftLogPivotMs = 1;            % preserves contrast below 10 ms
cfg.HILSoftLogPivotMs = 100;             % softens the absolute-time heatmap below 1 s
cfg.FigureVisible = "on";                % set to "off" for unattended batch execution
cfg.IncludeSpecialK1InFailureFigure = true;

%% Physical-validation modes used in the experiment
cfg.PhysicalValidationModes = table( ...
    ["CBAA";"ACBBA";"PI";"HIPC";"DMCHBA";"DGA"], ...
    ["R6";"R4";"R4";"R4";"R5";string(missing)], ...
    [0.03;0.10;0.10;0.10;0.05;NaN], ...
    [11;36;36;36;18;NaN], ...
    'VariableNames', ["Algorithm","Mode","Rate","K"]);

end
