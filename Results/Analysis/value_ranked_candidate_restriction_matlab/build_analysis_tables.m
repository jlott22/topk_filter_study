function build_analysis_tables()
%BUILD_ANALYSIS_TABLES Build the single source of truth for all figures and claims.
%
% Outputs:
%   value_ranked_candidate_restriction_analysis.mat
%   value_ranked_candidate_restriction_analysis.xlsx
%
% The workbook contains separate sheets for restriction-mode mapping,
% condition summaries, paired changes, failure rates, HIL feasibility,
% validation, manuscript claims, sensitivity tests, settings, and source audit.

cfg = analysis_config();
[cfg, SourceAudit, AnalysisSettings] = verify_inputs(cfg);
ModeMap = makeModeMap();

fprintf('Reading and standardizing primary campaign files...\n');
PSSimulation = standardizeSimulation(cfg.Files.PSSimulation, cfg.PSMissionName, ModeMap);
CVSimulation = standardizeSimulation(cfg.Files.CVSimulation, cfg.CVMissionName, ModeMap);
PSHIL = standardizeHIL(cfg.Files.PSHIL, cfg.PSMissionName, ModeMap);
CVHIL = standardizeHIL(cfg.Files.CVHIL, cfg.CVMissionName, ModeMap);
PSHILConditions = standardizeHILConditions(cfg.Files.PSHILConditions, cfg.PSMissionName, ModeMap);
CVHILConditions = standardizeHILConditions(cfg.Files.CVHILConditions, cfg.CVMissionName, ModeMap);

AllTrials = [PSSimulation; CVSimulation; PSHIL; CVHIL];
AllHILConditions = [PSHILConditions; CVHILConditions];

fprintf('Building condition and paired-change tables...\n');
ConditionSummary = buildConditionSummary(AllTrials, AllHILConditions, cfg);
PairedChangeSummary = buildPairedChangeSummary(AllTrials, cfg);
FailureSummary = buildFailureSummary(ConditionSummary);
HILFeasibility = buildHILFeasibility(ConditionSummary, cfg);

fprintf('Building HIL and physical validation tables...\n');
ValidationSummary = buildValidationSummary(PSSimulation, PSHIL, cfg.Files.Physical, ModeMap, cfg);
HILFeasibility = verifyPhysicalSelections(HILFeasibility, cfg.Files.Physical, ModeMap, cfg);

fprintf('Building claim-ready summary table...\n');
ClaimSummary = buildClaimSummary(ConditionSummary, PairedChangeSummary, FailureSummary, ...
    HILFeasibility, ValidationSummary, AllTrials, cfg);

if cfg.RunSensitivity
    fprintf('Running multivariate environmental-sensitivity analysis...\n');
    [SensitivityOmnibus, SensitivityPairwise, SensitivityLevelwise, ...
        SensitivitySignificantOnly, SensitivityCurveSummary] = ...
        runSensitivityAnalysis(cfg.Files.Sensitivity, cfg);
else
    SensitivityOmnibus = table();
    SensitivityPairwise = table();
    SensitivityLevelwise = table();
    SensitivitySignificantOnly = table();
    SensitivityCurveSummary = table();
end

fprintf('Saving MATLAB tables...\n');
save(cfg.AnalysisMat, ...
    'ModeMap', 'ConditionSummary', 'PairedChangeSummary', 'FailureSummary', ...
    'HILFeasibility', 'ValidationSummary', 'ClaimSummary', ...
    'SensitivityOmnibus', 'SensitivityPairwise', 'SensitivityLevelwise', ...
    'SensitivitySignificantOnly', 'SensitivityCurveSummary', ...
    'SourceAudit', 'AnalysisSettings', 'cfg', '-v7.3');

fprintf('Writing Excel workbook...\n');
writeAnalysisWorkbook(cfg.AnalysisWorkbook, ...
    ModeMap, ConditionSummary, PairedChangeSummary, FailureSummary, ...
    HILFeasibility, ValidationSummary, ClaimSummary, ...
    SensitivityOmnibus, SensitivityPairwise, SensitivityLevelwise, ...
    SensitivitySignificantOnly, SensitivityCurveSummary, ...
    SourceAudit, AnalysisSettings);

fprintf('\nAnalysis tables completed:\n  %s\n  %s\n\n', ...
    cfg.AnalysisMat, cfg.AnalysisWorkbook);

end

%% Restriction-mode mapping
function ModeMap = makeModeMap()
Mode = ("R" + string((0:7)'));
Order = (0:7)';

PSRate = [1.00;0.75;0.50;0.25;0.10;0.05;0.03;0.01];
PSPercent = 100 * PSRate;
PSK = [361;271;181;90;36;18;11;4];
PSDisplay = compose('%g%% (K=%g)', PSPercent, PSK);

CVRate = [1.00;0.75;0.50;0.25;0.10;0.05;0.04;NaN];
CVPercent = 100 * CVRate;
CVK = [50;38;25;13;5;3;2;NaN];
CVDisplay = strings(8,1);
for i = 1:8
    if isnan(CVRate(i))
        CVDisplay(i) = "---";
    else
        CVDisplay(i) = sprintf('%g%% (K=%g)', CVPercent(i), CVK(i));
    end
end

ModeMap = table(Mode, Order, PSRate, PSPercent, PSK, PSDisplay, ...
    CVRate, CVPercent, CVK, CVDisplay);
end

%% Standardization
function S = standardizeSimulation(filePath, missionName, ModeMap)
T = readtable(filePath, 'VariableNamingRule','preserve');
n = height(T);

S = table();
S.Mission = repmat(string(missionName),n,1);
S.Platform = repmat("Simulation",n,1);
S.Algorithm = string(T.algorithm);
S.Rate = double(T.top_k_rate);
S.K = double(T.top_k_max_cells);
[S.Mode,S.RestrictionOrder,S.ConditionLabel,S.IsSpecialK1] = ...
    mapConditions(missionName,S.Rate,S.K,ModeMap);
S.TrialID = double(T.trial_id);
S.Status = lower(string(getStringVariable(T,"trial_status","completed")));
S.FailureType = string(getStringVariable(T,"failure_type",""));
S.AnalysisValid = S.Status == "completed";

if ismember("max_steps_any_robot", string(T.Properties.VariableNames))
    S.MaxRobotSteps = double(T.max_steps_any_robot);
elseif ismember("max_robot_steps", string(T.Properties.VariableNames))
    S.MaxRobotSteps = double(T.max_robot_steps);
else
    S.MaxRobotSteps = NaN(n,1);
end
S.TotalTeamSteps = getNumericVariable(T,"total_team_steps",NaN(n,1));
S.AllocatorCalls = getNumericVariable(T,"allocator_calls_total",NaN(n,1));
S.FilterCalls = getNumericVariable(T,"candidate_filter_calls_total",NaN(n,1));
S.CombinedTotalMs = getNumericVariable(T,"allocator_time_ms_team_total",NaN(n,1));
S.AllocationOnlyTotalMs = getNumericVariable(T,"allocator_solve_time_ms_team_total",NaN(n,1));
S.FilterTotalMs = getNumericVariable(T,"candidate_filter_time_ms_team_total",NaN(n,1));
S.MaxCombinedCallMs = getNumericVariable(T,"allocator_time_ms_team_max",NaN(n,1));

S.CombinedPerCallMs = safeDivide(S.CombinedTotalMs,S.AllocatorCalls);
S.AllocationOnlyPerCallMs = safeDivide(S.AllocationOnlyTotalMs,S.AllocatorCalls);
S.FilterPerCallMs = safeDivide(S.FilterTotalMs,S.AllocatorCalls);
S.FilterPerInvocationMs = safeDivide(S.FilterTotalMs,S.FilterCalls);
S.SourceFile = repmat(string(filePath),n,1);
end

function S = standardizeHIL(filePath, missionName, ModeMap)
T = readtable(filePath, 'VariableNamingRule','preserve');
n = height(T);

S = table();
S.Mission = repmat(string(missionName),n,1);
S.Platform = repmat("RP2040 HIL",n,1);
S.Algorithm = string(T.algorithm);
S.Rate = double(T.top_k_rate);
if ismember("top_k_cells",string(T.Properties.VariableNames))
    S.K = double(T.top_k_cells);
else
    S.K = double(T.top_k_max_cells);
end
[S.Mode,S.RestrictionOrder,S.ConditionLabel,S.IsSpecialK1] = ...
    mapConditions(missionName,S.Rate,S.K,ModeMap);
S.TrialID = double(T.trial_id);
S.Status = lower(string(getStringVariable(T,"trial_status","completed")));
S.FailureType = repmat("",n,1);
if ismember("analysis_valid", string(T.Properties.VariableNames))
    rawAnalysisValid = lower(strtrim(string(T.analysis_valid)));
    analysisValid = ismember(rawAnalysisValid, ["true","1","yes","y"]);
else
    analysisValid = true(n,1);
end
S.AnalysisValid = S.Status == "completed" & analysisValid;

if ismember("max_steps_any_robot", string(T.Properties.VariableNames))
    S.MaxRobotSteps = double(T.max_steps_any_robot);
elseif ismember("max_robot_steps", string(T.Properties.VariableNames))
    S.MaxRobotSteps = double(T.max_robot_steps);
else
    S.MaxRobotSteps = NaN(n,1);
end
S.TotalTeamSteps = getNumericVariable(T,"total_team_steps",NaN(n,1));
S.AllocatorCalls = getNumericVariable(T,"timed_allocator_call_count", ...
    getNumericVariable(T,"allocator_call_count",NaN(n,1)));
S.FilterCalls = getNumericVariable(T,"candidate_filter_invocation_count",NaN(n,1));
S.CombinedTotalMs = getNumericVariable(T,"allocator_time_total_us",NaN(n,1)) / 1000;
S.AllocationOnlyTotalMs = getNumericVariable(T,"allocator_exclusive_time_total_us",NaN(n,1)) / 1000;
S.FilterTotalMs = getNumericVariable(T,"candidate_filter_time_total_us",NaN(n,1)) / 1000;
S.MaxCombinedCallMs = getNumericVariable(T,"allocator_time_max_us",NaN(n,1)) / 1000;

S.CombinedPerCallMs = safeDivide(S.CombinedTotalMs,S.AllocatorCalls);
S.AllocationOnlyPerCallMs = safeDivide(S.AllocationOnlyTotalMs,S.AllocatorCalls);
S.FilterPerCallMs = safeDivide(S.FilterTotalMs,S.AllocatorCalls);
S.FilterPerInvocationMs = safeDivide(S.FilterTotalMs,S.FilterCalls);
S.SourceFile = repmat(string(filePath),n,1);
end

function C = standardizeHILConditions(filePath, missionName, ModeMap)
T = readtable(filePath, 'VariableNamingRule','preserve');
n = height(T);
C = table();
C.Mission = repmat(string(missionName),n,1);
C.Platform = repmat("RP2040 HIL",n,1);
C.Algorithm = string(T.algorithm);
C.Rate = double(T.top_k_rate);
if ismember("top_k_cells",string(T.Properties.VariableNames))
    C.K = double(T.top_k_cells);
else
    C.K = double(T.top_k_max_cells);
end
[C.Mode,C.RestrictionOrder,C.ConditionLabel,C.IsSpecialK1] = ...
    mapConditions(missionName,C.Rate,C.K,ModeMap);
C.ConditionStatus = lower(string(T.status));
C.StopReason = string(getStringVariable(T,"stopped_reason",""));
C.CompletedTrials = double(T.completed_trial_count);
C.FailedTrials = double(T.failed_trial_count);
C.PlannedTrials = double(T.planned_trial_count);
C.MaxObservedCallMs = double(T.allocator_time_max_us) / 1000;
C.CapStopped = contains(lower(C.StopReason),"30s") | ...
    contains(lower(C.StopReason),"timing_unusable") | C.MaxObservedCallMs >= 30000;
C.SourceFile = repmat(string(filePath),n,1);
end

function [Mode,Order,ConditionLabel,IsSpecialK1] = mapConditions(missionName,rate,k,ModeMap)
n = numel(rate);
Mode = strings(n,1);
Order = NaN(n,1);
ConditionLabel = strings(n,1);
IsSpecialK1 = false(n,1);

if missionName == "Probabilistic Search"
    mapRate = ModeMap.PSRate;
    mapK = ModeMap.PSK;
else
    mapRate = ModeMap.CVRate;
    mapK = ModeMap.CVK;
end

for i = 1:n
    if k(i) == 1
        Mode(i) = missing;
        Order(i) = 99;
        ConditionLabel(i) = "K=1";
        IsSpecialK1(i) = true;
        continue;
    end
    idx = find(abs(mapRate-rate(i)) < 1e-8 & mapK == k(i),1);
    if isempty(idx)
        idx = find(abs(mapRate-rate(i)) < 1e-8,1);
    end
    if isempty(idx)
        Mode(i) = missing;
        Order(i) = 98;
        ConditionLabel(i) = sprintf('%g%% (K=%g)',100*rate(i),k(i));
    else
        Mode(i) = ModeMap.Mode(idx);
        Order(i) = ModeMap.Order(idx);
        ConditionLabel(i) = ModeMap.Mode(idx);
    end
end
end

function value = getNumericVariable(T,name,defaultValue)
if ismember(name,string(T.Properties.VariableNames))
    value = double(T.(name));
else
    value = defaultValue;
end
end

function value = getStringVariable(T,name,defaultValue)
if ismember(name,string(T.Properties.VariableNames))
    value = string(T.(name));
else
    value = repmat(string(defaultValue),height(T),1);
end
end

function out = safeDivide(numerator,denominator)
out = numerator ./ denominator;
out(~isfinite(out) | denominator == 0) = NaN;
end

%% Condition-level descriptive summaries and failure accounting
function ConditionSummary = buildConditionSummary(AllTrials, AllHILConditions, cfg)
keyVars = ["Mission","Platform","Algorithm","Mode","RestrictionOrder", ...
    "ConditionLabel","Rate","K","IsSpecialK1"];
allKeys = [AllTrials(:,keyVars);AllHILConditions(:,keyVars)];
missingModeToken = "__MISSING_MODE__";
allKeys.Mode(ismissing(allKeys.Mode)) = missingModeToken;
keys = unique(allKeys,'rows','stable');
keys.Mode(keys.Mode == missingModeToken) = missing;
keys = sortrows(keys,["Mission","Platform","Algorithm","RestrictionOrder"]);

n = height(keys);
PlannedTrials = NaN(n,1);
CompletedTrials = zeros(n,1);
FailedTrials = zeros(n,1);
MissingTrials = zeros(n,1);
CompletionRatePct = NaN(n,1);
FailureRatePct = NaN(n,1);
NoncompletionRatePct = NaN(n,1);
ConditionStatus = strings(n,1);
StopReason = strings(n,1);
CapStopped = false(n,1);

metricNames = ["MaxRobotSteps","TotalTeamSteps","AllocatorCalls", ...
    "CombinedPerCallMs","AllocationOnlyPerCallMs","FilterPerCallMs", ...
    "FilterPerInvocationMs","CombinedTotalMs","AllocationOnlyTotalMs", ...
    "FilterTotalMs","MaxCombinedCallMs"];
statNames = ["Mean","SD","Median","Q1","Q3","Min","Max","N"];
metricOutput = struct();
for m = metricNames
    for s = statNames
        metricOutput.(m+s) = NaN(n,1);
    end
end
FilterShareAggregatePct = NaN(n,1);
FilterShareTrialMeanPct = NaN(n,1);
FilterShareTrialSDPct = NaN(n,1);
MaxObservedCallMs = NaN(n,1);

for i = 1:n
    key = keys(i,:);
    rowMask = AllTrials.Mission == key.Mission & ...
        AllTrials.Platform == key.Platform & ...
        AllTrials.Algorithm == key.Algorithm & ...
        AllTrials.ConditionLabel == key.ConditionLabel & ...
        abs(AllTrials.Rate-key.Rate) < 1e-8 & AllTrials.K == key.K;
    rows = AllTrials(rowMask,:);
    validRows = rows(rows.Status == "completed" & rows.AnalysisValid,:);

    if key.Platform == "Simulation"
        baselineMask = AllTrials.Mission == key.Mission & ...
            AllTrials.Platform == "Simulation" & ...
            AllTrials.Algorithm == key.Algorithm & AllTrials.Mode == "R0";
        PlannedTrials(i) = numel(unique(AllTrials.TrialID(baselineMask)));
        CompletedTrials(i) = height(validRows);
        FailedTrials(i) = sum(rows.Status ~= "completed");
        MissingTrials(i) = max(0,PlannedTrials(i)-height(rows));
        if FailedTrials(i) > 0 || MissingTrials(i) > 0
            ConditionStatus(i) = "completed_with_failures";
        else
            ConditionStatus(i) = "complete";
        end
    else
        conditionMask = AllHILConditions.Mission == key.Mission & ...
            AllHILConditions.Algorithm == key.Algorithm & ...
            AllHILConditions.ConditionLabel == key.ConditionLabel & ...
            abs(AllHILConditions.Rate-key.Rate) < 1e-8 & ...
            AllHILConditions.K == key.K;
        conditionRows = AllHILConditions(conditionMask,:);
        if ~isempty(conditionRows)
            PlannedTrials(i) = conditionRows.PlannedTrials(1);
            CompletedTrials(i) = conditionRows.CompletedTrials(1);
            FailedTrials(i) = conditionRows.FailedTrials(1);
            MissingTrials(i) = max(0,PlannedTrials(i)-CompletedTrials(i)-FailedTrials(i));
            ConditionStatus(i) = conditionRows.ConditionStatus(1);
            StopReason(i) = conditionRows.StopReason(1);
            CapStopped(i) = conditionRows.CapStopped(1);
            MaxObservedCallMs(i) = conditionRows.MaxObservedCallMs(1);
        else
            PlannedTrials(i) = cfg.HILPlannedTrials;
            CompletedTrials(i) = height(validRows);
            FailedTrials(i) = sum(rows.Status ~= "completed");
            MissingTrials(i) = max(0,PlannedTrials(i)-CompletedTrials(i)-FailedTrials(i));
            ConditionStatus(i) = "condition_metadata_missing";
        end
    end

    if PlannedTrials(i) > 0
        CompletionRatePct(i) = 100 * CompletedTrials(i) / PlannedTrials(i);
        FailureRatePct(i) = 100 * FailedTrials(i) / PlannedTrials(i);
        NoncompletionRatePct(i) = 100 * (PlannedTrials(i)-CompletedTrials(i)) / PlannedTrials(i);
    end

    for m = metricNames
        stats = descriptiveStats(validRows.(m));
        for s = statNames
            metricOutput.(m+s)(i) = stats.(s);
        end
    end

    if ~isempty(validRows)
        totalCombined = sum(validRows.CombinedTotalMs,'omitnan');
        totalFilter = sum(validRows.FilterTotalMs,'omitnan');
        if totalCombined > 0
            FilterShareAggregatePct(i) = 100 * totalFilter / totalCombined;
        end
        shares = 100 * safeDivide(validRows.FilterTotalMs,validRows.CombinedTotalMs);
        FilterShareTrialMeanPct(i) = mean(shares,'omitnan');
        FilterShareTrialSDPct(i) = std(shares,0,'omitnan');
        MaxObservedCallMs(i) = max([MaxObservedCallMs(i);validRows.MaxCombinedCallMs],[],'omitnan');
    end
end

ConditionSummary = keys;
ConditionSummary.PlannedTrials = PlannedTrials;
ConditionSummary.CompletedTrials = CompletedTrials;
ConditionSummary.FailedTrials = FailedTrials;
ConditionSummary.MissingTrials = MissingTrials;
ConditionSummary.CompletionRatePct = CompletionRatePct;
ConditionSummary.FailureRatePct = FailureRatePct;
ConditionSummary.NoncompletionRatePct = NoncompletionRatePct;
ConditionSummary.ConditionStatus = ConditionStatus;
ConditionSummary.StopReason = StopReason;
ConditionSummary.CapStopped = CapStopped;

for m = metricNames
    for s = statNames
        ConditionSummary.(m+s) = metricOutput.(m+s);
    end
end
ConditionSummary.FilterShareAggregatePct = FilterShareAggregatePct;
ConditionSummary.FilterShareTrialMeanPct = FilterShareTrialMeanPct;
ConditionSummary.FilterShareTrialSDPct = FilterShareTrialSDPct;
ConditionSummary.MaxObservedCallMs = MaxObservedCallMs;
end

function stats = descriptiveStats(values)
values = double(values);
values = values(isfinite(values));
stats = struct('Mean',NaN,'SD',NaN,'Median',NaN,'Q1',NaN,'Q3',NaN, ...
    'Min',NaN,'Max',NaN,'N',0);
if isempty(values)
    return;
end
stats.Mean = mean(values);
stats.SD = std(values,0);
stats.Median = median(values);
stats.Q1 = prctile(values,25);
stats.Q3 = prctile(values,75);
stats.Min = min(values);
stats.Max = max(values);
stats.N = numel(values);
end

function FailureSummary = buildFailureSummary(ConditionSummary)
vars = ["Mission","Platform","Algorithm","Mode","RestrictionOrder", ...
    "ConditionLabel","Rate","K","IsSpecialK1","PlannedTrials", ...
    "CompletedTrials","FailedTrials","MissingTrials","CompletionRatePct", ...
    "FailureRatePct","NoncompletionRatePct","ConditionStatus","StopReason", ...
    "CapStopped","MaxObservedCallMs"];
FailureSummary = ConditionSummary(:,vars);
FailureSummary.DisplayCondition = FailureSummary.ConditionLabel;
FailureSummary = movevars(FailureSummary,"DisplayCondition",'After',"Algorithm");
end

%% Paired percentage changes from R0
function PairedChangeSummary = buildPairedChangeSummary(AllTrials, cfg)
metricFields = ["MaxRobotSteps","TotalTeamSteps","AllocatorCalls", ...
    "CombinedPerCallMs","CombinedTotalMs"];
metricLabels = ["Max Robot Steps","Total Team Steps","Allocator Calls", ...
    "Combined Time Per Call","Cumulative Combined Time"];
metricClasses = ["Performance","Performance","Behavior","Compute","Compute"];

PairedChangeSummary = emptyPairedChangeTable();
keyTable = unique(AllTrials(:,["Mission","Platform","Algorithm"]),'rows','stable');

for keyIndex = 1:height(keyTable)
    mission = keyTable.Mission(keyIndex);
    platform = keyTable.Platform(keyIndex);
    algorithm = keyTable.Algorithm(keyIndex);

    subset = AllTrials(AllTrials.Mission == mission & ...
        AllTrials.Platform == platform & AllTrials.Algorithm == algorithm & ...
        ~ismissing(AllTrials.Mode),:);
    subset = subset(subset.Status == "completed" & subset.AnalysisValid,:);
    baseline = subset(subset.Mode == "R0",:);
    if isempty(baseline)
        continue;
    end

    modeKeys = unique(subset(:,["Mode","RestrictionOrder","Rate","K"]),'rows','stable');
    modeKeys = sortrows(modeKeys,"RestrictionOrder");
    baselineCount = numel(unique(baseline.TrialID));

    for modeIndex = 1:height(modeKeys)
        mode = modeKeys.Mode(modeIndex);
        current = subset(subset.Mode == mode,:);

        for metricIndex = 1:numel(metricFields)
            field = metricFields(metricIndex);
            metric = metricLabels(metricIndex);
            metricClass = metricClasses(metricIndex);

            if mode == "R0"
                values = baseline.(field);
                values = values(isfinite(values) & values ~= 0);
                changes = zeros(size(values));
            else
                B = baseline(:,["TrialID",field]);
                C = current(:,["TrialID",field]);
                B.Properties.VariableNames{2} = 'BaselineValue';
                C.Properties.VariableNames{2} = 'CurrentValue';
                pairs = innerjoin(B,C,'Keys','TrialID');
                valid = isfinite(pairs.BaselineValue) & isfinite(pairs.CurrentValue) & ...
                    pairs.BaselineValue ~= 0;
                pairs = pairs(valid,:);
                changes = 100 * (pairs.CurrentValue-pairs.BaselineValue) ./ pairs.BaselineValue;
            end

            stats = pairedChangeStats(changes,cfg.Alpha);
            if metricClass == "Compute"
                meanSaving = -stats.Mean;
            else
                meanSaving = NaN;
            end

            newRow = table(mission,platform,algorithm,mode, ...
                modeKeys.RestrictionOrder(modeIndex),modeKeys.Rate(modeIndex), ...
                modeKeys.K(modeIndex),metric,metricClass,stats.N, ...
                100*stats.N/baselineCount,stats.Mean,stats.SD,stats.Median, ...
                stats.CI95Low,stats.CI95High,stats.TTestP,NaN,stats.WilcoxonP,NaN, ...
                meanSaving, ...
                'VariableNames',PairedChangeSummary.Properties.VariableNames);
            PairedChangeSummary = [PairedChangeSummary;newRow]; %#ok<AGROW>
        end
    end
end

% Holm correction across nonbaseline modes within each mission, platform,
% algorithm, and metric family.
groupKeys = unique(PairedChangeSummary(:,["Mission","Platform","Algorithm","Metric"]), ...
    'rows','stable');
for g = 1:height(groupKeys)
    mask = PairedChangeSummary.Mission == groupKeys.Mission(g) & ...
        PairedChangeSummary.Platform == groupKeys.Platform(g) & ...
        PairedChangeSummary.Algorithm == groupKeys.Algorithm(g) & ...
        PairedChangeSummary.Metric == groupKeys.Metric(g) & ...
        PairedChangeSummary.Mode ~= "R0";
    PairedChangeSummary.TTestPHolm(mask) = holmAdjust(PairedChangeSummary.TTestP(mask));
    PairedChangeSummary.WilcoxonPHolm(mask) = holmAdjust(PairedChangeSummary.WilcoxonP(mask));
end
end

function T = emptyPairedChangeTable()
T = table('Size',[0 21], ...
    'VariableTypes',{'string','string','string','string','double','double','double', ...
    'string','string','double','double','double','double','double','double','double', ...
    'double','double','double','double','double'}, ...
    'VariableNames',{'Mission','Platform','Algorithm','Mode','RestrictionOrder', ...
    'Rate','K','Metric','MetricClass','NPairs','PairCompletenessPct', ...
    'MeanPctChange','SDPctChange','MedianPctChange','CI95Low','CI95High', ...
    'TTestP','TTestPHolm','WilcoxonP','WilcoxonPHolm','MeanComputeSavingPct'});
end

function stats = pairedChangeStats(changes,alpha)
changes = double(changes);
changes = changes(isfinite(changes));
stats = struct('N',numel(changes),'Mean',NaN,'SD',NaN,'Median',NaN, ...
    'CI95Low',NaN,'CI95High',NaN,'TTestP',NaN,'WilcoxonP',NaN);
if isempty(changes)
    return;
end
stats.Mean = mean(changes);
stats.SD = std(changes,0);
stats.Median = median(changes);
if numel(changes) == 1
    stats.CI95Low = changes;
    stats.CI95High = changes;
    stats.TTestP = NaN;
    stats.WilcoxonP = NaN;
    return;
end
if all(abs(changes) < eps)
    stats.CI95Low = 0;
    stats.CI95High = 0;
    stats.TTestP = 1;
    stats.WilcoxonP = 1;
    return;
end
[~,p,ci] = ttest(changes,0,'Alpha',alpha);
stats.TTestP = p;
stats.CI95Low = ci(1);
stats.CI95High = ci(2);
try
    stats.WilcoxonP = signrank(changes,0,'method','approximate');
catch
    stats.WilcoxonP = NaN;
end
end

function adjusted = holmAdjust(pValues)
pValues = double(pValues);
adjusted = NaN(size(pValues));
valid = find(isfinite(pValues));
if isempty(valid)
    return;
end
[sortedP,order] = sort(pValues(valid));
m = numel(sortedP);
adjSorted = NaN(m,1);
running = 0;
for i = 1:m
    running = max(running,min(1,(m-i+1)*sortedP(i)));
    adjSorted(i) = running;
end
adjusted(valid(order)) = adjSorted;
end

%% HIL feasibility, per-call Pareto frontier, and balanced compromise
function HILFeasibility = buildHILFeasibility(ConditionSummary,cfg)
mask = ConditionSummary.Platform == "RP2040 HIL" & ~ismissing(ConditionSummary.Mode);
vars = ["Mission","Algorithm","Mode","RestrictionOrder","Rate","K", ...
    "PlannedTrials","CompletedTrials","FailedTrials","CompletionRatePct", ...
    "ConditionStatus","StopReason","CapStopped","MaxObservedCallMs", ...
    "MaxRobotStepsMean","CombinedTotalMsMean","CombinedPerCallMsMean"];
HILFeasibility = ConditionSummary(mask,vars);
HILFeasibility.Feasible = HILFeasibility.CompletedTrials == HILFeasibility.PlannedTrials & ...
    HILFeasibility.FailedTrials == 0 & ~HILFeasibility.CapStopped & ...
    HILFeasibility.MaxObservedCallMs < cfg.HILCallLimitMs & ...
    isfinite(HILFeasibility.MaxRobotStepsMean) & isfinite(HILFeasibility.CombinedPerCallMsMean);
HILFeasibility.ParetoEfficient = false(height(HILFeasibility),1);
HILFeasibility.NormalizedMaxSteps = NaN(height(HILFeasibility),1);
HILFeasibility.NormalizedPerCallCompute = NaN(height(HILFeasibility),1);
HILFeasibility.BalancedCost = NaN(height(HILFeasibility),1);
HILFeasibility.SelectedByPareto = false(height(HILFeasibility),1);
HILFeasibility.UsedInPhysicalTrials = false(height(HILFeasibility),1);
HILFeasibility.PhysicalSelectionMatches = false(height(HILFeasibility),1);

keys = unique(HILFeasibility(:,["Mission","Algorithm"]),'rows','stable');
for keyIndex = 1:height(keys)
    maskGroup = HILFeasibility.Mission == keys.Mission(keyIndex) & ...
        HILFeasibility.Algorithm == keys.Algorithm(keyIndex) & HILFeasibility.Feasible;
    idx = find(maskGroup);
    if isempty(idx)
        continue;
    end

    S = HILFeasibility.MaxRobotStepsMean(idx);
    C = HILFeasibility.CombinedPerCallMsMean(idx);
    pareto = true(numel(idx),1);
    for i = 1:numel(idx)
        dominated = (S <= S(i) & C <= C(i) & (S < S(i) | C < C(i)));
        dominated(i) = false;
        if any(dominated)
            pareto(i) = false;
        end
    end
    paretoIdx = idx(pareto);
    HILFeasibility.ParetoEfficient(paretoIdx) = true;

    sNorm = minMaxNormalize(HILFeasibility.MaxRobotStepsMean(paretoIdx));
    cNorm = minMaxNormalize(HILFeasibility.CombinedPerCallMsMean(paretoIdx));
    balance = sqrt(0.5*sNorm.^2 + 0.5*cNorm.^2);
    HILFeasibility.NormalizedMaxSteps(paretoIdx) = sNorm;
    HILFeasibility.NormalizedPerCallCompute(paretoIdx) = cNorm;
    HILFeasibility.BalancedCost(paretoIdx) = balance;
    [~,best] = min(balance);
    HILFeasibility.SelectedByPareto(paretoIdx(best)) = true;
end
end

function normalized = minMaxNormalize(values)
values = double(values);
rangeValue = max(values)-min(values);
if rangeValue == 0
    normalized = zeros(size(values));
else
    normalized = (values-min(values))/rangeValue;
end
end

function HILFeasibility = verifyPhysicalSelections(HILFeasibility,physicalFile,ModeMap,cfg)
P = readtable(physicalFile,'VariableNamingRule','preserve');
algorithms = string(P.algorithm);
rates = double(P.top_k_rate);
physicalMap = table();
physicalMap.Algorithm = unique(algorithms,'stable');
physicalMap.Rate = NaN(height(physicalMap),1);
physicalMap.Mode = strings(height(physicalMap),1);
for i = 1:height(physicalMap)
    values = unique(rates(algorithms == physicalMap.Algorithm(i)));
    if numel(values) == 1
        physicalMap.Rate(i) = values;
        [mode,~,~,~] = mapConditions(cfg.PSMissionName,values, ...
            modeKForRate(cfg.PSMissionName,values,ModeMap),ModeMap);
        physicalMap.Mode(i) = mode;
    end
end

for i = 1:height(HILFeasibility)
    match = physicalMap.Algorithm == HILFeasibility.Algorithm(i) & ...
        abs(physicalMap.Rate-HILFeasibility.Rate(i)) < 1e-8;
    HILFeasibility.UsedInPhysicalTrials(i) = any(match);
    HILFeasibility.PhysicalSelectionMatches(i) = ...
        HILFeasibility.SelectedByPareto(i) && HILFeasibility.UsedInPhysicalTrials(i);
end

% Verify every observed physical setting matches the HIL compromise selection.
for i = 1:height(physicalMap)
    selected = HILFeasibility.Algorithm == physicalMap.Algorithm(i) & ...
        HILFeasibility.Mission == cfg.PSMissionName & HILFeasibility.SelectedByPareto;
    if ~any(selected)
        warning('No HIL Pareto compromise was available for physical algorithm %s.', ...
            physicalMap.Algorithm(i));
    elseif abs(HILFeasibility.Rate(find(selected,1))-physicalMap.Rate(i)) > 1e-8
        warning(['Physical setting for %s (%g%%) does not match the recalculated ', ...
            'HIL Pareto compromise (%g%%).'],physicalMap.Algorithm(i), ...
            100*physicalMap.Rate(i),100*HILFeasibility.Rate(find(selected,1)));
    end
end
end

function k = modeKForRate(missionName,rate,ModeMap)
if missionName == "Probabilistic Search"
    idx = find(abs(ModeMap.PSRate-rate)<1e-8,1);
    k = ModeMap.PSK(idx);
else
    idx = find(abs(ModeMap.CVRate-rate)<1e-8,1);
    k = ModeMap.CVK(idx);
end
end

%% Validation summaries (tables only; no validation figure)
function ValidationSummary = buildValidationSummary(PSSimulation,PSHIL,physicalFile,ModeMap,cfg)
ValidationSummary = emptyValidationTable();

% Simulation versus HIL: match algorithm, retained rate, and trial ID.
S = PSSimulation(PSSimulation.Status == "completed" & PSSimulation.AnalysisValid,:);
H = PSHIL(PSHIL.Status == "completed" & PSHIL.AnalysisValid,:);
S = S(:,["Algorithm","Rate","TrialID","MaxRobotSteps","TotalTeamSteps"]);
H = H(:,["Algorithm","Rate","TrialID","MaxRobotSteps","TotalTeamSteps"]);
S.Properties.VariableNames(4:5) = ["SimulationMax","SimulationTotal"];
H.Properties.VariableNames(4:5) = ["ObservedMax","ObservedTotal"];
HILPairs = innerjoin(S,H,'Keys',["Algorithm","Rate","TrialID"]);

ValidationSummary = [ValidationSummary; validationRowsFromPairs( ...
    HILPairs,"Simulation vs RP2040 HIL",false,cfg)];

% Simulation versus physical trials.
P = readtable(physicalFile,'VariableNamingRule','preserve');
Physical = table();
Physical.Algorithm = string(P.algorithm);
Physical.Rate = double(P.top_k_rate);
Physical.TrialID = double(P.source_episode);
Physical.ObservedMax = double(P.max_steps_any_robot);
Physical.ObservedTotal = double(P.total_steps);
Physical.ReviewFlag = lower(string(getStringVariable(P,"review_flag","no")));

PhysicalPairs = innerjoin(S,Physical,'Keys',["Algorithm","Rate","TrialID"]);
ValidationSummary = [ValidationSummary; validationRowsFromPairs( ...
    PhysicalPairs,"Simulation vs Physical",true,cfg)];

% Add the restriction mode used in each validation row where a group refers
% to a single algorithm and physical condition.
ValidationSummary.Mode = strings(height(ValidationSummary),1);
for i = 1:height(ValidationSummary)
    if ValidationSummary.Comparison(i) == "Simulation vs Physical" && ...
            ValidationSummary.Group(i) ~= "Overall" && ...
            ~startsWith(ValidationSummary.Group(i),"Overall excluding")
        alg = ValidationSummary.Group(i);
        rates = unique(Physical.Rate(Physical.Algorithm == alg));
        if numel(rates) == 1
            k = modeKForRate(cfg.PSMissionName,rates,ModeMap);
            [mode,~,~,~] = mapConditions(cfg.PSMissionName,rates,k,ModeMap);
            ValidationSummary.Mode(i) = mode;
        end
    end
end
ValidationSummary = movevars(ValidationSummary,"Mode",'After',"Group");
end

function rows = validationRowsFromPairs(Pairs,comparison,hasReviewFlag,cfg)
rows = emptyValidationTable();
groups = ["Overall";unique(Pairs.Algorithm,'stable')];
if hasReviewFlag
    groups = [groups(1);"Overall excluding review-flagged matches";groups(2:end)];
end

metricNames = ["Max Robot Steps","Total Team Steps"];
simFields = ["SimulationMax","SimulationTotal"];
obsFields = ["ObservedMax","ObservedTotal"];

for groupIndex = 1:numel(groups)
    group = groups(groupIndex);
    if group == "Overall"
        subset = Pairs;
    elseif group == "Overall excluding review-flagged matches"
        subset = Pairs(Pairs.ReviewFlag ~= "yes",:);
    else
        subset = Pairs(Pairs.Algorithm == group,:);
    end

    for metricIndex = 1:2
        if metricIndex == 1
            margins = [1 3 5];
        else
            margins = [4 12 20];
        end
        stats = validationStats(subset.(simFields(metricIndex)), ...
            subset.(obsFields(metricIndex)),margins,cfg.Alpha);
        newRow = table(string(comparison),group,metricNames(metricIndex),stats.N, ...
            stats.SimulationMean,stats.SimulationSD,stats.ObservedMean,stats.ObservedSD, ...
            stats.MeanDifference,stats.DifferenceSD,stats.CI95Low,stats.CI95High, ...
            stats.PairedTTestP,stats.WilcoxonP,stats.Margin1,stats.TOSTP1, ...
            stats.Margin2,stats.TOSTP2,stats.Margin3,stats.TOSTP3,stats.MAE, ...
            stats.RMSE,stats.PearsonR,stats.CCC,stats.LoALow,stats.LoAHigh, ...
            'VariableNames',rows.Properties.VariableNames);
        rows = [rows;newRow]; %#ok<AGROW>
    end
end
end

function T = emptyValidationTable()
T = table('Size',[0 26], ...
    'VariableTypes',{'string','string','string','double','double','double', ...
    'double','double','double','double','double','double','double','double', ...
    'double','double','double','double','double','double','double','double', ...
    'double','double','double','double'}, ...
    'VariableNames',{'Comparison','Group','Metric','NPairs','SimulationMean', ...
    'SimulationSD','ObservedMean','ObservedSD','MeanDifference','DifferenceSD', ...
    'CI95Low','CI95High','PairedTTestP','WilcoxonP','TOSTMargin1','TOSTP1', ...
    'TOSTMargin2','TOSTP2','TOSTMargin3','TOSTP3','MAE','RMSE','PearsonR', ...
    'CCC','BlandAltmanLow','BlandAltmanHigh'});
end

function stats = validationStats(simulation,observed,margins,alpha)
valid = isfinite(simulation) & isfinite(observed);
simulation = double(simulation(valid));
observed = double(observed(valid));
difference = observed-simulation;
n = numel(difference);
stats = struct();
stats.N = n;
stats.SimulationMean = mean(simulation,'omitnan');
stats.SimulationSD = std(simulation,0,'omitnan');
stats.ObservedMean = mean(observed,'omitnan');
stats.ObservedSD = std(observed,0,'omitnan');
stats.MeanDifference = mean(difference,'omitnan');
stats.DifferenceSD = std(difference,0,'omitnan');
if n >= 2
    [~,stats.PairedTTestP,ci] = ttest(observed,simulation,'Alpha',alpha);
    stats.CI95Low = ci(1);
    stats.CI95High = ci(2);
    try
        stats.WilcoxonP = signrank(observed,simulation,'method','approximate');
    catch
        stats.WilcoxonP = NaN;
    end
else
    stats.PairedTTestP = NaN;
    stats.CI95Low = NaN;
    stats.CI95High = NaN;
    stats.WilcoxonP = NaN;
end
stats.Margin1 = margins(1);
stats.Margin2 = margins(2);
stats.Margin3 = margins(3);
stats.TOSTP1 = tostPValue(difference,margins(1));
stats.TOSTP2 = tostPValue(difference,margins(2));
stats.TOSTP3 = tostPValue(difference,margins(3));
stats.MAE = mean(abs(difference),'omitnan');
stats.RMSE = sqrt(mean(difference.^2,'omitnan'));
if n >= 2
    stats.PearsonR = corr(simulation,observed,'Rows','complete');
    stats.CCC = concordanceCorrelation(simulation,observed);
else
    stats.PearsonR = NaN;
    stats.CCC = NaN;
end
stats.LoALow = stats.MeanDifference-1.96*stats.DifferenceSD;
stats.LoAHigh = stats.MeanDifference+1.96*stats.DifferenceSD;
end

function p = tostPValue(difference,margin)
difference = double(difference(isfinite(difference)));
n = numel(difference);
if n < 2
    p = NaN;
    return;
end
mu = mean(difference);
se = std(difference,0)/sqrt(n);
if se == 0
    p = double(abs(mu) >= margin);
    return;
end
pLower = 1-tcdf((mu+margin)/se,n-1);
pUpper = tcdf((mu-margin)/se,n-1);
p = max(pLower,pUpper);
end

function value = concordanceCorrelation(x,y)
x = double(x); y = double(y);
mx = mean(x); my = mean(y);
vx = var(x,0); vy = var(y,0);
covxy = cov(x,y);
value = 2*covxy(1,2)/(vx+vy+(mx-my)^2);
end

%% Manuscript claim-ready values
function ClaimSummary = buildClaimSummary(ConditionSummary,PairedChangeSummary,FailureSummary, ...
    HILFeasibility,ValidationSummary,AllTrials,cfg)
ClaimSummary = emptyClaimTable();

% Performance degradation in probabilistic search.
psR4 = algorithmValues(PairedChangeSummary,cfg.PSMissionName,"Simulation", ...
    "R4","Max Robot Steps",cfg.AlgorithmOrder);
psR7 = algorithmValues(PairedChangeSummary,cfg.PSMissionName,"Simulation", ...
    "R7","Max Robot Steps",cfg.AlgorithmOrder);
ClaimSummary = addClaim(ClaimSummary,"PS_MEAN_DEG_R4","Mission Performance", ...
    cfg.PSMissionName,"All algorithms","R4","Max Robot Steps",mean(psR4,'omitnan'),NaN, ...
    "% degradation","Mean of the six algorithm-level paired degradation values at R4.", ...
    "PairedChangeSummary");
ClaimSummary = addClaim(ClaimSummary,"PS_MEAN_DEG_R7","Mission Performance", ...
    cfg.PSMissionName,"All algorithms","R7","Max Robot Steps",mean(psR7,'omitnan'),NaN, ...
    "% degradation","Mean of the six algorithm-level paired degradation values at R7.", ...
    "PairedChangeSummary");
ClaimSummary = addClaim(ClaimSummary,"PS_ADDITIONAL_DEG_R4_TO_R7","Mission Performance", ...
    cfg.PSMissionName,"All algorithms","R4 to R7","Max Robot Steps", ...
    mean(psR7,'omitnan')-mean(psR4,'omitnan'),NaN,"percentage points", ...
    "Additional mean degradation below R4.","PairedChangeSummary");
ClaimSummary = addClaim(ClaimSummary,"PS_DGA_DEG_R4","Mission Performance", ...
    cfg.PSMissionName,"DGA","R4","Max Robot Steps", ...
    pairedValue(PairedChangeSummary,cfg.PSMissionName,"Simulation","DGA","R4","Max Robot Steps"), ...
    NaN,"% degradation","DGA paired max-step degradation at R4.","PairedChangeSummary");
ClaimSummary = addClaim(ClaimSummary,"PS_HIPC_DEG_R4_R7","Mission Performance", ...
    cfg.PSMissionName,"HIPC","R4 to R7","Max Robot Steps", ...
    pairedValue(PairedChangeSummary,cfg.PSMissionName,"Simulation","HIPC","R4","Max Robot Steps"), ...
    pairedValue(PairedChangeSummary,cfg.PSMissionName,"Simulation","HIPC","R7","Max Robot Steps"), ...
    "% degradation","HIPC degradation at R4 and R7.","PairedChangeSummary");

% Collaborative stability and behavior.
ClaimSummary = addClaim(ClaimSummary,"CV_CBAA_R4","Mission Performance", ...
    cfg.CVMissionName,"CBAA","R4","Max Robot Steps", ...
    pairedValue(PairedChangeSummary,cfg.CVMissionName,"Simulation","CBAA","R4","Max Robot Steps"), ...
    NaN,"% change","CBAA paired max-step change at R4.","PairedChangeSummary");
for algorithm = ["ACBBA","PI"]
    values = algorithmModeRange(PairedChangeSummary,cfg.CVMissionName,"Simulation", ...
        algorithm,["R1","R2","R3","R4"],"Max Robot Steps");
    ClaimSummary = addClaim(ClaimSummary,"CV_"+algorithm+"_MAX_R1_R4", ...
        "Mission Performance",cfg.CVMissionName,algorithm,"R1 to R4", ...
        "Max Robot Steps",max(abs(values),[],'omitnan'),NaN,"maximum absolute % change", ...
        "Largest absolute paired change across R1--R4.","PairedChangeSummary");
end

% Simulation compute claims use paired combined time per call; positive changes
% are overhead and negative changes are savings.
for mode = ["R3","R2","R1"]
    ClaimSummary = addClaim(ClaimSummary,"PS_CBAA_COMPUTE_"+mode, ...
        "Computational Overhead",cfg.PSMissionName,"CBAA",mode, ...
        "Combined Time Per Call",pairedValue(PairedChangeSummary,cfg.PSMissionName, ...
        "Simulation","CBAA",mode,"Combined Time Per Call"),NaN,"% change", ...
        "Paired combined per-call computation change from R0.","PairedChangeSummary");
end

dmSavings = -algorithmModeRange(PairedChangeSummary,cfg.PSMissionName,"Simulation", ...
    "DMCHBA",["R3","R4"],"Combined Time Per Call");
ClaimSummary = addClaim(ClaimSummary,"PS_DMCHBA_SAVING_R3_R4", ...
    "Computational Overhead",cfg.PSMissionName,"DMCHBA","R3 to R4", ...
    "Combined Time Per Call",min(dmSavings,[],'omitnan'),max(dmSavings,[],'omitnan'), ...
    "% compute saving","Minimum and maximum paired saving across R3--R4.", ...
    "PairedChangeSummary");

dgaSavings = -algorithmModeRange(PairedChangeSummary,cfg.PSMissionName,"Simulation", ...
    "DGA",["R3","R4","R5"],"Combined Time Per Call");
ClaimSummary = addClaim(ClaimSummary,"PS_DGA_SAVING_R3_R5", ...
    "Computational Overhead",cfg.PSMissionName,"DGA","R3 to R5", ...
    "Combined Time Per Call",min(dgaSavings,[],'omitnan'),max(dgaSavings,[],'omitnan'), ...
    "% compute saving","Minimum and maximum paired saving across R3--R5.", ...
    "PairedChangeSummary");

% Collaborative compute reduction across all filtered standard modes.
cvMask = PairedChangeSummary.Mission == cfg.CVMissionName & ...
    PairedChangeSummary.Platform == "Simulation" & ...
    PairedChangeSummary.Metric == "Combined Time Per Call" & ...
    PairedChangeSummary.Mode ~= "R0";
cvSavings = -PairedChangeSummary.MeanPctChange(cvMask);
ClaimSummary = addClaim(ClaimSummary,"CV_FILTERED_SAVING_RANGE", ...
    "Computational Overhead",cfg.CVMissionName,"All algorithms","R1 to R6", ...
    "Combined Time Per Call",min(cvSavings,[],'omitnan'),max(cvSavings,[],'omitnan'), ...
    "% compute saving","Range across all algorithms and standard filtered modes.", ...
    "PairedChangeSummary");

% HIL filter-share and timing-limit claims.
cbaaMask = ConditionSummary.Mission == cfg.PSMissionName & ...
    ConditionSummary.Platform == "RP2040 HIL" & ConditionSummary.Algorithm == "CBAA" & ...
    ~ismissing(ConditionSummary.Mode) & ConditionSummary.Mode ~= "R0";
cbaaShares = ConditionSummary.FilterShareAggregatePct(cbaaMask);
ClaimSummary = addClaim(ClaimSummary,"PS_HIL_CBAA_FILTER_SHARE", ...
    "Computational Overhead",cfg.PSMissionName,"CBAA","Filtered modes", ...
    "Filter Share",min(cbaaShares,[],'omitnan'),max(cbaaShares,[],'omitnan'), ...
    "% combined compute","Aggregate filter-share range across observed filtered HIL conditions.", ...
    "ConditionSummary");
for algorithm = ["HIPC","DMCHBA","DGA"]
    capCount = sum(HILFeasibility.Mission == cfg.PSMissionName & ...
        HILFeasibility.Algorithm == algorithm & HILFeasibility.CapStopped);
    feasibleCount = sum(HILFeasibility.Mission == cfg.PSMissionName & ...
        HILFeasibility.Algorithm == algorithm & HILFeasibility.Feasible);
    ClaimSummary = addClaim(ClaimSummary,"PS_HIL_"+algorithm+"_CAP_FEASIBLE", ...
        "Computational Overhead",cfg.PSMissionName,algorithm,"All tested modes", ...
        "HIL Feasibility",capCount,feasibleCount,"conditions", ...
        "Number of cap-stopped conditions and hardware-feasible conditions.", ...
        "HILFeasibility");
end

% K=1 reliability boundary.
psK1 = FailureSummary.Mission == cfg.PSMissionName & ...
    FailureSummary.Platform == "Simulation" & FailureSummary.IsSpecialK1;
psAll = FailureSummary.Mission == cfg.PSMissionName & FailureSummary.Platform == "Simulation";
ClaimSummary = addClaim(ClaimSummary,"PS_K1_FAILURES", ...
    "Reliability",cfg.PSMissionName,"All algorithms","K=1", ...
    "Failed Trials",sum(FailureSummary.FailedTrials(psK1),'omitnan'), ...
    sum(FailureSummary.FailedTrials(psAll),'omitnan'),"failed trials", ...
    "K=1 failures and total probabilistic-simulation failures.","FailureSummary");

% Invocation-weighted mean filter time by mission and platform, excluding K=1.
for mission = [cfg.PSMissionName,cfg.CVMissionName]
    for platform = ["Simulation","RP2040 HIL"]
        mask = AllTrials.Mission == mission & AllTrials.Platform == platform & ...
            AllTrials.Status == "completed" & AllTrials.AnalysisValid & ...
            ~AllTrials.IsSpecialK1;
        filterTotal = sum(AllTrials.FilterTotalMs(mask),'omitnan');
        filterCalls = sum(AllTrials.FilterCalls(mask),'omitnan');
        weightedMean = filterTotal/filterCalls;
        ClaimSummary = addClaim(ClaimSummary, ...
            replace(upper(extractBefore(mission," "))+"_"+replace(platform," ","_")+"_FILTER_MEAN","-","_"), ...
            "Computational Overhead",mission,"All algorithms","Standard modes", ...
            "Filter Time Per Invocation",weightedMean,NaN,"ms", ...
            "Invocation-weighted aggregate filter execution time.","AllTrials");
    end
end

% Validation claims.
validationClaims = ValidationSummary(ValidationSummary.Group == "Overall" & ...
    ValidationSummary.Metric == "Max Robot Steps",:);
for i = 1:height(validationClaims)
    ClaimSummary = addClaim(ClaimSummary, ...
        "VALIDATION_"+replace(upper(validationClaims.Comparison(i))," ","_"), ...
        "Validation",cfg.PSMissionName,"All algorithms","Matched trials", ...
        "Max Robot Steps",validationClaims.MeanDifference(i),validationClaims.CCC(i), ...
        "steps / CCC","Mean paired difference and concordance correlation.", ...
        "ValidationSummary");
end
end

function T = emptyClaimTable()
T = table('Size',[0 11], ...
    'VariableTypes',{'string','string','string','string','string','string', ...
    'double','double','string','string','string'}, ...
    'VariableNames',{'ClaimID','ResultsSubsection','Mission','Algorithm','ModeRange', ...
    'Metric','Value','SecondaryValue','Units','Description','SourceTable'});
end

function T = addClaim(T,id,section,mission,algorithm,modeRange,metric,value,secondary,units,description,source)
row = table(string(id),string(section),string(mission),string(algorithm), ...
    string(modeRange),string(metric),double(value),double(secondary),string(units), ...
    string(description),string(source),'VariableNames',T.Properties.VariableNames);
T = [T;row];
end

function value = pairedValue(T,mission,platform,algorithm,mode,metric)
mask = T.Mission == mission & T.Platform == platform & T.Algorithm == algorithm & ...
    T.Mode == mode & T.Metric == metric;
if sum(mask) == 1
    value = T.MeanPctChange(mask);
else
    value = NaN;
end
end

function values = algorithmValues(T,mission,platform,mode,metric,algorithmOrder)
values = NaN(numel(algorithmOrder),1);
for i = 1:numel(algorithmOrder)
    values(i) = pairedValue(T,mission,platform,algorithmOrder(i),mode,metric);
end
end

function values = algorithmModeRange(T,mission,platform,algorithm,modes,metric)
values = NaN(numel(modes),1);
for i = 1:numel(modes)
    values(i) = pairedValue(T,mission,platform,algorithm,modes(i),metric);
end
end

%% Environmental sensitivity: multivariate permutation tests
function [Omnibus,Pairwise,Levelwise,SignificantOnly,CurveSummary] = ...
    runSensitivityAnalysis(filePath,cfg)

sourceInfo = dir(filePath);
cacheValid = false;
if cfg.ReuseSensitivityCache && isfile(cfg.SensitivityCache)
    cache = load(cfg.SensitivityCache);
    if isfield(cache,'CacheInfo') && isfield(cache.CacheInfo,'Complete') && ...
            cache.CacheInfo.Complete && ...
            cache.CacheInfo.SourceBytes == sourceInfo.bytes && ...
            cache.CacheInfo.NumPermutations == cfg.NumPermutations && ...
            cache.CacheInfo.RandomSeed == cfg.RandomSeed
        Omnibus = cache.Omnibus;
        Pairwise = cache.Pairwise;
        Levelwise = cache.Levelwise;
        SignificantOnly = cache.SignificantOnly;
        CurveSummary = cache.CurveSummary;
        fprintf('Loaded sensitivity results from cache: %s\n',cfg.SensitivityCache);
        cacheValid = true;
    end
end
if cacheValid
    return;
end

T = readtable(filePath,'VariableNamingRule','preserve');
T.environment = string(T.environment);
T.algorithm = string(T.algorithm);
T.comm_model = string(T.comm_model);

rates = [0.75 0.50 0.25 0.10 0.05];
modes = ["R1" "R2" "R3" "R4" "R5"];

families(1) = struct( ...
    'Name',"Grid Size", ...
    'EnvironmentIDs',["scale_g14_r4","scale_g19_r4","scale_g28_r4"], ...
    'EnvironmentLabels',["14x14","19x19","28x28"], ...
    'BaselineID',"scale_g19_r4", ...
    'BaselineLabel',"19x19", ...
    'Paired',false);
families(2) = struct( ...
    'Name',"Team Size", ...
    'EnvironmentIDs',["scale_g19_r2","scale_g19_r4","scale_g19_r8"], ...
    'EnvironmentLabels',["2 robots","4 robots","8 robots"], ...
    'BaselineID',"scale_g19_r4", ...
    'BaselineLabel',"4 robots", ...
    'Paired',true);
families(3) = struct( ...
    'Name',"Communication", ...
    'EnvironmentIDs',["scale_g19_r4","comm_g19_r4_bernoulli_drop025", ...
        "comm_g19_r4_ge_drop025","comm_g19_r4_rayleigh_drop025"], ...
    'EnvironmentLabels',["Ideal","Bernoulli 25%","Gilbert-Elliott 25%","Rayleigh 25%"], ...
    'BaselineID',"scale_g19_r4", ...
    'BaselineLabel',"Ideal", ...
    'Paired',true);

metrics = ["Max Robot Steps","Total Team Steps"];
Omnibus = emptySensitivityOmnibus();
Pairwise = emptySensitivityPairwise();
Levelwise = emptySensitivityLevelwise();
CurveSummary = emptySensitivityCurveSummary();

testCounter = 0;
for familyIndex = 1:numel(families)
    family = families(familyIndex);
    for metricIndex = 1:numel(metrics)
        metric = metrics(metricIndex);
        familyRows = emptySensitivityOmnibus();

        for algorithm = cfg.AlgorithmOrder
            testCounter = testCounter + 1;
            [curves,trialIDs,curveRows] = extractSensitivityCurves( ...
                T,algorithm,family,metric,rates,modes,cfg.PSMissionName);
            CurveSummary = [CurveSummary;curveRows]; %#ok<AGROW>

            seed = cfg.RandomSeed + 10000*familyIndex + 1000*metricIndex + testCounter;
            if family.Paired
                [X,matchedIDs] = matchedCurveArray(curves,trialIDs);
                if size(X,1) < 2
                    statistic = NaN; pValue = NaN; nComplete = size(X,1);
                else
                    [statistic,pValue] = pairedGlobalPermutationTest( ...
                        X,cfg.NumPermutations,seed);
                    nComplete = size(X,1);
                end
                groupSizes = repmat(nComplete,1,numel(family.EnvironmentIDs));
            else
                [statistic,pValue] = independentGlobalPermutationTest( ...
                    curves,cfg.NumPermutations,seed);
                groupSizes = cellfun(@(x)size(x,1),curves);
                nComplete = sum(groupSizes);
                matchedIDs = [];
            end

            if family.Paired
                designLabel = "paired";
            else
                designLabel = "independent";
            end
            row = table(cfg.PSMissionName,family.Name,metric,algorithm, ...
                designLabel,strjoin(family.EnvironmentLabels," | "), ...
                family.BaselineLabel,strjoin(string(groupSizes)," | "), ...
                nComplete,cfg.NumPermutations,statistic,pValue,NaN,false, ...
                'VariableNames',familyRows.Properties.VariableNames);
            familyRows = [familyRows;row]; %#ok<AGROW>

            fprintf('  %s | %s | %s: global permutation p = %.6g\n', ...
                family.Name,metric,algorithm,pValue);
        end

        familyRows.HolmP = holmAdjust(familyRows.PValue);
        familyRows.Significant = familyRows.HolmP < cfg.Alpha;
        Omnibus = [Omnibus;familyRows]; %#ok<AGROW>

        % Follow-up tests only for Holm-significant global interactions.
        significantAlgorithms = familyRows.Algorithm(familyRows.Significant);
        for algorithm = significantAlgorithms'
            [curves,trialIDs,~] = extractSensitivityCurves( ...
                T,algorithm,family,metric,rates,modes,cfg.PSMissionName);
            baselineIndex = find(family.EnvironmentIDs == family.BaselineID,1);
            alternativeIndices = setdiff(1:numel(family.EnvironmentIDs),baselineIndex);
            algorithmPairRows = emptySensitivityPairwise();

            for altCounter = 1:numel(alternativeIndices)
                altIndex = alternativeIndices(altCounter);
                seed = cfg.RandomSeed + 500000 + 10000*familyIndex + ...
                    1000*metricIndex + 100*find(cfg.AlgorithmOrder==algorithm,1) + altCounter;

                if family.Paired
                    [baseCurve,altCurve,nPair] = matchCurvePair( ...
                        curves{baselineIndex},trialIDs{baselineIndex}, ...
                        curves{altIndex},trialIDs{altIndex});
                    [statistic,pValue] = pairedCurvePermutationTest( ...
                        baseCurve,altCurve,cfg.NumPermutations,seed);
                    nBaseline = nPair; nAlternative = nPair;
                else
                    baseCurve = curves{baselineIndex};
                    altCurve = curves{altIndex};
                    [statistic,pValue] = independentCurvePermutationTest( ...
                        baseCurve,altCurve,cfg.NumPermutations,seed);
                    nBaseline = size(baseCurve,1);
                    nAlternative = size(altCurve,1);
                    nPair = NaN;
                end

                if family.Paired
                    designLabel = "paired";
                else
                    designLabel = "independent";
                end
                pairRow = table(cfg.PSMissionName,family.Name,metric,algorithm, ...
                    family.BaselineLabel,family.EnvironmentLabels(altIndex), ...
                    designLabel,nBaseline,nAlternative,nPair, ...
                    cfg.NumPermutations,statistic,pValue,NaN,false, ...
                    'VariableNames',algorithmPairRows.Properties.VariableNames);
                algorithmPairRows = [algorithmPairRows;pairRow]; %#ok<AGROW>
            end

            algorithmPairRows.HolmP = holmAdjust(algorithmPairRows.PValue);
            algorithmPairRows.Significant = algorithmPairRows.HolmP < cfg.Alpha;
            Pairwise = [Pairwise;algorithmPairRows]; %#ok<AGROW>

            % Level-wise follow-up only for significant pairwise curves.
            for pairIndex = find(algorithmPairRows.Significant)'
                altLabel = algorithmPairRows.Alternative(pairIndex);
                altIndex = find(family.EnvironmentLabels == altLabel,1);
                if family.Paired
                    [baseCurve,altCurve,~] = matchCurvePair( ...
                        curves{baselineIndex},trialIDs{baselineIndex}, ...
                        curves{altIndex},trialIDs{altIndex});
                else
                    baseCurve = curves{baselineIndex};
                    altCurve = curves{altIndex};
                end

                levelRowsForPair = emptySensitivityLevelwise();
                for levelIndex = 1:numel(rates)
                    seed = cfg.RandomSeed + 900000 + 10000*familyIndex + ...
                        1000*metricIndex + 100*find(cfg.AlgorithmOrder==algorithm,1) + ...
                        10*altIndex + levelIndex;
                    if family.Paired
                        [statistic,pValue] = pairedLevelPermutationTest( ...
                            baseCurve(:,levelIndex),altCurve(:,levelIndex), ...
                            cfg.NumPermutations,seed);
                        nBaseline = size(baseCurve,1);
                        nAlternative = nBaseline;
                    else
                        [statistic,pValue] = independentLevelPermutationTest( ...
                            baseCurve(:,levelIndex),altCurve(:,levelIndex), ...
                            cfg.NumPermutations,seed);
                        nBaseline = size(baseCurve,1);
                        nAlternative = size(altCurve,1);
                    end
                    meanBase = mean(baseCurve(:,levelIndex),'omitnan');
                    meanAlt = mean(altCurve(:,levelIndex),'omitnan');
                    levelRow = table(cfg.PSMissionName,family.Name,metric,algorithm, ...
                        family.BaselineLabel,altLabel,modes(levelIndex),rates(levelIndex), ...
                        nBaseline,nAlternative,meanBase,meanAlt,meanAlt-meanBase, ...
                        statistic,pValue,NaN,false, ...
                        'VariableNames',levelRowsForPair.Properties.VariableNames);
                    levelRowsForPair = [levelRowsForPair;levelRow]; %#ok<AGROW>
                end
                levelRowsForPair.HolmP = holmAdjust(levelRowsForPair.PValue);
                levelRowsForPair.Significant = levelRowsForPair.HolmP < cfg.Alpha;
                Levelwise = [Levelwise;levelRowsForPair]; %#ok<AGROW>
            end
        end
    end
end

SignificantOnly = buildSignificantSensitivityTable(Omnibus,Pairwise,Levelwise);
CacheInfo = struct('Complete',true,'SourceBytes',sourceInfo.bytes, ...
    'NumPermutations',cfg.NumPermutations,'RandomSeed',cfg.RandomSeed, ...
    'Created',datetime('now'));
save(cfg.SensitivityCache,'Omnibus','Pairwise','Levelwise','SignificantOnly', ...
    'CurveSummary','CacheInfo','-v7.3');
end

function [curves,trialIDs,CurveSummary] = extractSensitivityCurves(T,algorithm,family,metric,rates,modes,mission)
curves = cell(numel(family.EnvironmentIDs),1);
trialIDs = cell(numel(family.EnvironmentIDs),1);
CurveSummary = emptySensitivityCurveSummary();

for environmentIndex = 1:numel(family.EnvironmentIDs)
    envID = family.EnvironmentIDs(environmentIndex);
    subset = T(T.algorithm == algorithm & T.environment == envID,:);
    ids = unique(subset.trial_id,'stable');
    envCurves = NaN(0,numel(rates));
    completeIDs = NaN(0,1);

    for trialID = ids'
        trial = subset(subset.trial_id == trialID,:);
        if numel(unique(trial.top_k_rate)) < numel(rates)+1 || ...
                ~any(abs(trial.top_k_rate-1.0)<1e-8)
            continue;
        end
        curve = NaN(1,numel(rates));
        if metric == "Max Robot Steps"
            baselineRows = trial(abs(trial.top_k_rate-1.0)<1e-8,:);
            baseline = baselineRows.max_steps_any_robot(1);
            if ~isfinite(baseline) || baseline == 0
                continue;
            end
            for rateIndex = 1:numel(rates)
                rows = trial(abs(trial.top_k_rate-rates(rateIndex))<1e-8,:);
                if isempty(rows) || ~isfinite(rows.max_steps_any_robot(1))
                    break;
                end
                curve(rateIndex) = 100 * ...
                    (rows.max_steps_any_robot(1)-baseline)/baseline;
            end
        else
            for rateIndex = 1:numel(rates)
                rows = trial(abs(trial.top_k_rate-rates(rateIndex))<1e-8,:);
                if isempty(rows) || ~isfinite(rows.total_step_degradation_pct(1))
                    break;
                end
                curve(rateIndex) = rows.total_step_degradation_pct(1);
            end
        end
        if all(isfinite(curve))
            envCurves(end+1,:) = curve; %#ok<AGROW>
            completeIDs(end+1,1) = trialID; %#ok<AGROW>
        end
    end

    curves{environmentIndex} = envCurves;
    trialIDs{environmentIndex} = completeIDs;
    for rateIndex = 1:numel(rates)
        values = envCurves(:,rateIndex);
        row = table(string(mission),family.Name,metric,algorithm, ...
            family.EnvironmentLabels(environmentIndex), ...
            family.EnvironmentIDs(environmentIndex) == family.BaselineID, ...
            modes(rateIndex),rates(rateIndex),numel(values),mean(values,'omitnan'), ...
            std(values,0,'omitnan'),median(values,'omitnan'), ...
            'VariableNames',CurveSummary.Properties.VariableNames);
        CurveSummary = [CurveSummary;row]; %#ok<AGROW>
    end
end
end

function [X,matchedIDs] = matchedCurveArray(curves,trialIDs)
matchedIDs = trialIDs{1};
for i = 2:numel(trialIDs)
    matchedIDs = intersect(matchedIDs,trialIDs{i},'stable');
end
n = numel(matchedIDs);
g = numel(curves);
p = size(curves{1},2);
X = NaN(n,g,p);
for environmentIndex = 1:g
    [~,loc] = ismember(matchedIDs,trialIDs{environmentIndex});
    X(:,environmentIndex,:) = reshape(curves{environmentIndex}(loc,:),[n 1 p]);
end
end

function [baseCurve,altCurve,n] = matchCurvePair(base,baseIDs,alt,altIDs)
ids = intersect(baseIDs,altIDs,'stable');
[~,baseLoc] = ismember(ids,baseIDs);
[~,altLoc] = ismember(ids,altIDs);
baseCurve = base(baseLoc,:);
altCurve = alt(altLoc,:);
n = numel(ids);
end

function [statistic,pValue] = independentGlobalPermutationTest(curves,B,seed)
X = vertcat(curves{:});
groupSizes = cellfun(@(x)size(x,1),curves);
labels = repelem((1:numel(curves))',groupSizes(:));
statistic = independentGlobalStatistic(X,labels);
rng(seed,'twister');
exceed = 0;
N = size(X,1);
for b = 1:B
    permuted = labels(randperm(N));
    exceed = exceed + (independentGlobalStatistic(X,permuted) >= statistic-1e-12);
end
pValue = (exceed+1)/(B+1);
end

function value = independentGlobalStatistic(X,labels)
groups = unique(labels,'stable');
N = size(X,1);
g = numel(groups);
grandMean = mean(X,1);
ssBetween = 0;
ssWithin = 0;
for group = groups'
    rows = X(labels==group,:);
    groupMean = mean(rows,1);
    ssBetween = ssBetween + size(rows,1)*sum((groupMean-grandMean).^2);
    ssWithin = ssWithin + sum((rows-groupMean).^2,'all');
end
value = (ssBetween/(g-1))/(ssWithin/(N-g));
end

function [statistic,pValue] = pairedGlobalPermutationTest(X,B,seed)
statistic = pairedGlobalStatistic(X);
[N,G,P] = size(X);
centered = X-mean(X,2);
permutationList = perms(1:G);
nPerm = size(permutationList,1);
rng(seed,'twister');
exceed = 0;
subjectIndex = (1:N)';
for b = 1:B
    choices = randi(nPerm,N,1);
    environmentMeans = zeros(G,P);
    for targetEnvironment = 1:G
        sourceEnvironment = permutationList(choices,targetEnvironment);
        for feature = 1:P
            featureMatrix = centered(:,:,feature);
            linearIndex = sub2ind([N,G],subjectIndex,sourceEnvironment);
            environmentMeans(targetEnvironment,feature) = mean(featureMatrix(linearIndex));
        end
    end
    permutedStatistic = sum(environmentMeans.^2,'all');
    exceed = exceed + (permutedStatistic >= statistic-1e-12);
end
pValue = (exceed+1)/(B+1);
end

function value = pairedGlobalStatistic(X)
centered = X-mean(X,2);
environmentMeans = squeeze(mean(centered,1));
value = sum(environmentMeans.^2,'all');
end

function [statistic,pValue] = pairedCurvePermutationTest(base,alt,B,seed)
difference = alt-base;
statistic = sum(mean(difference,1).^2);
rng(seed,'twister');
exceed = 0;
n = size(difference,1);
batchSize = 5000;
for first = 1:batchSize:B
    currentBatch = min(batchSize,B-first+1);
    signs = 2*(rand(currentBatch,n)>0.5)-1;
    permutedMeans = signs*difference/n;
    permutedStats = sum(permutedMeans.^2,2);
    exceed = exceed + sum(permutedStats >= statistic-1e-12);
end
pValue = (exceed+1)/(B+1);
end

function [statistic,pValue] = independentCurvePermutationTest(base,alt,B,seed)
X = [base;alt];
nBase = size(base,1);
N = size(X,1);
statistic = sum((mean(alt,1)-mean(base,1)).^2);
rng(seed,'twister');
exceed = 0;
for b = 1:B
    order = randperm(N);
    permBase = X(order(1:nBase),:);
    permAlt = X(order(nBase+1:end),:);
    permutedStatistic = sum((mean(permAlt,1)-mean(permBase,1)).^2);
    exceed = exceed + (permutedStatistic >= statistic-1e-12);
end
pValue = (exceed+1)/(B+1);
end

function [statistic,pValue] = pairedLevelPermutationTest(base,alt,B,seed)
difference = alt-base;
statistic = abs(mean(difference));
rng(seed,'twister');
exceed = 0;
n = numel(difference);
batchSize = 10000;
for first = 1:batchSize:B
    currentBatch = min(batchSize,B-first+1);
    signs = 2*(rand(currentBatch,n)>0.5)-1;
    permuted = abs(signs*difference/n);
    exceed = exceed + sum(permuted >= statistic-1e-12);
end
pValue = (exceed+1)/(B+1);
end

function [statistic,pValue] = independentLevelPermutationTest(base,alt,B,seed)
base = base(isfinite(base)); alt = alt(isfinite(alt));
X = [base;alt];
nBase = numel(base);
N = numel(X);
statistic = abs(mean(alt)-mean(base));
rng(seed,'twister');
exceed = 0;
for b = 1:B
    order = randperm(N);
    permuted = abs(mean(X(order(nBase+1:end)))-mean(X(order(1:nBase))));
    exceed = exceed + (permuted >= statistic-1e-12);
end
pValue = (exceed+1)/(B+1);
end

function T = emptySensitivityOmnibus()
T = table('Size',[0 14], ...
    'VariableTypes',{'string','string','string','string','string','string','string', ...
    'string','double','double','double','double','double','logical'}, ...
    'VariableNames',{'Mission','Family','Metric','Algorithm','PairedDesign', ...
    'Environments','Baseline','GroupSizes','NCompleteCurves','Permutations', ...
    'Statistic','PValue','HolmP','Significant'});
end

function T = emptySensitivityPairwise()
T = table('Size',[0 15], ...
    'VariableTypes',{'string','string','string','string','string','string','string', ...
    'double','double','double','double','double','double','double','logical'}, ...
    'VariableNames',{'Mission','Family','Metric','Algorithm','Baseline','Alternative', ...
    'PairedDesign','NBaseline','NAlternative','NMatched','Permutations','Statistic', ...
    'PValue','HolmP','Significant'});
end

function T = emptySensitivityLevelwise()
T = table('Size',[0 17], ...
    'VariableTypes',{'string','string','string','string','string','string','string', ...
    'double','double','double','double','double','double','double','double','double','logical'}, ...
    'VariableNames',{'Mission','Family','Metric','Algorithm','Baseline','Alternative', ...
    'Mode','Rate','NBaseline','NAlternative','MeanBaselinePct','MeanAlternativePct', ...
    'MeanDifferencePP','Statistic','PValue','HolmP','Significant'});
end

function T = emptySensitivityCurveSummary()
T = table('Size',[0 12], ...
    'VariableTypes',{'string','string','string','string','string','logical','string', ...
    'double','double','double','double','double'}, ...
    'VariableNames',{'Mission','Family','Metric','Algorithm','Environment','IsBaseline', ...
    'Mode','Rate','NCompleteCurves','MeanDegradationPct','SDDegradationPct', ...
    'MedianDegradationPct'});
end

function SignificantOnly = buildSignificantSensitivityTable(Omnibus,Pairwise,Levelwise)
SignificantOnly = table('Size',[0 12], ...
    'VariableTypes',{'string','string','string','string','string','string','string', ...
    'double','double','double','double','string'}, ...
    'VariableNames',{'ResultLevel','Mission','Family','Metric','Algorithm','Comparison', ...
    'Mode','EffectPP','Statistic','RawP','HolmP','Interpretation'});

for i = find(Omnibus.Significant)'
    row = table("Omnibus",Omnibus.Mission(i),Omnibus.Family(i),Omnibus.Metric(i), ...
        Omnibus.Algorithm(i),"All environments","",NaN,Omnibus.Statistic(i), ...
        Omnibus.PValue(i),Omnibus.HolmP(i), ...
        "The normalized five-mode degradation curve differs across environments.", ...
        'VariableNames',SignificantOnly.Properties.VariableNames);
    SignificantOnly = [SignificantOnly;row]; %#ok<AGROW>
end
for i = find(Pairwise.Significant)'
    comparison = Pairwise.Baseline(i)+" vs "+Pairwise.Alternative(i);
    row = table("Pairwise Curve",Pairwise.Mission(i),Pairwise.Family(i),Pairwise.Metric(i), ...
        Pairwise.Algorithm(i),comparison,"",NaN,Pairwise.Statistic(i), ...
        Pairwise.PValue(i),Pairwise.HolmP(i), ...
        "The complete normalized degradation curves differ for this comparison.", ...
        'VariableNames',SignificantOnly.Properties.VariableNames);
    SignificantOnly = [SignificantOnly;row]; %#ok<AGROW>
end
for i = find(Levelwise.Significant)'
    comparison = Levelwise.Baseline(i)+" vs "+Levelwise.Alternative(i);
    row = table("Mode Follow-up",Levelwise.Mission(i),Levelwise.Family(i), ...
        Levelwise.Metric(i),Levelwise.Algorithm(i),comparison,Levelwise.Mode(i), ...
        Levelwise.MeanDifferencePP(i),Levelwise.Statistic(i),Levelwise.PValue(i), ...
        Levelwise.HolmP(i), ...
        "The normalized degradation differs at this restriction mode.", ...
        'VariableNames',SignificantOnly.Properties.VariableNames);
    SignificantOnly = [SignificantOnly;row]; %#ok<AGROW>
end
end

%% Workbook output
function writeAnalysisWorkbook(filePath,ModeMap,ConditionSummary,PairedChangeSummary, ...
    FailureSummary,HILFeasibility,ValidationSummary,ClaimSummary, ...
    SensitivityOmnibus,SensitivityPairwise,SensitivityLevelwise, ...
    SensitivitySignificantOnly,SensitivityCurveSummary,SourceAudit,AnalysisSettings)

if isfile(filePath)
    delete(filePath);
end

sheets = {
    "ModeMap", ModeMap;
    "Conditions", ConditionSummary;
    "PairedChanges", PairedChangeSummary;
    "Failures", FailureSummary;
    "HILFeasibility", HILFeasibility;
    "Validation", ValidationSummary;
    "Claims", ClaimSummary;
    "SensOmnibus", SensitivityOmnibus;
    "SensPairwise", SensitivityPairwise;
    "SensLevelwise", SensitivityLevelwise;
    "SensSignificant", SensitivitySignificantOnly;
    "SensCurves", SensitivityCurveSummary;
    "SourceAudit", SourceAudit;
    "Settings", AnalysisSettings;
    };

for i = 1:size(sheets,1)
    sheetName = sheets{i,1};
    tableValue = sheets{i,2};
    if isempty(tableValue.Properties.VariableNames)
        tableValue = table("No rows generated",'VariableNames',{'Status'});
    end
    writetable(tableValue,filePath,'Sheet',sheetName,'WriteMode','overwritesheet');
end
end
