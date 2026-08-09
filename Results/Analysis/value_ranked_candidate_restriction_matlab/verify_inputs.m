function [cfg, SourceAudit, AnalysisSettings] = verify_inputs(cfg)
%VERIFY_INPUTS Verify MATLAB capabilities, input files, and required columns.
%
% This function is called automatically by build_analysis_tables.m. It can
% also be run independently:
%   cfg = analysis_config();
%   [cfg, SourceAudit, AnalysisSettings] = verify_inputs(cfg);

fprintf('\n=== Value-ranked candidate restriction analysis preflight ===\n');
fprintf('MATLAB release: %s\n', version('-release'));
fprintf('MATLAB version: %s\n', version);

statsInstalled = license('test','Statistics_Toolbox') && ~isempty(ver('stats'));
parallelInstalled = license('test','Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));

if ~statsInstalled
    error(['Statistics and Machine Learning Toolbox is required for the paired ', ...
        'tests and validation statistics. Install/enable it before continuing.']);
end

if cfg.UseParallelIfAvailable && ~parallelInstalled
    warning(['Parallel Computing Toolbox was requested but is not installed. ', ...
        'The 99,999-permutation sensitivity tests will run serially.']);
    cfg.UseParallelIfAvailable = false;
end

% No additional toolbox is required. The multivariate permutation procedures
% are implemented locally rather than depending on a separate PERMANOVA package.
fprintf('Statistics and Machine Learning Toolbox: available\n');
if parallelInstalled
    fprintf('Parallel Computing Toolbox: available (use controlled by config)\n');
else
    fprintf('Parallel Computing Toolbox: not installed; not required\n');
end

ensureDirectory(cfg.OutputDir);
ensureDirectory(cfg.TableDir);
ensureDirectory(cfg.FigureDir);
ensureDirectory(cfg.CacheDir);

% Locate condition-level HIL files when exact configured names differ.
cfg.Files.PSHILConditions = discoverConditionFile( ...
    cfg.Files.PSHILConditions, fileparts(cfg.Files.PSHIL), "probabilistic HIL");
cfg.Files.CVHILConditions = discoverConditionFile( ...
    cfg.Files.CVHILConditions, fileparts(cfg.Files.CVHIL), "collaborative HIL");

spec = {
    "PSSimulation", cfg.Files.PSSimulation, true, ...
        ["algorithm","top_k_rate","top_k_max_cells","trial_id","trial_status", ...
         "max_steps_any_robot","total_team_steps","allocator_calls_total", ...
         "allocator_time_ms_team_total","allocator_solve_time_ms_team_total", ...
         "candidate_filter_time_ms_team_total"];
    "CVSimulation", cfg.Files.CVSimulation, true, ...
        ["algorithm","top_k_rate","top_k_max_cells","trial_id","trial_status", ...
         "total_team_steps","allocator_calls_total","allocator_time_ms_team_total", ...
         "allocator_solve_time_ms_team_total","candidate_filter_time_ms_team_total"];
    "PSHIL", cfg.Files.PSHIL, true, ...
        ["algorithm","top_k_rate","trial_id","trial_status", ...
         "max_steps_any_robot","total_team_steps","timed_allocator_call_count", ...
         "allocator_time_total_us","allocator_time_max_us", ...
         "candidate_filter_time_total_us","allocator_exclusive_time_total_us"];
    "CVHIL", cfg.Files.CVHIL, true, ...
        ["algorithm","top_k_rate","trial_id","trial_status", ...
         "total_team_steps","timed_allocator_call_count","allocator_time_total_us", ...
         "allocator_time_max_us","candidate_filter_time_total_us", ...
         "allocator_exclusive_time_total_us"];
    "CombinedHIL", cfg.Files.CombinedHIL, false, ["algorithm","top_k_rate","trial_id"];
    "PSHILConditions", cfg.Files.PSHILConditions, true, ...
        ["algorithm","top_k_rate","status","stopped_reason", ...
         "completed_trial_count","failed_trial_count","planned_trial_count", ...
         "allocator_time_max_us"];
    "CVHILConditions", cfg.Files.CVHILConditions, true, ...
        ["algorithm","top_k_rate","status","stopped_reason", ...
         "completed_trial_count","failed_trial_count","planned_trial_count", ...
         "allocator_time_max_us"];
    "Sensitivity", cfg.Files.Sensitivity, true, ...
        ["suite","environment","pair_key","grid_size","robot_count", ...
         "algorithm","top_k_rate","top_k_max_cells","comm_model","trial_id", ...
         "max_steps_any_robot","total_step_degradation_pct"];
    "Physical", cfg.Files.Physical, true, ...
        ["algorithm","source_episode","top_k_rate","top_k_max_cells", ...
         "max_steps_any_robot","total_steps"];
    };

nFiles = size(spec,1);
FileKey = strings(nFiles,1);
Path = strings(nFiles,1);
Required = false(nFiles,1);
Exists = false(nFiles,1);
Bytes = NaN(nFiles,1);
Modified = NaT(nFiles,1);
DataRows = NaN(nFiles,1);
MissingColumns = strings(nFiles,1);
Status = strings(nFiles,1);

fatalMessages = strings(0,1);

for i = 1:nFiles
    key = spec{i,1};
    filePath = string(spec{i,2});
    isRequired = spec{i,3};
    requiredColumns = string(spec{i,4});

    FileKey(i) = key;
    Path(i) = filePath;
    Required(i) = isRequired;
    Exists(i) = isfile(filePath);

    if ~Exists(i)
        Status(i) = "MISSING";
        if isRequired
            fatalMessages(end+1,1) = key + ": " + filePath; %#ok<AGROW>
        end
        continue;
    end

    info = dir(filePath);
    Bytes(i) = info.bytes;
    Modified(i) = datetime(info.datenum,'ConvertFrom','datenum');
    DataRows(i) = max(0, countFileLines(filePath) - 1);

    opts = detectImportOptions(filePath, 'VariableNamingRule','preserve');
    available = string(opts.VariableNames);
    missing = requiredColumns(~ismember(requiredColumns, available));

    % Collaborative files have used either max_robot_steps or
    % max_steps_any_robot. Accept either name.
    if key == "CVSimulation" && ...
            ~ismember("max_robot_steps", available) && ...
            ~ismember("max_steps_any_robot", available)
        missing(end+1) = "max_robot_steps OR max_steps_any_robot"; %#ok<AGROW>
    end
    if key == "CVHIL" && ...
            ~ismember("max_robot_steps", available) && ...
            ~ismember("max_steps_any_robot", available)
        missing(end+1) = "max_robot_steps OR max_steps_any_robot"; %#ok<AGROW>
    end
    if ismember(key,["PSHIL","CVHIL","PSHILConditions","CVHILConditions"]) && ...
            ~ismember("top_k_cells",available) && ~ismember("top_k_max_cells",available)
        missing(end+1) = "top_k_cells OR top_k_max_cells"; %#ok<AGROW>
    end

    if isempty(missing)
        Status(i) = "OK";
    else
        MissingColumns(i) = strjoin(missing, ", ");
        Status(i) = "MISSING COLUMNS";
        if isRequired
            fatalMessages(end+1,1) = key + " missing: " + MissingColumns(i); %#ok<AGROW>
        end
    end
end

SourceAudit = table(FileKey, Path, Required, Exists, Bytes, Modified, DataRows, ...
    MissingColumns, Status);

% Cross-check the combined HIL export when it is available.
psIndex = find(SourceAudit.FileKey == "PSHIL",1);
cvIndex = find(SourceAudit.FileKey == "CVHIL",1);
combinedIndex = find(SourceAudit.FileKey == "CombinedHIL",1);
if ~isempty(combinedIndex) && SourceAudit.Exists(combinedIndex) && ...
        SourceAudit.Exists(psIndex) && SourceAudit.Exists(cvIndex)
    expectedCombinedRows = SourceAudit.DataRows(psIndex) + SourceAudit.DataRows(cvIndex);
    if SourceAudit.DataRows(combinedIndex) ~= expectedCombinedRows
        SourceAudit.Status(combinedIndex) = "ROW COUNT MISMATCH";
        warning(['Combined HIL row count (%g) does not equal the sum of the mission ', ...
            'files (%g). Mission-specific files remain authoritative.'], ...
            SourceAudit.DataRows(combinedIndex),expectedCombinedRows);
    end
end

AnalysisSettings = table( ...
    ["MATLABRelease";"MATLABVersion";"StatisticsToolbox";"ParallelToolbox"; ...
     "UseParallel";"Permutations";"Alpha";"RandomSeed";"HILCallLimitMs"; ...
     "MethodName";"SensitivityGlobalIndependent";"SensitivityGlobalPaired"; ...
     "SensitivityPairwise";"SensitivityLevelwise";"TotalTeamSensitivitySource"], ...
    [string(version('-release')); string(version); string(statsInstalled); ...
     string(parallelInstalled); string(cfg.UseParallelIfAvailable); ...
     string(cfg.NumPermutations); string(cfg.Alpha); string(cfg.RandomSeed); ...
     string(cfg.HILCallLimitMs); cfg.MethodName; ...
     "Euclidean multivariate pseudo-F with unrestricted curve-label permutation"; ...
     "Within-trial environment-label permutation using centered curve centroid separation"; ...
     "Squared Euclidean distance between mean five-mode curves"; ...
     "Absolute mean degradation difference at each mode"; ...
     "Source-provided total_step_degradation_pct"], ...
    'VariableNames', ["Setting","Value"]);

fprintf('\nInput-file audit:\n');
disp(SourceAudit(:,["FileKey","Required","Exists","DataRows","Status"]));

if ~isempty(fatalMessages)
    fprintf(2, '\nPreflight failed. Resolve the following items:\n');
    for i = 1:numel(fatalMessages)
        fprintf(2, '  - %s\n', fatalMessages(i));
    end
    error('Input verification failed. No analysis outputs were written.');
end

fprintf('Preflight passed. No additional MATLAB toolbox is required.\n\n');

end

function ensureDirectory(folder)
if ~isfolder(folder)
    mkdir(folder);
end
end

function selected = discoverConditionFile(configuredPath, folder, label)
selected = string(configuredPath);
if isfile(selected)
    return;
end
if ~isfolder(folder)
    return;
end
patterns = ["*condition*aggregate*metrics*.csv", "*condition*metrics*.csv"];
candidates = strings(0,1);
for pattern = patterns
    listing = dir(fullfile(folder, pattern));
    if ~isempty(listing)
        candidates = [candidates; string(fullfile({listing.folder},{listing.name}))']; %#ok<AGROW>
    end
end
candidates = unique(candidates,'stable');
if numel(candidates) == 1
    selected = candidates(1);
    fprintf('Auto-detected %s condition file:\n  %s\n', label, selected);
elseif numel(candidates) > 1
    warning('Multiple %s condition files were found. Configure the exact path in analysis_config.m.\n%s', ...
        label, strjoin(candidates,newline));
end
end

function n = countFileLines(filePath)
fid = fopen(filePath,'r');
if fid < 0
    n = NaN;
    return;
end
cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
blockSize = 1024 * 1024;
n = 0;
while ~feof(fid)
    bytes = fread(fid, blockSize, '*uint8');
    n = n + sum(bytes == 10);
end
% Count the final non-newline-terminated row.
fseek(fid, -1, 'eof');
lastByte = fread(fid, 1, '*uint8');
if ~isempty(lastByte) && lastByte ~= 10
    n = n + 1;
end
end
