function result = AutoLandingDataParallel(num_workers, scenarios_per_worker, gazebo_server_mode, enable_visualization, mission_overrides)
% AutoLandingDataParallel
% Parallel data collection orchestration (inspired by IICC26 AutoSimMain).
% Spawns multiple workers collecting raw sensor data independently.
%
% Usage:
%   AutoLandingDataParallel()              % 4 workers, 10 scenarios each
%   AutoLandingDataParallel(2, 50)         % 2 workers, 50 scenarios each

if nargin < 1 || isempty(num_workers)
    num_workers = 4;
end
if nargin < 2 || isempty(scenarios_per_worker)
    scenarios_per_worker = 10;
end
if nargin < 3 || isempty(gazebo_server_mode)
    gazebo_server_mode = true;  % Default: server mode (headless)
end
if nargin < 4 || isempty(enable_visualization)
    enable_visualization = true;  % Default: enable real-time visualization
end
if nargin < 5 || isempty(mission_overrides)
    mission_overrides = struct();
end

% Validate inputs
num_workers = max(1, round(num_workers));
scenarios_per_worker = max(1, round(scenarios_per_worker));

rootDir = fileparts(fileparts(mfilename('fullpath')));

% Setup paths
modDir = fullfile(rootDir, 'matlab', 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

fprintf('\n');
fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  AutoLanding Parallel Data Collection\n');
fprintf('  Workers: %d\n', num_workers);
fprintf('  Scenarios per Worker: %d\n', scenarios_per_worker);
fprintf('  Total Scenarios: %d\n', num_workers * scenarios_per_worker);
fprintf('═══════════════════════════════════════════════════════\n');

% Create output directories
output_base = fullfile(rootDir, 'data', 'collected', 'parallel');
log_base = fullfile(rootDir, 'data', 'logs', 'parallel');

if ~exist(output_base, 'dir')
    mkdir(output_base);
end
if ~exist(log_base, 'dir')
    mkdir(log_base);
end

% Session ID
session_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
session_dir = fullfile(output_base, session_id);
if ~exist(session_dir, 'dir')
    mkdir(session_dir);
end
flow_log_dir = fullfile(session_dir, 'flow_logs');
if ~exist(flow_log_dir, 'dir')
    mkdir(flow_log_dir);
end
session_flow_log = fullfile(flow_log_dir, 'pipeline_flow.jsonl');
session_start_time = datetime('now');

fprintf('\n[AutoLandingDataParallel] Session ID: %s\n', session_id);
fprintf('[AutoLandingDataParallel] Output: %s\n\n', session_dir);
autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'session_started', struct( ...
    'session_id', session_id, 'num_workers', num_workers, 'scenarios_per_worker', scenarios_per_worker, ...
    'total_scenarios', num_workers * scenarios_per_worker));

% Register cleanup function for graceful shutdown (Ctrl+C)
cleanup_obj = onCleanup(@() autlParallelCleanup(session_dir, num_workers));

% Build worker configuration
worker_config = struct();
worker_config.scenarios_per_worker = scenarios_per_worker;
worker_config.sample_rate = 50;  % Hz
worker_config.scenario_duration = 120;  % seconds
worker_config.output_dir = session_dir;
worker_config.worker_log_dir = fullfile(log_base, session_id);
worker_config.flow_log_dir = flow_log_dir;
worker_config.flow_log_file = session_flow_log;
worker_config.gazebo_server_mode = gazebo_server_mode;  % Server or GUI
worker_config.enable_visualization = enable_visualization;  % Real-time monitoring
worker_config.mission_overrides = mission_overrides;
worker_config.num_workers = num_workers;
worker_config.progress_queue = [];
worker_config.reject_fallback_only_mode = true;
if isstruct(mission_overrides) && isfield(mission_overrides, 'reject_fallback_only_mode')
    worker_config.reject_fallback_only_mode = logical(mission_overrides.reject_fallback_only_mode);
end

% Root cause guard:
% parfor uses process workers, and process workers cannot reliably own desktop figure windows.
% Keep visualization on the client side and disable worker-local MATLAB figures.
if num_workers > 1 && worker_config.enable_visualization
    worker_config.enable_visualization = false;
    fprintf(['[AutoLandingDataParallel] Info: Worker MATLAB figure visualization disabled in multi-worker mode ', ...
        '(parfor process workers do not support stable desktop figure rendering).\n']);
    fprintf('[AutoLandingDataParallel] Info: Client real-time monitor enabled for multi-worker progress visualization.\n');
    autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'worker_figure_viz_disabled', struct( ...
        'reason', 'parfor_process_workers_no_stable_desktop_figure', 'num_workers', num_workers));
end

% Create shared worker log root once to avoid mkdir race warnings in workers
if ~exist(worker_config.worker_log_dir, 'dir')
    mkdir(worker_config.worker_log_dir);
end

% Create tasks array for parallel.pool.DataQueue
if num_workers == 1 || verLessThan('matlab', '9.9')
    % Use sequential execution for single-worker mode (and older MATLAB).
    % This avoids unnecessary parfor orchestration/interruption overhead.
    if num_workers == 1
        fprintf('[AutoLandingDataParallel] Single-worker mode detected. Running sequentially (no parfor).\n\n');
    else
        fprintf('[AutoLandingDataParallel] MATLAB version < 9.9 detected. Running sequential workers.\n\n');
    end
    
    worker_failed = false;
    for worker_id = 1:num_workers
        try
            fprintf('[AutoLandingDataParallel] Launching worker %d/%d\n', worker_id, num_workers);
            autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'worker_launch', struct( ...
                'worker_id', worker_id, 'mode', 'sequential'));
            worker_cfg_i = autlBuildWorkerConfig(worker_config, worker_id);
            autlDataCollectorWorker(worker_id, worker_cfg_i);
            fprintf('[AutoLandingDataParallel] Worker %d completed.\n\n', worker_id);
            autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'worker_completed', struct( ...
                'worker_id', worker_id, 'mode', 'sequential'));
        catch ME
            if autlParallelIsUserInterrupt(ME)
                rethrow(ME);
            end
            fprintf('[AutoLandingDataParallel] Worker %d FAILED: %s\n', worker_id, ME.message);
            autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'worker_failed', struct( ...
                'worker_id', worker_id, 'mode', 'sequential', 'error', ME.message));
            worker_failed = true;
        end
    end
    if worker_failed
        error('AutoLanding:ParallelCollectionFailed', '[AutoLandingDataParallel] One or more workers failed during sequential collection.');
    end
else
    % Use parallel pool with DataQueue (MATLAB 9.9+)
    fprintf('[AutoLandingDataParallel] Creating parallel pool (%d workers)...\n\n', num_workers);
    autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'parallel_pool_create', struct('num_workers', num_workers));

    monitor_enabled = logical(enable_visualization && num_workers > 1);
    autlParallelMonitor('close');
    if monitor_enabled
        autlParallelMonitor('init', num_workers, scenarios_per_worker, session_id);
    end
    monitor_cleanup = onCleanup(@() autlParallelMonitor('close')); %#ok<NASGU>
    
    % Always recreate pool to avoid stale worker function cache across reruns.
    pool = gcp('nocreate');
    if ~isempty(pool)
        fprintf('[AutoLandingDataParallel] Recreating existing pool to refresh worker code cache...\n');
        delete(pool);
    end
    pool = parpool(num_workers, 'IdleTimeout', 240);

    % Force workers to reload updated function definitions.
    try
        pctRunOnAll('rehash; clear functions; clear autlRunDataCollection; clear autlDataCollectorWorker;');
    catch
        % Non-fatal; parfor will still proceed.
    end

    dq = parallel.pool.DataQueue;
    if monitor_enabled
        afterEach(dq, @(msg) autlParallelMonitor('update', msg));
        worker_config.progress_queue = dq;
    end
    
    % Launch workers via parfor
    results = cellArray(num_workers);
    worker_cfg_all = cellArray(num_workers);
    for worker_id = 1:num_workers
        worker_cfg_all{worker_id} = autlBuildWorkerConfig(worker_config, worker_id);
    end
    worker_ids = 1:num_workers;
    
    parfor worker_id = worker_ids
        try
            % Each worker runs independently
            autlDataCollectorWorker(worker_id, worker_cfg_all{worker_id});
            results{worker_id} = 'success';
        catch ME
            results{worker_id} = sprintf('error: %s', ME.message);
        end
    end
    
    % Summarize results
    fprintf('\n[AutoLandingDataParallel] Parallel execution results:\n');
    success_count = 0;
    for i = 1:num_workers
        if strcmp(results{i}, 'success')
            fprintf('  Worker %d: SUCCESS\n', i);
            success_count = success_count + 1;
            autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'worker_completed', struct( ...
                'worker_id', i, 'mode', 'parallel'));
        else
            fprintf('  Worker %d: %s\n', i, results{i});
            autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'worker_failed', struct( ...
                'worker_id', i, 'mode', 'parallel', 'error', results{i}));
        end
    end
    
    fprintf('\n[AutoLandingDataParallel] %d/%d workers completed successfully.\n', success_count, num_workers);
    if success_count < num_workers
        error('AutoLanding:ParallelCollectionFailed', '[AutoLandingDataParallel] %d/%d workers failed.', num_workers - success_count, num_workers);
    end

    if monitor_enabled
        autlParallelMonitor('flush');
    end
end

function tf = autlParallelIsUserInterrupt(ME)
% True when exception corresponds to Ctrl+C/user cancellation.

tf = false;
if nargin < 1 || isempty(ME)
    return;
end

id = lower(string(ME.identifier));
msg = lower(string(ME.message));
tf = contains(id, "operationterminatedbyuser") || ...
     contains(id, "interrupted") || ...
     contains(id, "autolanding:userinterrupted") || ...
     contains(msg, "operation terminated by user") || ...
     contains(msg, "terminated by user") || ...
     contains(msg, "interrupted") || ...
     contains(msg, "ctrl+c");

if tf
    return;
end

try
    causes = ME.cause;
    for i = 1:numel(causes)
        if autlParallelIsUserInterrupt(causes{i})
            tf = true;
            return;
        end
    end
catch
end
end

% Monitor progress
fprintf('\n[AutoLandingDataParallel] Session directory structure:\n');
fprintf('  %s\n', session_dir);
workerDirs = dir(fullfile(session_dir, 'worker_*'));
for i = 1:numel(workerDirs)
    if workerDirs(i).isdir
        scenarioDirs = dir(fullfile(session_dir, workerDirs(i).name, 'scenario_*'));
        fprintf('    %s/ (%d scenarios)\n', workerDirs(i).name, numel(scenarioDirs));
    end
end

% Merge metadata
fprintf('\n[AutoLandingDataParallel] Creating session metadata...\n');
session_meta = struct();
session_meta.session_id = session_id;
session_meta.num_workers = num_workers;
session_meta.scenarios_per_worker = scenarios_per_worker;
session_meta.total_scenarios = num_workers * scenarios_per_worker;
session_meta.start_time = session_start_time;
session_meta.collection_type = 'raw_parallel';
session_meta.ontology_applied = false;

meta_path = fullfile(session_dir, 'session_metadata.json');
autlSaveJson(meta_path, session_meta);

fprintf('[AutoLandingDataParallel] Session metadata: %s\n', meta_path);
autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'session_metadata_saved', struct('path', meta_path));

flow_report = autlGenerateFlowStopReport(session_dir);
if isstruct(flow_report) && isfield(flow_report, 'ok') && flow_report.ok
    fprintf('[AutoLandingDataParallel] Flow stop report: %s\n', flow_report.report_path);
end

autlFlowLog(session_flow_log, 'AutoLandingDataParallel', 'session_completed', struct( ...
    'actual_collected', numel(dir(fullfile(session_dir, 'worker_*', 'scenario_*', 'raw_data.mat')))));
fprintf('\n[AutoLandingDataParallel] Parallel data collection COMPLETE.\n');
fprintf('  Next steps: Process collected raw data offline with ontology/AI fusion pipeline.\n\n');

% Return session directory and actually collected raw-data scenario count.
actual_raw_files = dir(fullfile(session_dir, 'worker_*', 'scenario_*', 'raw_data.mat'));
actual_collected = numel(actual_raw_files);
if actual_collected == 0
    error('AutoLanding:NoRawData', '[AutoLandingDataParallel] No raw_data.mat files were produced in %s', session_dir);
end
result = {session_dir, actual_collected};

end

function cellArr = cellArray(n)
% Helper for MATLAB compatibility
cellArr = cell(n, 1);
end

function autlParallelMonitor(action, varargin)
% Client-side visualization for multi-worker progress using DataQueue updates.

persistent st
if isempty(st)
    st = struct('initialized', false);
end

switch lower(string(action))
    case "init"
        num_workers = varargin{1};
        scenarios_per_worker = varargin{2};
        session_id = char(string(varargin{3}));

        st = struct();
        st.initialized = true;
        st.num_workers = num_workers;
        st.scenarios_per_worker = scenarios_per_worker;
        st.started = zeros(1, num_workers);
        st.success = zeros(1, num_workers);
        st.failed = zeros(1, num_workers);
        st.current = zeros(1, num_workers);

        try
            st.fig = figure('Name', sprintf('AutoLanding Parallel Monitor (%s)', session_id), ...
                'NumberTitle', 'off', 'Tag', 'autl_parallel_monitor', 'Position', [120, 120, 980, 480]);
            st.ax1 = subplot(1, 2, 1, 'Parent', st.fig);
            st.bar_started = bar(st.ax1, 1:num_workers, st.started, 0.25, 'FaceColor', [0.3 0.6 0.9]);
            hold(st.ax1, 'on');
            st.bar_success = bar(st.ax1, 1:num_workers, st.success, 0.25, 'FaceColor', [0.2 0.7 0.3]);
            st.bar_failed = bar(st.ax1, 1:num_workers, st.failed, 0.25, 'FaceColor', [0.85 0.25 0.25]);
            hold(st.ax1, 'off');
            ylim(st.ax1, [0, max(1, scenarios_per_worker)]);
            xticks(st.ax1, 1:num_workers);
            xlabel(st.ax1, 'Worker ID');
            ylabel(st.ax1, 'Scenario Count');
            title(st.ax1, 'Started / Success / Failed');
            legend(st.ax1, {'Started','Success','Failed'}, 'Location', 'northoutside');
            grid(st.ax1, 'on');

            st.ax2 = subplot(1, 2, 2, 'Parent', st.fig);
            st.txt = text(st.ax2, 0.02, 0.98, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
                'FontName', 'Courier', 'FontSize', 10);
            axis(st.ax2, 'off');
            autlParallelMonitorRefreshText();
            drawnow;
        catch
            st.initialized = false;
        end

    case "update"
        if ~st.initialized
            return;
        end
        msg = varargin{1};
        if ~isstruct(msg) || ~isfield(msg, 'worker_id') || ~isfield(msg, 'event')
            return;
        end
        wid = double(msg.worker_id);
        if wid < 1 || wid > st.num_workers
            return;
        end

        ev = lower(char(string(msg.event)));
        switch ev
            case 'scenario_started'
                if isfield(msg, 'scenario_num')
                    st.current(wid) = max(st.current(wid), double(msg.scenario_num));
                end
                st.started(wid) = max(st.started(wid), st.current(wid));
            case 'scenario_success'
                st.success(wid) = st.success(wid) + 1;
            case 'scenario_failed'
                st.failed(wid) = st.failed(wid) + 1;
            otherwise
        end

        autlParallelMonitorRefreshBars();
        autlParallelMonitorRefreshText();
        drawnow limitrate;

    case "flush"
        if st.initialized
            autlParallelMonitorRefreshBars();
            autlParallelMonitorRefreshText();
            drawnow;
        end

    case "close"
        if st.initialized && isfield(st, 'fig') && isgraphics(st.fig)
            try
                close(st.fig);
            catch
            end
        end
        st = struct('initialized', false);
end

    function autlParallelMonitorRefreshBars()
        if ~st.initialized || ~isgraphics(st.bar_started)
            return;
        end
        set(st.bar_started, 'YData', st.started);
        set(st.bar_success, 'YData', st.success);
        set(st.bar_failed, 'YData', st.failed);
        ymax = max([1, st.scenarios_per_worker, max(st.started), max(st.success + st.failed)]);
        ylim(st.ax1, [0, ymax]);
    end

    function autlParallelMonitorRefreshText()
        if ~st.initialized || ~isgraphics(st.txt)
            return;
        end
        lines = strings(0, 1);
        total_success = sum(st.success);
        total_failed = sum(st.failed);
        total_started = sum(st.started);
        total_target = st.num_workers * st.scenarios_per_worker;
        lines(end+1) = sprintf('Total progress: started=%d, success=%d, failed=%d / target=%d', ...
            total_started, total_success, total_failed, total_target);
        lines(end+1) = "";
        for wi = 1:st.num_workers
            lines(end+1) = sprintf('Worker %d  current=%d  started=%d  success=%d  failed=%d', ...
                wi, st.current(wi), st.started(wi), st.success(wi), st.failed(wi));
        end
        set(st.txt, 'String', strjoin(cellstr(lines), newline));
    end
end

function autlParallelCleanup(session_dir, num_workers)
% Cleanup function for graceful shutdown
% Called automatically when AutoLandingDataParallel ends (including on Ctrl+C)

fprintf('\n[AutoLandingDataParallel.cleanup] Performing cleanup...\n');
cleanup_flow_log = fullfile(session_dir, 'flow_logs', 'pipeline_flow.jsonl');
autlFlowLog(cleanup_flow_log, 'AutoLandingDataParallel.cleanup', 'cleanup_started', struct('num_workers', num_workers));

% Kill all Gazebo sim and SITL processes to prevent port conflicts
fprintf('[AutoLandingDataParallel.cleanup] Killing gazebo and ArduPilot processes...\n');
try
    system('pkill -f "gz sim" 2>/dev/null');
    system('pkill -f "mavproxy.py" 2>/dev/null');
    pause(0.5);
    fprintf('[AutoLandingDataParallel.cleanup] Processes terminated.\n');
    autlFlowLog(cleanup_flow_log, 'AutoLandingDataParallel.cleanup', 'processes_terminated', struct());
catch
    % Silent fail
end

% Kill parallel pool if it exists
try
    pool = gcp('nocreate');
    if ~isempty(pool)
        fprintf('[AutoLandingDataParallel.cleanup] Deleting parallel pool...\n');
        delete(pool);
        autlFlowLog(cleanup_flow_log, 'AutoLandingDataParallel.cleanup', 'parallel_pool_deleted', struct());
    end
catch
    % Silent fail
end

% Ensure session directory is readable (check latest data)
try
    if isdir(session_dir)
        fprintf('[AutoLandingDataParallel.cleanup] Session data saved in: %s\n', session_dir);
        % Count completed scenarios
        all_scenarios = dir(fullfile(session_dir, 'worker_*', 'scenario_*'));
        fprintf('[AutoLandingDataParallel.cleanup] Recovered %d scenario directories\n', numel(all_scenarios));
        autlFlowLog(cleanup_flow_log, 'AutoLandingDataParallel.cleanup', 'session_recovered', struct( ...
            'scenario_dirs', numel(all_scenarios)));

        % Generate stop report even on interrupted runs.
        flow_report = autlGenerateFlowStopReport(session_dir);
        if isstruct(flow_report) && isfield(flow_report, 'ok') && flow_report.ok
            fprintf('[AutoLandingDataParallel.cleanup] Flow stop report: %s\n', flow_report.report_path);
            autlFlowLog(cleanup_flow_log, 'AutoLandingDataParallel.cleanup', 'flow_report_generated', struct( ...
                'report_path', flow_report.report_path));
        end
    end
catch
    % Silent fail
end

fprintf('[AutoLandingDataParallel.cleanup] Cleanup complete.\n');
autlFlowLog(cleanup_flow_log, 'AutoLandingDataParallel.cleanup', 'cleanup_completed', struct());
end

function worker_cfg = autlBuildWorkerConfig(base_cfg, worker_id)
% Create per-worker config so each worker can collect from distinct vehicle states.

worker_cfg = base_cfg;

if ~isfield(worker_cfg, 'mission_overrides') || ~isstruct(worker_cfg.mission_overrides)
    worker_cfg.mission_overrides = struct();
end

overrides = worker_cfg.mission_overrides;
if isfield(overrides, 'worker_profiles') && ~isempty(overrides.worker_profiles)
    profiles = overrides.worker_profiles;
    if isstruct(profiles)
        if numel(profiles) >= worker_id
            p = profiles(worker_id);
        else
            p = profiles(end);
        end

        p_fields = fieldnames(p);
        for fi = 1:numel(p_fields)
            f = p_fields{fi};
            overrides.(f) = p.(f);
        end
    end
end

if ~isfield(overrides, 'worker_state_tag') || strlength(string(overrides.worker_state_tag)) == 0
    overrides.worker_state_tag = sprintf('worker_%d', worker_id);
end

worker_cfg.mission_overrides = overrides;
end
