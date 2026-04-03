function autlDataCollectorWorker(worker_id, config)
% autlDataCollectorWorker
% Single worker for parallel data collection.
% Spawned by AutoLandingDataParallel as independent process.
%
% Usage:
%   autlDataCollectorWorker(1, config_struct)

if nargin < 1
    worker_id = 1;
end
if nargin < 2
    config = struct();
end

% Resolve root directory
rootDir = fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));

% Setup paths
modDir = fullfile(rootDir, 'matlab', 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

% Worker configuration defaults
if ~isfield(config, 'scenarios_per_worker')
    config.scenarios_per_worker = 10;
end
if ~isfield(config, 'sample_rate')
    config.sample_rate = 50;
end
if ~isfield(config, 'scenario_duration')
    config.scenario_duration = 120;
end
if ~isfield(config, 'output_dir')
    config.output_dir = fullfile(rootDir, 'data', 'collected', 'parallel');
end
if ~isfield(config, 'worker_log_dir')
    config.worker_log_dir = fullfile(rootDir, 'data', 'logs', sprintf('worker_%d', worker_id));
end
if ~isfield(config, 'enable_realtime_viz')
    config.enable_realtime_viz = false;
end
if ~isfield(config, 'min_samples_per_scenario')
    config.min_samples_per_scenario = 1;
end
if ~isfield(config, 'gazebo_server_mode')
    config.gazebo_server_mode = true;  % Default: server/headless mode
end
if ~isfield(config, 'enable_visualization')
    config.enable_visualization = true;  % Default: enable visualization
end
if ~isfield(config, 'reset_pose_each_scenario')
    config.reset_pose_each_scenario = true;
end
if ~isfield(config, 'reset_ardupilot_each_scenario')
    config.reset_ardupilot_each_scenario = true;
end
if ~isfield(config, 'progress_queue')
    config.progress_queue = [];
end
if ~isfield(config, 'reject_fallback_only_mode')
    config.reject_fallback_only_mode = true;
end

% Create worker output directory
if ~exist(config.worker_log_dir, 'dir')
    try
        warn_state = warning('off', 'MATLAB:MKDIR:DirectoryExists');
        cleanup_warn = onCleanup(@() warning(warn_state)); %#ok<NASGU>
        mkdir(config.worker_log_dir);
    catch ME
        % Parallel workers can race on mkdir; ignore benign "already exists".
        if ~contains(lower(ME.message), 'exists')
            rethrow(ME);
        end
    end
end

worker_log_file = fullfile(config.worker_log_dir, sprintf('worker_%d.log', worker_id));
worker_flow_log = '';
if isfield(config, 'flow_log_dir') && strlength(string(config.flow_log_dir)) > 0
    worker_flow_log = fullfile(char(string(config.flow_log_dir)), sprintf('worker_%d_flow.jsonl', worker_id));
elseif isfield(config, 'flow_log_file') && strlength(string(config.flow_log_file)) > 0
    worker_flow_log = char(string(config.flow_log_file));
end
log_fid = fopen(worker_log_file, 'a');

fprintf(log_fid, '[Worker %d] Started at %s\n', worker_id, datetime('now'));
fprintf(log_fid, '[Worker %d] Config: scenarios=%d, rate=%d Hz, duration=%.0f sec\n', ...
    worker_id, config.scenarios_per_worker, config.sample_rate, config.scenario_duration);
autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'worker_started', struct( ...
    'worker_id', worker_id, 'scenarios_per_worker', config.scenarios_per_worker, ...
    'sample_rate', config.sample_rate, 'scenario_duration', config.scenario_duration));
autlWorkerSendProgress(config, struct('event', 'worker_started', 'worker_id', worker_id, ...
    'scenarios_per_worker', config.scenarios_per_worker));

% Register cleanup for this worker
worker_state = struct('log_fid', log_fid, 'worker_id', worker_id);
cleanup_obj = onCleanup(@() autlWorkerCleanup(worker_state));

try
    % Per-scenario data collection loop
    scenario_success_count = 0;
    scenario_fail_count = 0;
    for scenario_num = 1:config.scenarios_per_worker
        fprintf('[Worker %d] Collecting scenario %d/%d...\n', ...
            worker_id, scenario_num, config.scenarios_per_worker);
        fprintf(log_fid, '[Worker %d] Collecting scenario %d/%d at %s\n', ...
            worker_id, scenario_num, config.scenarios_per_worker, datetime('now'));
        autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'scenario_started', struct( ...
            'worker_id', worker_id, 'scenario_num', scenario_num));
        autlWorkerSendProgress(config, struct('event', 'scenario_started', 'worker_id', worker_id, ...
            'scenario_num', scenario_num));

        % Force-refresh function cache so workers do not keep stale local-function maps.
        try
            rehash;
            clear functions;
            clear autlRunDataCollection;
        catch
        end
        
        % Setup mission config
        mission_cfg = struct();
        mission_cfg.max_duration = config.scenario_duration;
        mission_cfg.sample_rate = config.sample_rate;
        mission_cfg.output_dir = fullfile(config.output_dir, sprintf('worker_%d', worker_id));
        mission_cfg.session_id = sprintf('scenario_%d_%s', scenario_num, ...
            char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
        mission_cfg.gazebo_server_mode = config.gazebo_server_mode;  % Pass server mode
        mission_cfg.enable_visualization = config.enable_visualization;  % Pass visualization flag
        mission_cfg.log_prefix = sprintf('[Worker %d] ', worker_id);
        mission_cfg.flow_log_file = worker_flow_log;
        mission_cfg.flow_context = struct('worker_id', worker_id, 'scenario_num', scenario_num);
        if isfield(config, 'mission_overrides') && isstruct(config.mission_overrides)
            override_fields = fieldnames(config.mission_overrides);
            for f_idx = 1:numel(override_fields)
                f_name = override_fields{f_idx};
                mission_cfg.(f_name) = config.mission_overrides.(f_name);
            end
        end
        if ~isfield(mission_cfg, 'reset_pose_each_scenario')
            mission_cfg.reset_pose_each_scenario = config.reset_pose_each_scenario;
        end
        if ~isfield(mission_cfg, 'reset_ardupilot_each_scenario')
            mission_cfg.reset_ardupilot_each_scenario = config.reset_ardupilot_each_scenario;
        end
        if ~isfield(mission_cfg, 'mavlink_master')
            mission_cfg.mavlink_master = 'tcp:127.0.0.1:5760';
        end
        if ~isfield(mission_cfg, 'mavlink_control_timeout_s')
            mission_cfg.mavlink_control_timeout_s = 30.0;
        end
        if ~isfield(mission_cfg, 'mavlink_precheck_timeout_s') || ~isfinite(mission_cfg.mavlink_precheck_timeout_s)
            mission_cfg.mavlink_precheck_timeout_s = mission_cfg.mavlink_control_timeout_s;
        end
        % Keep probes bounded, but avoid too-short timeouts that force false fallback-only mode.
        mission_cfg.mavlink_precheck_timeout_s = min(30.0, max(6.0, double(mission_cfg.mavlink_precheck_timeout_s)));
        mission_cfg.mavlink_control_timeout_s = min(30.0, max(6.0, double(mission_cfg.mavlink_control_timeout_s)));
        if isfield(mission_cfg, 'enable_auto_motion') && mission_cfg.enable_auto_motion && worker_id ~= 1
            if strcmp(char(string(mission_cfg.mavlink_master)), 'tcp:127.0.0.1:5760')
                mission_cfg.enable_auto_motion = false;
                fprintf(log_fid, '[Worker %d] Auto motion disabled: shared MAVLink master (%s).\n', worker_id, char(string(mission_cfg.mavlink_master)));
            else
                fprintf(log_fid, '[Worker %d] Auto motion enabled with dedicated MAVLink master (%s).\n', worker_id, char(string(mission_cfg.mavlink_master)));
            end
        end
        if isfield(mission_cfg, 'worker_state_tag')
            fprintf(log_fid, '[Worker %d] Worker state profile: %s\n', worker_id, char(string(mission_cfg.worker_state_tag)));
        end

        if scenario_num == 1
            [ready_ok, ready_msg] = autlWaitForMavlinkReady(mission_cfg);
            if ready_ok
                fprintf(log_fid, '[Worker %d] MAVLink ready gate passed: %s\n', worker_id, ready_msg);
            else
                fprintf(log_fid, '[Worker %d] MAVLink ready gate warning: %s\n', worker_id, ready_msg);
            end
        end

        if config.num_workers > 1
            [barrier_ok, barrier_msg] = autlWaitForWorkerStartBarrier(config, worker_id, log_fid);
            if barrier_ok
                fprintf(log_fid, '[Worker %d] Start barrier passed: %s\n', worker_id, barrier_msg);
            else
                fprintf(log_fid, '[Worker %d] Start barrier warning: %s\n', worker_id, barrier_msg);
            end
        end

        if mission_cfg.reset_pose_each_scenario
            [reset_ok, reset_msg] = autlResetGazeboDronePose(mission_cfg);
            if reset_ok
                fprintf('[Worker %d] Scenario %d pose reset applied: %s\n', worker_id, scenario_num, reset_msg);
                fprintf(log_fid, '[Worker %d] Scenario %d pose reset applied: %s\n', worker_id, scenario_num, reset_msg);
            else
                fprintf('[Worker %d] Scenario %d pose reset skipped/failed: %s\n', worker_id, scenario_num, reset_msg);
                fprintf(log_fid, '[Worker %d] Scenario %d pose reset skipped/failed: %s\n', worker_id, scenario_num, reset_msg);
            end
            autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'pose_reset', struct( ...
                'worker_id', worker_id, 'scenario_num', scenario_num, 'ok', reset_ok, 'message', reset_msg));
        end

        if mission_cfg.reset_ardupilot_each_scenario
            [ap_ok, ap_msg] = autlResetArduPilotState(mission_cfg);
            if ap_ok
                fprintf('[Worker %d] Scenario %d ArduPilot reset applied: %s\n', worker_id, scenario_num, ap_msg);
                fprintf(log_fid, '[Worker %d] Scenario %d ArduPilot reset applied: %s\n', worker_id, scenario_num, ap_msg);
            else
                fprintf('[Worker %d] Scenario %d ArduPilot reset warning: %s\n', worker_id, scenario_num, ap_msg);
                fprintf(log_fid, '[Worker %d] Scenario %d ArduPilot reset warning: %s\n', worker_id, scenario_num, ap_msg);
                ap_msg_l = lower(char(string(ap_msg)));
                if contains(ap_msg_l, 'mavlink_unreachable') || contains(ap_msg_l, 'fcu not connected') || contains(ap_msg_l, 'state echo timeout')
                    if config.reject_fallback_only_mode
                        error('AutoLanding:FallbackOnlyRejected', ...
                            'Scenario %d rejected: MAVROS state/topic unreachable (%s)', scenario_num, ap_msg);
                    else
                        mission_cfg.enable_auto_motion = false;
                        mission_cfg.require_mavlink_for_auto_motion = false;
                        fprintf('[Worker %d] Scenario %d fallback-only mode forced (MAVLink unreachable).\n', worker_id, scenario_num);
                        fprintf(log_fid, '[Worker %d] Scenario %d fallback-only mode forced (MAVLink unreachable).\n', worker_id, scenario_num);
                    end
                end
            end
            autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'ardupilot_reset', struct( ...
                'worker_id', worker_id, 'scenario_num', scenario_num, 'ok', ap_ok, 'message', ap_msg));
        end

        if mission_cfg.reset_pose_each_scenario || mission_cfg.reset_ardupilot_each_scenario
            pause(0.5);
        end
        
        % Raw data collection (NO ontology processing)
        try
            fprintf(log_fid, '[Worker %d] Scenario %d: Calling autlRunDataCollection...\n', worker_id, scenario_num);
            autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'collection_call', struct( ...
                'worker_id', worker_id, 'scenario_num', scenario_num));
            result = autlRunDataCollection(rootDir, mission_cfg);
            if ~isstruct(result) || ~isfield(result, 'status')
                error('AutoLanding:CollectionFailed', 'Invalid collection result for worker %d scenario %d.', worker_id, scenario_num);
            end
            if isstruct(result) && isfield(result, 'status') && strcmpi(string(result.status), "interrupted")
                error('AutoLanding:UserInterrupted', 'User interrupted during data collection (worker %d, scenario %d).', worker_id, scenario_num);
            end
            if ~strcmpi(string(result.status), "completed")
                err_msg = 'unknown';
                if isfield(result, 'error_message')
                    err_msg = char(string(result.error_message));
                end
                error('AutoLanding:CollectionFailed', 'Collection not completed (status=%s): %s', char(string(result.status)), err_msg);
            end

            samples = 0;
            duration = 0.0;
            if isfield(result, 'sample_count'), samples = result.sample_count; end
            if isfield(result, 'duration'), duration = result.duration; end
            if samples < config.min_samples_per_scenario
                error('AutoLanding:CollectionInsufficientSamples', ...
                    'Collected only %d sample(s), below minimum %d.', samples, config.min_samples_per_scenario);
            end
            fprintf('[Worker %d] Scenario %d: SUCCESS - %d samples in %.1f sec\n', ...
                worker_id, scenario_num, samples, duration);
            fprintf(log_fid, '[Worker %d] Scenario %d collected: %d samples in %.1f sec\n', ...
                worker_id, scenario_num, samples, duration);
            autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'scenario_completed', struct( ...
                'worker_id', worker_id, 'scenario_num', scenario_num, 'samples', samples, 'duration_s', duration));
            scenario_success_count = scenario_success_count + 1;
            autlWorkerSendProgress(config, struct('event', 'scenario_success', 'worker_id', worker_id, ...
                'scenario_num', scenario_num, 'samples', samples, 'duration_s', duration));
            
        catch ME
            if autlWorkerIsUserInterrupt(ME)
                rethrow(ME);
            end
            fprintf('[Worker %d] Scenario %d FAILED: %s\n', worker_id, scenario_num, ME.message);
            fprintf(log_fid, '[Worker %d] Scenario %d FAILED: %s\n', worker_id, scenario_num, ME.message);
            fprintf(log_fid, '%s\n', getReport(ME));
            autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'scenario_failed', struct( ...
                'worker_id', worker_id, 'scenario_num', scenario_num, 'error', ME.message));
            scenario_fail_count = scenario_fail_count + 1;
            autlWorkerSendProgress(config, struct('event', 'scenario_failed', 'worker_id', worker_id, ...
                'scenario_num', scenario_num, 'error', ME.message));
        end
        
        % Brief delay between scenarios
        pause(1);
    end
    
    fprintf('[Worker %d] Collection complete. success=%d, failed=%d, total=%d\n', ...
        worker_id, scenario_success_count, scenario_fail_count, config.scenarios_per_worker);
    fprintf(log_fid, '[Worker %d] Collection complete at %s (success=%d, failed=%d)\n', ...
        worker_id, datetime('now'), scenario_success_count, scenario_fail_count);
    autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'worker_completed', struct( ...
        'worker_id', worker_id, 'success_count', scenario_success_count, 'fail_count', scenario_fail_count));
    autlWorkerSendProgress(config, struct('event', 'worker_completed', 'worker_id', worker_id, ...
        'success_count', scenario_success_count, 'fail_count', scenario_fail_count));

    if scenario_fail_count > 0
        error('AutoLanding:WorkerScenarioFailed', 'Worker %d had %d failed scenario(s).', worker_id, scenario_fail_count);
    end
    
catch ME
    % Check if user interrupted
    if autlWorkerIsUserInterrupt(ME)
        fprintf(log_fid, '[Worker %d] User interrupted. Saved data recovered.\n', worker_id);
        fprintf('[Worker %d] INTERRUPTED - partial results saved.\n', worker_id);
        autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'worker_interrupted', struct('worker_id', worker_id));
        rethrow(ME);
    else
        fprintf(log_fid, '[Worker %d] CRITICAL ERROR: %s\n', worker_id, ME.message);
        fprintf(log_fid, '%s\n', getReport(ME));
        autlFlowLog(worker_flow_log, 'autlDataCollectorWorker', 'worker_failed', struct( ...
            'worker_id', worker_id, 'error', ME.message));
        rethrow(ME);
    end
end

fclose(log_fid);
fprintf('[Worker %d] Exiting.\n', worker_id);
end

function autlWorkerSendProgress(config, msg)
% Send non-critical progress events to client-side monitor.

if nargin < 2 || ~isstruct(msg)
    return;
end

try
    if isstruct(config) && isfield(config, 'progress_queue') && ~isempty(config.progress_queue)
        send(config.progress_queue, msg);
    end
catch
    % Monitoring must not affect data collection stability.
end
end

function tf = autlWorkerIsUserInterrupt(ME)
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
        if autlWorkerIsUserInterrupt(causes{i})
            tf = true;
            return;
        end
    end
catch
end
end

function autlWorkerCleanup(worker_state)
% Cleanup for individual worker

try
    if isfield(worker_state, 'log_fid') && ~isempty(worker_state.log_fid)
        worker_id = worker_state.worker_id;
        if isvalid(worker_state.log_fid)
            fprintf(worker_state.log_fid, '[Worker %d] Cleanup: Closing log file\n', worker_id);
            fclose(worker_state.log_fid);
        end
    end
catch
    % Silent fail
end

end

function [ok, msg] = autlResetGazeboDronePose(mission_cfg)
% Reset drone pose via Gazebo service before each scenario.

ok = false;
msg = '';

if ~isfield(mission_cfg, 'landing_pad_center')
    mission_cfg.landing_pad_center = [0.0, 0.0, 0.0];
end
if ~isfield(mission_cfg, 'spawn_radius_m')
    mission_cfg.spawn_radius_m = 0.35;
end
if ~isfield(mission_cfg, 'spawn_angle_deg')
    mission_cfg.spawn_angle_deg = 15.0;
end
if ~isfield(mission_cfg, 'reset_spawn_z_m')
    mission_cfg.reset_spawn_z_m = 0.195;
end
if ~isfield(mission_cfg, 'reset_spawn_yaw_deg')
    mission_cfg.reset_spawn_yaw_deg = 90.0;
end
if ~isfield(mission_cfg, 'reset_model_name')
    mission_cfg.reset_model_name = 'iris_with_gimbal';
end

if isfield(mission_cfg, 'reset_spawn_xy') && numel(mission_cfg.reset_spawn_xy) >= 2
    spawn_x = mission_cfg.reset_spawn_xy(1);
    spawn_y = mission_cfg.reset_spawn_xy(2);
else
    ang = deg2rad(mission_cfg.spawn_angle_deg);
    spawn_x = mission_cfg.landing_pad_center(1) + mission_cfg.spawn_radius_m * cos(ang);
    spawn_y = mission_cfg.landing_pad_center(2) + mission_cfg.spawn_radius_m * sin(ang);
end

yaw_rad = deg2rad(mission_cfg.reset_spawn_yaw_deg);
qz = sin(yaw_rad / 2.0);
qw = cos(yaw_rad / 2.0);

[svc_status, svc_out] = system('bash -lc ''gz service -l 2>/dev/null | grep -E "^/world/.*/set_pose$" | head -n1''');
if svc_status ~= 0 || strlength(string(strtrim(svc_out))) == 0
    msg = 'No /world/*/set_pose service found.';
    return;
end

svc_name = strtrim(svc_out);
name_candidates = {char(string(mission_cfg.reset_model_name)), 'iris', 'iris_with_gimbal_0'};
name_candidates = unique(name_candidates, 'stable');

applied_entities = strings(0, 1);
last_err = "";
for i = 1:numel(name_candidates)
    entity_name = name_candidates{i};
    req = sprintf(['name: "%s", position: {x: %.3f, y: %.3f, z: %.3f}, ' ...
        'orientation: {x: 0.0, y: 0.0, z: %.6f, w: %.6f}'], ...
        entity_name, spawn_x, spawn_y, mission_cfg.reset_spawn_z_m, qz, qw);

    cmd = sprintf('bash -lc ''gz service -s "%s" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req ''''''%s'''''' 2>/dev/null''', ...
        svc_name, req);
    [rc, out] = system(cmd);
    out_l = lower(string(out));
    if rc == 0 && (~contains(out_l, 'false'))
        ok = true;
        applied_entities(end+1) = string(entity_name); %#ok<AGROW>
    else
        last_err = string(strtrim(out));
    end
end

if ok
    msg = sprintf('service=%s entity=[%s] spawn=[%.2f, %.2f, %.2f] yaw=%.1fdeg', ...
        svc_name, strjoin(applied_entities, ', '), spawn_x, spawn_y, mission_cfg.reset_spawn_z_m, mission_cfg.reset_spawn_yaw_deg);
else
    if strlength(last_err) == 0
        msg = sprintf('set_pose failed for candidates on %s', svc_name);
    else
        msg = sprintf('set_pose failed for candidates on %s (%s)', svc_name, char(last_err));
    end
end
end

function [ok, msg] = autlWaitForMavlinkReady(mission_cfg)
% Wait briefly at worker startup until the designated MAVLink endpoint responds.

ok = false;
msg = '';

master_conn = 'tcp:127.0.0.1:5762';
if isfield(mission_cfg, 'mavlink_master') && strlength(string(mission_cfg.mavlink_master)) > 0
    master_conn = char(string(mission_cfg.mavlink_master));
end

fallback_conn = '';
if isfield(mission_cfg, 'mavlink_master_fallback') && strlength(string(mission_cfg.mavlink_master_fallback)) > 0
    fallback_conn = char(string(mission_cfg.mavlink_master_fallback));
end

wait_timeout_s = 30.0;  % Increased from 18.0 to allow SITL JSON sensor bridge initialization
if isfield(mission_cfg, 'mavlink_ready_timeout_s')
    wait_timeout_s = max(2.0, double(mission_cfg.mavlink_ready_timeout_s));
end

poll_interval_s = 1.0;
if isfield(mission_cfg, 'mavlink_ready_poll_interval_s')
    poll_interval_s = max(0.3, double(mission_cfg.mavlink_ready_poll_interval_s));
end

ctrl_cfg = struct('mav', struct('master_connection', master_conn, 'timeout_s', mission_cfg.mavlink_control_timeout_s));
ctrl_cfg.control = struct('backend', 'mavproxy', 'allow_backend_fallback', true, 'mavros_namespace', '/mavros');
if isfield(mission_cfg, 'control_backend')
    ctrl_cfg.control.backend = char(string(mission_cfg.control_backend));
end
if isfield(mission_cfg, 'control_backend_fallback')
    ctrl_cfg.control.allow_backend_fallback = logical(mission_cfg.control_backend_fallback);
end
if isfield(mission_cfg, 'mavros_namespace')
    ctrl_cfg.control.mavros_namespace = char(string(mission_cfg.mavros_namespace));
end
if isfield(mission_cfg, 'flow_log_file')
    ctrl_cfg.flow_log_file = mission_cfg.flow_log_file;
end
if isfield(mission_cfg, 'flow_context')
    ctrl_cfg.flow_context = mission_cfg.flow_context;
end
ctrl_cfg.flow_log_all_actions = false;
start_t = tic;
attempt = 0;
last_err = "unknown";
boot_used = false;

while toc(start_t) < wait_timeout_s
    attempt = attempt + 1;
    status_res = autlMavproxyControl('status', struct(), ctrl_cfg);
    if status_res.is_success
        ok = true;
        msg = sprintf('master=%s attempts=%d elapsed=%.1fs', master_conn, attempt, toc(start_t));
        return;
    end

    last_err = autlSanitizeMavError(status_res.error_message);
    if strlength(last_err) == 0
        last_err = "heartbeat timeout";
    end

    if ~boot_used && strlength(string(fallback_conn)) > 0
        fb_cfg = struct('mav', struct('master_connection', fallback_conn, 'allow_port_fallback', false));
        if isfield(mission_cfg, 'flow_log_file')
            fb_cfg.flow_log_file = mission_cfg.flow_log_file;
        end
        if isfield(mission_cfg, 'flow_context')
            fb_cfg.flow_context = mission_cfg.flow_context;
        end
        fb_cfg.flow_log_all_actions = false;
        fb_res = autlMavproxyControl('status', struct(), fb_cfg);
        boot_used = true;
        if fb_res.is_success
            last_err = "master=" + string(last_err) + ", fallback_bootstrap=ok";
        else
            last_err = "master=" + string(last_err) + ", fallback_bootstrap=failed:" + autlSanitizeMavError(fb_res.error_message);
        end
    end

    pause(poll_interval_s);
end

msg = sprintf('timeout after %.1fs (master=%s, attempts=%d, last=%s)', ...
    wait_timeout_s, master_conn, attempt, char(string(last_err)));
end

function [ok, msg] = autlResetArduPilotState(mission_cfg)
% Force ArduPilot into a clean state between scenarios.

ok = false;
parts = strings(0, 1);

master_conn = 'tcp:127.0.0.1:5762';
if isfield(mission_cfg, 'mavlink_master') && strlength(string(mission_cfg.mavlink_master)) > 0
    master_conn = char(string(mission_cfg.mavlink_master));
end

fallback_conn = '';
if isfield(mission_cfg, 'mavlink_master_fallback') && strlength(string(mission_cfg.mavlink_master_fallback)) > 0
    fallback_conn = char(string(mission_cfg.mavlink_master_fallback));
end

mode_sequence = {'GUIDED'};
if isfield(mission_cfg, 'reset_mode_sequence') && ~isempty(mission_cfg.reset_mode_sequence)
    if iscell(mission_cfg.reset_mode_sequence)
        mode_sequence = mission_cfg.reset_mode_sequence;
    else
        mode_sequence = {char(string(mission_cfg.reset_mode_sequence))};
    end
end

ctrl_cfg = struct('mav', struct('master_connection', master_conn, 'timeout_s', mission_cfg.mavlink_control_timeout_s));
ctrl_cfg.control = struct('backend', 'mavproxy', 'allow_backend_fallback', true, 'mavros_namespace', '/mavros');
if isfield(mission_cfg, 'control_backend')
    ctrl_cfg.control.backend = char(string(mission_cfg.control_backend));
end
if isfield(mission_cfg, 'control_backend_fallback')
    ctrl_cfg.control.allow_backend_fallback = logical(mission_cfg.control_backend_fallback);
end
if isfield(mission_cfg, 'mavros_namespace')
    ctrl_cfg.control.mavros_namespace = char(string(mission_cfg.mavros_namespace));
end
if isfield(mission_cfg, 'flow_log_file')
    ctrl_cfg.flow_log_file = mission_cfg.flow_log_file;
end
if isfield(mission_cfg, 'flow_context')
    ctrl_cfg.flow_context = mission_cfg.flow_context;
end
ctrl_cfg.flow_log_all_actions = false;

% Avoid churn: when master is unreachable, skip disarm/mode calls for this scenario.
status_res = autlMavproxyControl('status', struct(), ctrl_cfg);
if ~status_res.is_success
    % Some SITL instances expose SERIAL1 only after first SERIAL0 touch.
    [boot_ok, boot_msg] = autlMaybeBootstrapMavlinkFallback(fallback_conn);
    if boot_ok
        status_res = autlMavproxyControl('status', struct(), ctrl_cfg);
    end

    if ~status_res.is_success
        emsg = autlSanitizeMavError(status_res.error_message);
        if strlength(emsg) == 0
            emsg = "unknown";
        end
        if boot_ok
            msg = char("skip_reset:mavlink_unreachable(" + emsg + "; bootstrap=" + string(boot_msg) + ")");
        else
            msg = char("skip_reset:mavlink_unreachable(" + emsg + ")");
        end
        return;
    end
end

disarm_res = autlMavproxyControl('disarm', struct(), ctrl_cfg);
if disarm_res.is_success
    parts(end+1) = "disarm=ok"; %#ok<AGROW>
else
    emsg = autlSanitizeMavError(disarm_res.error_message);
    if strlength(emsg) == 0
        emsg = "unknown";
    end
    parts(end+1) = "disarm=warn(" + emsg + ")"; %#ok<AGROW>
end

function [ok, msg] = autlMaybeBootstrapMavlinkFallback(fallback_conn)
% Perform a throttled one-shot status ping on fallback port to trigger SITL serial init.

ok = false;
msg = "not_used";
if nargin < 1 || strlength(string(fallback_conn)) == 0
    msg = "no_fallback";
    return;
end

persistent bootstrap_keys bootstrap_until_s
if isempty(bootstrap_keys)
    bootstrap_keys = {};
    bootstrap_until_s = [];
end

key = char(string(fallback_conn));
now_s = posixtime(datetime('now'));
idx = find(strcmp(bootstrap_keys, key), 1);
if ~isempty(idx) && bootstrap_until_s(idx) > now_s
    msg = "throttled";
    return;
end

cfg = struct('mav', struct('master_connection', key, 'allow_port_fallback', false));
if isfield(mission_cfg, 'flow_log_file')
    cfg.flow_log_file = mission_cfg.flow_log_file;
end
if isfield(mission_cfg, 'flow_context')
    cfg.flow_context = mission_cfg.flow_context;
end
cfg.flow_log_all_actions = false;
res = autlMavproxyControl('status', struct(), cfg);
ok = logical(res.is_success);
if ok
    msg = "ok";
else
    msg = "failed:" + autlSanitizeMavError(res.error_message);
end

if isempty(idx)
    bootstrap_keys{end+1} = key; %#ok<AGROW>
    bootstrap_until_s(end+1) = now_s + 10.0; %#ok<AGROW>
else
    bootstrap_until_s(idx) = now_s + 10.0;
end
end

mode_all_ok = true;
for mi = 1:numel(mode_sequence)
    target_mode = char(string(mode_sequence{mi}));
    mode_res = autlMavproxyControl('set_mode', struct('mode', target_mode), ctrl_cfg);
    if mode_res.is_success
        parts(end+1) = "mode=" + string(target_mode) + "(ok)"; %#ok<AGROW>
    else
        mode_all_ok = false;
        emsg = autlSanitizeMavError(mode_res.error_message);
        if strlength(emsg) == 0
            emsg = "unknown";
        end
        parts(end+1) = "mode=" + string(target_mode) + "(warn:" + emsg + ")"; %#ok<AGROW>
    end
end

ok = mode_all_ok;
msg = char(strjoin(parts, ', '));
end

function [ok, msg] = autlWaitForWorkerStartBarrier(config, worker_id, log_fid)
% Synchronize worker launch so multi-drone collection starts together.

ok = false;
msg = 'not_enabled';

if nargin < 1 || ~isstruct(config) || ~isfield(config, 'output_dir') || ~isfield(config, 'num_workers')
    return;
end

barrier_dir = fullfile(config.output_dir, '.worker_start_barrier');
if ~exist(barrier_dir, 'dir')
    mkdir(barrier_dir);
end

ready_file = fullfile(barrier_dir, sprintf('worker_%02d.ready', worker_id));
fid = fopen(ready_file, 'w');
if fid >= 0
    fprintf(fid, 'worker=%d\n', worker_id);
    fclose(fid);
end

start_t = tic;
timeout_s = 30.0;
while toc(start_t) < timeout_s
    ready_files = dir(fullfile(barrier_dir, 'worker_*.ready'));
    if numel(ready_files) >= max(1, round(config.num_workers))
        ok = true;
        msg = sprintf('workers_ready=%d/%d', numel(ready_files), config.num_workers);
        return;
    end
    pause(0.25);
end

ready_files = dir(fullfile(barrier_dir, 'worker_*.ready'));
msg = sprintf('timeout waiting for workers_ready=%d/%d', numel(ready_files), config.num_workers);
if nargin >= 3 && ~isempty(log_fid)
    fprintf(log_fid, '[Barrier] %s\n', msg);
end
end

function emsg = autlSanitizeMavError(raw_msg)
% Collapse multiline pymavlink errors into compact one-line text.

emsg = strtrim(string(raw_msg));
if strlength(emsg) == 0
    return;
end

parts = regexp(char(emsg), '\\r?\\n', 'split');
parts = parts(~cellfun('isempty', strtrim(parts)));
if isempty(parts)
    emsg = "";
    return;
end

keep = strings(0, 1);
for i = 1:numel(parts)
    p = strtrim(string(parts{i}));
    if strlength(p) == 0
        continue;
    end
    p_low = lower(p);
    if contains(p_low, "eof") && contains(p_low, "tcp socket")
        continue;
    end
    if contains(p_low, "connection refused sleeping")
        continue;
    end
    if contains(p_low, "connection reset by peer")
        continue;
    end
    if contains(p_low, "socket closed")
        continue;
    end
    keep(end+1) = p; %#ok<AGROW>
end

if isempty(keep)
    emsg = "heartbeat timeout";
else
    emsg = keep(end);
end
end

