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
log_fid = fopen(worker_log_file, 'a');

fprintf(log_fid, '[Worker %d] Started at %s\n', worker_id, datetime('now'));
fprintf(log_fid, '[Worker %d] Config: scenarios=%d, rate=%d Hz, duration=%.0f sec\n', ...
    worker_id, config.scenarios_per_worker, config.sample_rate, config.scenario_duration);

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
            mission_cfg.mavlink_master = 'tcp:127.0.0.1:5762';
        end
        if isfield(mission_cfg, 'enable_auto_motion') && mission_cfg.enable_auto_motion && worker_id ~= 1
            if strcmp(char(string(mission_cfg.mavlink_master)), 'tcp:127.0.0.1:5762')
                mission_cfg.enable_auto_motion = false;
                fprintf(log_fid, '[Worker %d] Auto motion disabled: shared MAVLink master (%s).\n', worker_id, char(string(mission_cfg.mavlink_master)));
            else
                fprintf(log_fid, '[Worker %d] Auto motion enabled with dedicated MAVLink master (%s).\n', worker_id, char(string(mission_cfg.mavlink_master)));
            end
        end
        if isfield(mission_cfg, 'worker_state_tag')
            fprintf(log_fid, '[Worker %d] Worker state profile: %s\n', worker_id, char(string(mission_cfg.worker_state_tag)));
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
        end

        if mission_cfg.reset_ardupilot_each_scenario
            [ap_ok, ap_msg] = autlResetArduPilotState(mission_cfg);
            if ap_ok
                fprintf('[Worker %d] Scenario %d ArduPilot reset applied: %s\n', worker_id, scenario_num, ap_msg);
                fprintf(log_fid, '[Worker %d] Scenario %d ArduPilot reset applied: %s\n', worker_id, scenario_num, ap_msg);
            else
                fprintf('[Worker %d] Scenario %d ArduPilot reset warning: %s\n', worker_id, scenario_num, ap_msg);
                fprintf(log_fid, '[Worker %d] Scenario %d ArduPilot reset warning: %s\n', worker_id, scenario_num, ap_msg);
            end
        end

        if mission_cfg.reset_pose_each_scenario || mission_cfg.reset_ardupilot_each_scenario
            pause(0.5);
        end
        
        % Raw data collection (NO ontology processing)
        try
            fprintf(log_fid, '[Worker %d] Scenario %d: Calling autlRunDataCollection...\n', worker_id, scenario_num);
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
            fprintf('[Worker %d] Scenario %d: SUCCESS - %d samples in %.1f sec\n', ...
                worker_id, scenario_num, samples, duration);
            fprintf(log_fid, '[Worker %d] Scenario %d collected: %d samples in %.1f sec\n', ...
                worker_id, scenario_num, samples, duration);
            scenario_success_count = scenario_success_count + 1;
            
        catch ME
            if autlWorkerIsUserInterrupt(ME)
                rethrow(ME);
            end
            fprintf('[Worker %d] Scenario %d FAILED: %s\n', worker_id, scenario_num, ME.message);
            fprintf(log_fid, '[Worker %d] Scenario %d FAILED: %s\n', worker_id, scenario_num, ME.message);
            fprintf(log_fid, '%s\n', getReport(ME));
            scenario_fail_count = scenario_fail_count + 1;
        end
        
        % Brief delay between scenarios
        pause(1);
    end
    
    fprintf('[Worker %d] Collection complete. success=%d, failed=%d, total=%d\n', ...
        worker_id, scenario_success_count, scenario_fail_count, config.scenarios_per_worker);
    fprintf(log_fid, '[Worker %d] Collection complete at %s (success=%d, failed=%d)\n', ...
        worker_id, datetime('now'), scenario_success_count, scenario_fail_count);

    if scenario_fail_count > 0
        error('AutoLanding:WorkerScenarioFailed', 'Worker %d had %d failed scenario(s).', worker_id, scenario_fail_count);
    end
    
catch ME
    % Check if user interrupted
    if autlWorkerIsUserInterrupt(ME)
        fprintf(log_fid, '[Worker %d] User interrupted. Saved data recovered.\n', worker_id);
        fprintf('[Worker %d] INTERRUPTED - partial results saved.\n', worker_id);
        rethrow(ME);
    else
        fprintf(log_fid, '[Worker %d] CRITICAL ERROR: %s\n', worker_id, ME.message);
        fprintf(log_fid, '%s\n', getReport(ME));
        rethrow(ME);
    end
end

fclose(log_fid);
fprintf('[Worker %d] Exiting.\n', worker_id);
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
    mission_cfg.spawn_radius_m = 0.8;
end
if ~isfield(mission_cfg, 'spawn_angle_deg')
    mission_cfg.spawn_angle_deg = 35.0;
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

function [ok, msg] = autlResetArduPilotState(mission_cfg)
% Force ArduPilot into a clean state between scenarios.

ok = false;
parts = strings(0, 1);

master_conn = 'tcp:127.0.0.1:5762';
if isfield(mission_cfg, 'mavlink_master') && strlength(string(mission_cfg.mavlink_master)) > 0
    master_conn = char(string(mission_cfg.mavlink_master));
end

mode_sequence = {'GUIDED'};
if isfield(mission_cfg, 'reset_mode_sequence') && ~isempty(mission_cfg.reset_mode_sequence)
    if iscell(mission_cfg.reset_mode_sequence)
        mode_sequence = mission_cfg.reset_mode_sequence;
    else
        mode_sequence = {char(string(mission_cfg.reset_mode_sequence))};
    end
end

ctrl_cfg = struct('mav', struct('master_connection', master_conn));

disarm_res = autlMavproxyControl('disarm', struct(), ctrl_cfg);
if disarm_res.is_success
    parts(end+1) = "disarm=ok"; %#ok<AGROW>
else
    emsg = strtrim(string(disarm_res.error_message));
    if strlength(emsg) == 0
        emsg = "unknown";
    end
    parts(end+1) = "disarm=warn(" + emsg + ")"; %#ok<AGROW>
end

mode_all_ok = true;
for mi = 1:numel(mode_sequence)
    target_mode = char(string(mode_sequence{mi}));
    mode_res = autlMavproxyControl('set_mode', struct('mode', target_mode), ctrl_cfg);
    if mode_res.is_success
        parts(end+1) = "mode=" + string(target_mode) + "(ok)"; %#ok<AGROW>
    else
        mode_all_ok = false;
        emsg = strtrim(string(mode_res.error_message));
        if strlength(emsg) == 0
            emsg = "unknown";
        end
        parts(end+1) = "mode=" + string(target_mode) + "(warn:" + emsg + ")"; %#ok<AGROW>
    end
end

ok = mode_all_ok;
msg = char(strjoin(parts, ', '));
end

