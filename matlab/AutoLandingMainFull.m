function summary = AutoLandingMainFull(varargin)
% AUTOLANDINGMAINFULL
% MATLAB entrypoint for the full AutoLanding workspace orchestration.
%
% Quick configuration examples:
%   summary = AutoLandingMainFull();
%   summary = AutoLandingMainFull(struct( ...
%       'mode', 'full', ...
%       'num_drones', 2, ...
%       'num_landing_pads', 2, ...
%       'scenarios_per_worker', 12, ...
%       'train_ratio', 0.8, ...
%       'val_ratio', 0.2, ...
%       'headless', false, ...
%       'auto_flight_demo', true, ...
%       'drone_spawn_above_pad_m', 0.35, ...
%       'landing_pad_size_xy', 2.4, ...
%       'initial_spawn_x_m', 1.0, ...
%       'initial_spawn_y_m', 0.35, ...
%       'initial_spawn_z_m', 0.15, ...
%       'demo_takeoff_alt_m', 1.2, ...
%       'control_backend', 'mavros'));
%
% Legacy positional call is still supported:
%   summary = AutoLandingMainFull('full', 1, 10);

rootDir = fileparts(fileparts(mfilename('fullpath')));
modDir = fullfile(rootDir, 'matlab', 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

ros_domain_id_value = strtrim(getenv('ROS_DOMAIN_ID'));
if isempty(ros_domain_id_value) || isempty(regexp(ros_domain_id_value, '^[0-9]+$', 'once'))
    setenv('ROS_DOMAIN_ID', '0');
end
run_params = localResolveRunParams(varargin{:});
mode = localResolveMode(run_params);
if localNeedsSimulation(mode)
    cleanup_guard = onCleanup(@() localCleanupSimulationStack(rootDir)); %#ok<NASGU>
    localCleanupSimulationStack(rootDir);
    localEnsureSimulationStack(rootDir, run_params);
end

if isfield(run_params, 'num_landing_pads') && isfinite(run_params.num_landing_pads) && run_params.num_landing_pads > 1
    fprintf('[AutoLandingMainFull] Warning: num_landing_pads=%d requested, but current world generator supports one active landing pad model.\n', ...
        round(run_params.num_landing_pads));
end

summary = autlRunWorkspacePipeline(rootDir, run_params);
end

function params = localResolveRunParams(varargin)
params = localDefaultRunParams();

if isempty(varargin)
    return;
end

if numel(varargin) == 1 && isstruct(varargin{1})
    params = localMergeParams(params, varargin{1});
    return;
end

% Backward compatibility: AutoLandingMainFull('full', workers, scenarios)
if ~isempty(varargin) && (ischar(varargin{1}) || isstring(varargin{1}))
    params.mode = lower(strtrim(char(string(varargin{1}))));
    params = localMarkExplicit(params, 'mode');
    if numel(varargin) >= 2
        params.num_drones = double(varargin{2});
        params = localMarkExplicit(params, 'num_drones');
    end
    if numel(varargin) >= 3
        params.scenarios_per_worker = double(varargin{3});
        params = localMarkExplicit(params, 'scenarios_per_worker');
    end
    return;
end

% Name-value style: AutoLandingMainFull('mode','full','num_drones',2,...)
if mod(numel(varargin), 2) ~= 0
    error('[AutoLandingMainFull] Name-value arguments must be provided in pairs.');
end

for k = 1:2:numel(varargin)
    key = lower(strtrim(char(string(varargin{k}))));
    val = varargin{k + 1};
    switch key
        case {'mode'}
            params.mode = lower(strtrim(char(string(val))));
            params = localMarkExplicit(params, 'mode');
        case {'num_drones', 'workers'}
            params.num_drones = double(val);
            params = localMarkExplicit(params, 'num_drones');
        case {'num_landing_pads', 'landing_pad_count'}
            params.num_landing_pads = double(val);
            params = localMarkExplicit(params, 'num_landing_pads');
        case {'scenarios_per_worker', 'scenarios'}
            params.scenarios_per_worker = double(val);
            params = localMarkExplicit(params, 'scenarios_per_worker');
        case {'train_ratio'}
            params.train_ratio = double(val);
            params = localMarkExplicit(params, 'train_ratio');
        case {'val_ratio'}
            params.val_ratio = double(val);
            params = localMarkExplicit(params, 'val_ratio');
        case {'headless'}
            params.headless = logical(val);
            params = localMarkExplicit(params, 'headless');
        case {'enable_visualization'}
            params.enable_visualization = logical(val);
            params = localMarkExplicit(params, 'enable_visualization');
        case {'auto_flight_demo'}
            params.auto_flight_demo = logical(val);
            params = localMarkExplicit(params, 'auto_flight_demo');
        case {'drone_spawn_above_pad_m'}
            params.drone_spawn_above_pad_m = double(val);
            params = localMarkExplicit(params, 'drone_spawn_above_pad_m');
        case {'landing_pad_size_xy'}
            params.landing_pad_size_xy = double(val);
            params = localMarkExplicit(params, 'landing_pad_size_xy');
        case {'initial_spawn_x_m'}
            params.initial_spawn_x_m = double(val);
            params = localMarkExplicit(params, 'initial_spawn_x_m');
        case {'initial_spawn_y_m'}
            params.initial_spawn_y_m = double(val);
            params = localMarkExplicit(params, 'initial_spawn_y_m');
        case {'initial_spawn_z_m'}
            params.initial_spawn_z_m = double(val);
            params = localMarkExplicit(params, 'initial_spawn_z_m');
        case {'demo_takeoff_alt_m'}
            params.demo_takeoff_alt_m = double(val);
            params = localMarkExplicit(params, 'demo_takeoff_alt_m');
        case {'control_backend'}
            params.control_backend = lower(strtrim(char(string(val))));
            params = localMarkExplicit(params, 'control_backend');
        case {'reset_spawn_z_m'}
            params.reset_spawn_z_m = double(val);
            params = localMarkExplicit(params, 'reset_spawn_z_m');
        case {'mavlink_precheck_timeout_s'}
            params.mavlink_precheck_timeout_s = double(val);
            params = localMarkExplicit(params, 'mavlink_precheck_timeout_s');
        otherwise
            error('[AutoLandingMainFull] Unsupported parameter key: %s', key);
    end
end
end

function mode = localResolveMode(params)
mode = "full";
if isstruct(params) && isfield(params, 'mode')
    candidate = string(params.mode);
    if strlength(candidate) > 0
        mode = lower(strtrim(candidate));
    end
end
end

function params = localDefaultRunParams()
params = struct();
params.explicit_fields__ = {};
params.mode = 'full';
params.num_drones = 1;
params.num_landing_pads = 1;
params.scenarios_per_worker = 10;
params.train_ratio = 0.8;
params.val_ratio = 0.2;
params.headless = false;
params.enable_visualization = true;
params.auto_flight_demo = false;
params.drone_spawn_above_pad_m = 0.35;
params.landing_pad_size_xy = 2.4;
params.initial_spawn_x_m = 1.0;
params.initial_spawn_y_m = 0.35;
params.initial_spawn_z_m = 0.15;
params.demo_takeoff_alt_m = 1.2;
params.control_backend = 'mavros';
params.mavlink_precheck_timeout_s = 90.0;
end

function out = localMergeParams(base, overrides)
out = base;
if ~isstruct(overrides)
    return;
end
names = fieldnames(overrides);
if isfield(overrides, 'explicit_fields__')
    names = setdiff(names, {'explicit_fields__'});
end
for i = 1:numel(names)
    out.(names{i}) = overrides.(names{i});
end
if ~isfield(out, 'explicit_fields__') || ~iscell(out.explicit_fields__)
    out.explicit_fields__ = {};
end
out.explicit_fields__ = unique([out.explicit_fields__(:); names(:)]);
if isfield(overrides, 'explicit_fields__') && iscell(overrides.explicit_fields__)
    out.explicit_fields__ = unique([out.explicit_fields__(:); overrides.explicit_fields__(:)]);
end
end

function params = localMarkExplicit(params, field_name)
if ~isfield(params, 'explicit_fields__') || ~iscell(params.explicit_fields__)
    params.explicit_fields__ = {};
end
params.explicit_fields__ = unique([params.explicit_fields__(:); {char(field_name)}]);
end

function tf = localNeedsSimulation(mode)
tf = any(strcmp(mode, {"full", "collect", "collect_parallel", "mission", "sim", "gui", "server", "headless"}));
end

function localEnsureSimulationStack(rootDir, run_params)
launcher = fullfile(rootDir, 'scripts', 'verify_gz_ardupilot_stack.sh');
if ~isfile(launcher)
    error('[AutoLandingMainFull] Missing simulator launcher: %s', launcher);
end

fprintf('[AutoLandingMainFull] Preparing Gazebo/SITL stack...\n');
localPreKillSimulationProcesses();
headless_explicit = localIsExplicitRunField(run_params, 'headless');
force_headless = false;
if nargin >= 2 && isstruct(run_params) && isfield(run_params, 'headless')
    force_headless = logical(run_params.headless);
end

% Default behavior should allow launcher auto-fallback (GUI -> headless)
% unless caller explicitly requested headless/gui.
if headless_explicit && force_headless
    cmd = sprintf(['AUTOLANDING_INITIAL_SPAWN_X_M=%.3f AUTOLANDING_INITIAL_SPAWN_Y_M=%.3f ' ...
        'AUTOLANDING_INITIAL_SPAWN_Z_M=%.3f AUTOLANDING_FORCE_HEADLESS=1 bash "%s"'], ...
        double(run_params.initial_spawn_x_m), double(run_params.initial_spawn_y_m), ...
        double(run_params.initial_spawn_z_m), launcher);
elseif headless_explicit && ~force_headless
    cmd = sprintf(['AUTOLANDING_INITIAL_SPAWN_X_M=%.3f AUTOLANDING_INITIAL_SPAWN_Y_M=%.3f ' ...
        'AUTOLANDING_INITIAL_SPAWN_Z_M=%.3f AUTOLANDING_FORCE_GUI=1 AUTOLANDING_FORCE_HEADLESS=0 bash "%s"'], ...
        double(run_params.initial_spawn_x_m), double(run_params.initial_spawn_y_m), ...
        double(run_params.initial_spawn_z_m), launcher);
else
    cmd = sprintf(['AUTOLANDING_INITIAL_SPAWN_X_M=%.3f AUTOLANDING_INITIAL_SPAWN_Y_M=%.3f ' ...
        'AUTOLANDING_INITIAL_SPAWN_Z_M=%.3f bash "%s"'], ...
        double(run_params.initial_spawn_x_m), double(run_params.initial_spawn_y_m), ...
        double(run_params.initial_spawn_z_m), launcher);
end
[status, output] = system(cmd);
if status ~= 0
    error('[AutoLandingMainFull] Failed to prepare Gazebo/SITL stack (exit=%d): %s', status, strtrim(output));
end
fprintf('[AutoLandingMainFull] Gazebo/SITL stack is ready.\n');
end

function tf = localIsExplicitRunField(run_params, field_name)
tf = false;
if nargin < 2 || ~isstruct(run_params)
    return;
end
if ~isfield(run_params, 'explicit_fields__') || ~iscell(run_params.explicit_fields__)
    return;
end
target = char(string(field_name));
fields = cellfun(@char, run_params.explicit_fields__, 'UniformOutput', false);
tf = any(strcmp(fields, target));
end

function localPreKillSimulationProcesses()
% Extra safety: terminate stale stack processes before launcher cleanup runs.

localCleanupSimulationStack(fileparts(fileparts(mfilename('fullpath'))));
end

function localCleanupSimulationStack(rootDir)
cleanup_script = fullfile(rootDir, 'scripts', 'cleanup_utils.sh');
if ~isfile(cleanup_script)
    return;
end

cmd = sprintf('bash -lc ''set +e; source "%s" >/dev/null 2>&1; cleanup_all_processes >/dev/null 2>&1; exit 0''', cleanup_script);
system(cmd);
end
