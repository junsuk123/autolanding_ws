function summary = AutoLandingMain(varargin)
% AUTOLANDINGMAIN
% MATLAB entrypoint for standard workspace execution.

  rootDir = fileparts(fileparts(mfilename('fullpath')));

  % Pass through non-struct invocations (e.g. AutoLandingMain('gui')).
  if nargin >= 1 && ~isstruct(varargin{1})
    if (ischar(varargin{1}) || isstring(varargin{1}))
      mode = lower(strtrim(char(string(varargin{1}))));
      if any(strcmp(mode, {'stop', 'cleanup', 'kill'}))
        summary = localStopSimulationStack(rootDir);
        return;
      end
      if any(strcmp(mode, {'gui', 'headless', 'server'}))
        localCleanupSimulationStack(rootDir);
        summary = localStartSimulationStack(any(strcmp(mode, {'headless', 'server'})));
        return;
      end
    end
    summary = AutoLandingMainFull(varargin{:});
    return;
  end

  defaults = struct( ...
      'mode', 'full', ...
      'num_drones', 1, ...
      'num_landing_pads', 1, ...
      'scenarios_per_worker', 20, ...
      'train_ratio', 0.8, ...
      'val_ratio', 0.2, ...
      'headless', false, ...
      'auto_flight_demo', true, ...
    'drone_spawn_above_pad_m', 0.25, ...
    'landing_pad_size_xy', 5.4, ...
    'initial_spawn_x_m', 1.0, ...
    'initial_spawn_y_m', 0.35, ...
    'initial_spawn_z_m', 0.15, ...
    'demo_takeoff_alt_m', 1.2, ...
    'control_backend', 'mavproxy');

  if nargin >= 1 && isstruct(varargin{1})
    user_overrides = varargin{1};
    names = fieldnames(user_overrides);
    for i = 1:numel(names)
      defaults.(names{i}) = user_overrides.(names{i});
    end
  end

  cleanup_guard = onCleanup(@() localCleanupSimulationStack(rootDir)); %#ok<NASGU>
  summary = AutoLandingMainFull(defaults);
end

function summary = localStopSimulationStack(rootDir)
% Stop Gazebo/SITL/MAVROS stack launched by AutoLandingMain/AutoLandingMainFull.

  localCleanupSimulationStack(rootDir);
  summary = struct('ok', true, 'message', 'Simulation stack stop command executed.');
  fprintf('[AutoLandingMain] %s\n', summary.message);
end

function summary = localStartSimulationStack(force_headless)
% Start only Gazebo/SITL/MAVROS stack without running mission/pipeline.

  rootDir = fileparts(fileparts(mfilename('fullpath')));
  launcher = fullfile(rootDir, 'scripts', 'verify_gz_ardupilot_stack.sh');
  if ~isfile(launcher)
    error('[AutoLandingMain] Missing simulator launcher: %s', launcher);
  end

  spawn_env = 'AUTOLANDING_INITIAL_SPAWN_X_M=1.0 AUTOLANDING_INITIAL_SPAWN_Y_M=0.35 AUTOLANDING_INITIAL_SPAWN_Z_M=0.15';
  if force_headless
    cmd = sprintf('%s AUTOLANDING_FORCE_HEADLESS=1 bash "%s"', spawn_env, launcher);
  else
    cmd = sprintf('%s AUTOLANDING_FORCE_GUI=1 AUTOLANDING_FORCE_HEADLESS=0 bash "%s"', spawn_env, launcher);
  end

  [status, output] = system(cmd);
  summary = struct('ok', status == 0, 'message', strtrim(output));
  if summary.ok
    fprintf('[AutoLandingMain] Simulation stack started (%s).\n', ternary(force_headless, 'headless', 'gui'));
  else
    error('[AutoLandingMain] Failed to start simulation stack (exit=%d): %s', status, strtrim(output));
  end
end

function localCleanupSimulationStack(rootDir)
cleanup_script = fullfile(rootDir, 'scripts', 'cleanup_utils.sh');
if ~isfile(cleanup_script)
  return;
end

cmd = sprintf('bash -lc ''set +e; source "%s" >/dev/null 2>&1; cleanup_all_processes >/dev/null 2>&1; exit 0''', cleanup_script);
system(cmd);
end

function out = ternary(cond, a, b)
if cond
  out = a;
else
  out = b;
end
end
