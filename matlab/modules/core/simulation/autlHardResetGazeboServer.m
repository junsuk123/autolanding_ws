function [success, message] = autlHardResetGazeboServer(root_dir, mission_config, log_prefix)
% autlHardResetGazeboServer
% Safely hard-reset Gazebo server by terminating existing Gazebo processes
% and starting a fresh Gazebo instance.

if nargin < 1 || strlength(string(root_dir)) == 0
    this_file = mfilename('fullpath');
    root_dir = fileparts(fileparts(fileparts(fileparts(fileparts(fileparts(this_file))))));
end
if nargin < 2 || isempty(mission_config)
    mission_config = struct();
end
if nargin < 3
    log_prefix = '';
end

success = false;
message = '';

launcher_py = fullfile(root_dir, 'scripts', 'launch_gazebo_py.py');
if exist(launcher_py, 'file') ~= 2
    message = sprintf('Missing Gazebo launcher: %s', launcher_py);
    fprintf('%s[AutoLandingDataCollection] Warning: %s\n', log_prefix, message);
    return;
end

if isfield(mission_config, 'gazebo_world_file') && strlength(string(mission_config.gazebo_world_file)) > 0
    world_file = char(string(mission_config.gazebo_world_file));
else
    world_file = '/tmp/iris_runway_aruco_landing.sdf';
end

if isfield(mission_config, 'gazebo_hard_reset_wait_s') && isfinite(mission_config.gazebo_hard_reset_wait_s)
    wait_s = max(5.0, double(mission_config.gazebo_hard_reset_wait_s));
else
    wait_s = 25.0;
end

restart_gui = false;
if isfield(mission_config, 'gazebo_server_mode')
    restart_gui = ~logical(mission_config.gazebo_server_mode);
end
if isfield(mission_config, 'gazebo_hard_reset_restart_gui')
    restart_gui = logical(mission_config.gazebo_hard_reset_restart_gui);
end
if strcmp(getenv('AUTOLANDING_FORCE_GUI'), '1')
    restart_gui = true;
end
if strcmp(getenv('AUTOLANDING_FORCE_HEADLESS'), '1')
    restart_gui = false;
end

py_exec = 'python3';
venv_py = fullfile(root_dir, '.venv', 'bin', 'python');
if exist(venv_py, 'file') == 2
    py_exec = venv_py;
end

gz_log = fullfile(root_dir, 'data', 'processed', 'verify_gz_server.log');

fprintf('%s[AutoLandingDataCollection] Gazebo hard reset: stopping existing Gazebo processes...\n', log_prefix);
kill_core = [ ...
    'set +e; ' ...
    'pkill -TERM -f "gz sim" >/dev/null 2>&1 || true; ' ...
    'pkill -TERM -f "gzserver" >/dev/null 2>&1 || true; ' ...
    'pkill -TERM -f "gzclient" >/dev/null 2>&1 || true; ' ...
    'pkill -TERM -f "ign gazebo" >/dev/null 2>&1 || true; ' ...
    'pkill -TERM -f "gazebo " >/dev/null 2>&1 || true; ' ...
    'sleep 2; ' ...
    'pkill -KILL -f "gz sim" >/dev/null 2>&1 || true; ' ...
    'pkill -KILL -f "gzserver" >/dev/null 2>&1 || true; ' ...
    'pkill -KILL -f "gzclient" >/dev/null 2>&1 || true; ' ...
    'pkill -KILL -f "ign gazebo" >/dev/null 2>&1 || true; ' ...
    'pkill -KILL -f "gazebo " >/dev/null 2>&1 || true; ' ...
    'sleep 1' ...
];
kill_cmd = sprintf('bash -lc "%s"', localEscapeDoubleQuotes(kill_core));
[~, ~] = system(kill_cmd);

fprintf('%s[AutoLandingDataCollection] Gazebo hard reset: launching fresh server (%s)...\n', ...
    log_prefix, ternary(restart_gui, 'GUI', 'headless'));

if restart_gui
    mode_args = '--gui';
else
    mode_args = '--headless';
end

launch_core = sprintf([ ...
    'nohup setsid "%s" "%s" --world "%s" %s --verbose ' ...
    '> "%s" 2>&1 < /dev/null &' ...
    ], py_exec, launcher_py, world_file, mode_args, gz_log);
launch_cmd = sprintf('bash -lc "%s"', localEscapeDoubleQuotes(launch_core));
[launch_rc, launch_out] = system(launch_cmd);
if launch_rc ~= 0
    message = sprintf('Failed to relaunch Gazebo (rc=%d): %s', launch_rc, strtrim(launch_out));
    fprintf('%s[AutoLandingDataCollection] Warning: %s\n', log_prefix, message);
    return;
end

start_t = tic;
while toc(start_t) < wait_s
    [alive_rc, ~] = system('bash -lc "pgrep -f \"gz sim\" >/dev/null 2>&1"');
    if alive_rc == 0
        success = true;
        message = sprintf('Gazebo hard reset completed (%s mode)', ternary(restart_gui, 'GUI', 'headless'));
        fprintf('%s[AutoLandingDataCollection] %s\n', log_prefix, message);
        return;
    end
    pause(0.5);
end

message = sprintf('Gazebo hard reset timed out after %.1f s', wait_s);
fprintf('%s[AutoLandingDataCollection] Warning: %s\n', log_prefix, message);
end

function out = localEscapeDoubleQuotes(in)
out = strrep(char(in), '"', '\\"');
end

function out = ternary(cond, true_text, false_text)
if cond
    out = true_text;
else
    out = false_text;
end
end
