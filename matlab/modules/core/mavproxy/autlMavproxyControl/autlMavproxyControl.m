function result = autlMavproxyControl(action, params, cfg)
% AUTLMAVPROXYCONTROL Control drone via MAVProxy
%
%   result = autlMavproxyControl(action, params, cfg)
%
% Inputs:
%   action     : 'arm', 'disarm', 'takeoff', 'land', 'set_velocity', 'status', 'set_mode'
%   params     : struct with action-specific parameters
%              - takeoff: params.height (meters)
%              - set_velocity: params.vx, params.vy, params.vz (m/s), params.duration (sec)
%              - set_mode: params.mode (e.g., 'GUIDED', 'STABILIZE', 'LAND')
%   cfg        : config struct with mav.master_connection if applicable
%
% Outputs:
%   result     : struct with status, output, error_message

    result = struct();
    result.status = 'unknown';
    result.output = '';
    result.error_message = '';
    result.is_success = false;
    
    % Default MAVProxy connection
    if ~isfield(cfg, 'mav') || ~isfield(cfg.mav, 'master_connection')
        % Use SERIAL1 to avoid churning the primary SERIAL0 link.
        master_conn = 'tcp:127.0.0.1:5762';
    else
        master_conn = cfg.mav.master_connection;
    end
    
    % Timeout for control operation (seconds)
    timeout = 4;
    if isfield(cfg, 'mav') && isfield(cfg.mav, 'timeout_s') && ~isempty(cfg.mav.timeout_s)
        timeout = max(0.5, min(10.0, double(cfg.mav.timeout_s)));
    end

    flow_log_file = '';
    flow_ctx = struct();
    flow_log_all_actions = false;
    if isfield(cfg, 'flow_log_file')
        flow_log_file = char(string(cfg.flow_log_file));
    end
    if isfield(cfg, 'flow_context') && isstruct(cfg.flow_context)
        flow_ctx = cfg.flow_context;
    end
    if isfield(cfg, 'flow_log_all_actions')
        flow_log_all_actions = logical(cfg.flow_log_all_actions);
    end

    if flow_log_all_actions
        autlFlowLog(flow_log_file, 'autlMavproxyControl', 'action_started', autlFlowMerge(flow_ctx, struct( ...
            'action', char(string(action)), 'master', char(string(master_conn)))));
    end

    control_backend = 'mavproxy';
    allow_backend_fallback = true;
    prefer_mavros_only = false;
    if isfield(cfg, 'control') && isstruct(cfg.control)
        if isfield(cfg.control, 'backend') && strlength(string(cfg.control.backend)) > 0
            control_backend = lower(char(string(cfg.control.backend)));
        end
        if isfield(cfg.control, 'allow_backend_fallback')
            allow_backend_fallback = logical(cfg.control.allow_backend_fallback);
        end
        if isfield(cfg.control, 'prefer_mavros_only')
            prefer_mavros_only = logical(cfg.control.prefer_mavros_only);
        end
    end

    if strcmp(control_backend, 'mavros') && ~prefer_mavros_only
        % In worker MAVROS mode, keep a single control path to avoid
        % repeatedly touching tcp:57xx probes that can interfere with FCU link stability.
        prefer_mavros_only = true;
    end
    if prefer_mavros_only
        allow_backend_fallback = false;
    end

    if strcmp(control_backend, 'mavros')
        [mavros_result, mavros_ok] = autlRunMavrosControl(action, params, cfg, timeout);
        if mavros_ok
            result = mavros_result;
            should_log_result = flow_log_all_actions || ~result.is_success;
            if should_log_result
                autlFlowLog(flow_log_file, 'autlMavproxyControl', 'action_result', autlFlowMerge(flow_ctx, struct( ...
                    'action', char(string(action)), 'status', char(string(result.status)), ...
                    'is_success', logical(result.is_success), 'error', char(string(result.error_message)))));
            end
            return;
        end
        if ~allow_backend_fallback
            result = mavros_result;
            should_log_result = flow_log_all_actions || ~result.is_success;
            if should_log_result
                autlFlowLog(flow_log_file, 'autlMavproxyControl', 'action_result', autlFlowMerge(flow_ctx, struct( ...
                    'action', char(string(action)), 'status', char(string(result.status)), ...
                    'is_success', logical(result.is_success), 'error', char(string(result.error_message)))));
            end
            return;
        end
    end

    % Default to a single designated endpoint per worker to avoid
    % cross-port churn (SERIAL0<->SERIAL1) that causes EOF floods.
    allow_port_fallback = false;
    if isfield(cfg, 'mav') && isfield(cfg.mav, 'allow_port_fallback')
        allow_port_fallback = logical(cfg.mav.allow_port_fallback);
    end

    master_candidates = {master_conn};
    if allow_port_fallback
        if contains(master_conn, ':5762')
            master_candidates{end+1} = strrep(master_conn, ':5762', ':5760');
        elseif contains(master_conn, ':5760')
            master_candidates{end+1} = strrep(master_conn, ':5760', ':5762');
        end
        master_candidates = unique(master_candidates, 'stable');
    end

    function [ok, out, err] = run_pymav(action_name, params_struct)
        if nargin < 2 || isempty(params_struct)
            params_struct = struct();
        end
        py_template = [ ...
            'import base64,json,sys,time' newline ...
            'import contextlib,io' newline ...
            'from pymavlink import mavutil' newline ...
            'try:' newline ...
            '  d = json.loads(base64.b64decode("__PAYLOAD_B64__").decode())' newline ...
            '  a = d["action"]; p = d["params"]; t = float(d["timeout"])' newline ...
            '  _mav_stdout = io.StringIO()' newline ...
            '  _mav_stderr = io.StringIO()' newline ...
            '  with contextlib.redirect_stdout(_mav_stdout), contextlib.redirect_stderr(_mav_stderr):' newline ...
            '    m = mavutil.mavlink_connection(d["master"], source_system=255, autoreconnect=False)' newline ...
            '    hb = m.wait_heartbeat(timeout=max(0.8, t))' newline ...
            '  if hb is None: raise RuntimeError("heartbeat timeout")' newline ...
            '  def set_mode(mode_name):' newline ...
            '    mm = m.mode_mapping() or {}' newline ...
            '    if mode_name not in mm: raise RuntimeError(f"mode not supported: {mode_name}")' newline ...
            '    m.set_mode(mm[mode_name]); time.sleep(0.5)' newline ...
            '  def set_param(name, val):' newline ...
            '    m.mav.param_set_send(m.target_system, m.target_component, name.encode("ascii"), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)' newline ...
            '    time.sleep(0.2)' newline ...
            '  def wait_arm_state(want_armed):' newline ...
            '    end_t = time.time() + max(t, 5.0)' newline ...
            '    while time.time() < end_t:' newline ...
            '      m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)' newline ...
            '      armed = bool(m.motors_armed())' newline ...
            '      if armed == want_armed:' newline ...
            '        return True' newline ...
            '    return False' newline ...
            '  def force_arm(want_armed):' newline ...
            '    arm_cmd = 1.0 if want_armed else 0.0' newline ...
            '    m.mav.command_long_send(m.target_system,m.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,arm_cmd,21196,0,0,0,0,0)' newline ...
            '    time.sleep(0.4)' newline ...
            '  if a == "set_mode":' newline ...
            '    set_mode(str(p.get("mode","GUIDED")))' newline ...
            '  elif a == "arm":' newline ...
            '    set_param("ARMING_CHECK", 0)' newline ...
            '    set_param("BRD_SAFETYENABLE", 0)' newline ...
            '    m.arducopter_arm()' newline ...
            '    if not wait_arm_state(True):' newline ...
            '      force_arm(True)' newline ...
            '      if not wait_arm_state(True):' newline ...
            '        raise RuntimeError("arm failed")' newline ...
            '  elif a == "disarm":' newline ...
            '    m.arducopter_disarm()' newline ...
            '    if not wait_arm_state(False):' newline ...
            '      force_arm(False)' newline ...
            '      if not wait_arm_state(False):' newline ...
            '        raise RuntimeError("disarm failed")' newline ...
            '  elif a == "takeoff":' newline ...
            '    h = float(p.get("height",3.0))' newline ...
            '    set_mode("GUIDED")' newline ...
            '    set_param("ARMING_CHECK", 0)' newline ...
            '    set_param("BRD_SAFETYENABLE", 0)' newline ...
            '    if not m.motors_armed():' newline ...
            '      m.arducopter_arm()' newline ...
            '      if not wait_arm_state(True):' newline ...
            '        force_arm(True)' newline ...
            '        if not wait_arm_state(True):' newline ...
            '          raise RuntimeError("takeoff aborted: cannot arm")' newline ...
            '    m.mav.command_long_send(m.target_system,m.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,h)' newline ...
            '    time.sleep(0.5)' newline ...
            '  elif a == "land":' newline ...
            '    set_mode("LAND")' newline ...
            '  elif a == "set_velocity":' newline ...
            '    vx=float(p.get("vx",0.0)); vy=float(p.get("vy",0.0)); vz=float(p.get("vz",0.0))' newline ...
            '    mask = 3527' newline ...
            '    m.mav.set_position_target_local_ned_send(0,m.target_system,m.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,mask,0,0,0,vx,vy,vz,0,0,0,0,0)' newline ...
            '    time.sleep(max(0.02,float(p.get("duration",0.05))))' newline ...
            '  elif a == "status":' newline ...
            '    pass' newline ...
            '  else:' newline ...
            '    raise RuntimeError(f"unknown action: {a}")' newline ...
            '  print("OK")' newline ...
            '  print(f"MODE={getattr(m, ''flightmode'', ''UNKNOWN'')}")' newline ...
            '  sys.exit(0)' newline ...
            'except Exception as e:' newline ...
            '  _noise = (_mav_stdout.getvalue() + _mav_stderr.getvalue()).strip()' newline ...
            '  print(f"ERR:{e}")' newline ...
            '  sys.exit(2)' newline ...
            ];

        ok = false;
        out = '';
        err_messages = strings(0, 1);

        for i = 1:numel(master_candidates)
            payload = struct('action', action_name, 'params', params_struct, ...
                'master', master_candidates{i}, 'timeout', timeout);
            payload_json = jsonencode(payload);
            payload_b64 = matlab.net.base64encode(uint8(payload_json));
            py_code = strrep(py_template, '__PAYLOAD_B64__', payload_b64);

            % Use a here-doc to avoid shell escaping issues.
            % Guard with shell timeout so TCP connect stalls cannot block workers indefinitely.
            shell_timeout_s = max(3, ceil(timeout + 2));
            cmd = sprintf('timeout %d python3 - <<''PY''\n%s\nPY', shell_timeout_s, py_code);
            [rc, cmd_out] = system(cmd);

            if (rc == 0) && contains(cmd_out, 'OK')
                ok = true;
                out = cmd_out;
                err = '';
                return;
            end

            msg = strtrim(cmd_out);
            if strlength(string(msg)) > 0
                lines = regexp(char(msg), '\\r?\\n', 'split');
                lines = lines(~cellfun('isempty', strtrim(lines)));
                if ~isempty(lines)
                    line_s = lower(strtrim(string(lines)));
                    keep_mask = ~contains(line_s, 'eof on tcp socket') & ...
                                ~contains(line_s, 'socket closed') & ...
                                ~contains(line_s, 'connection refused sleeping') & ...
                                ~contains(line_s, 'connection refused') & ...
                                ~contains(line_s, 'connection reset by peer') & ...
                                ~contains(line_s, 'connection timed out');
                    lines = lines(keep_mask);
                    if isempty(lines)
                        lines = {'ERR:heartbeat timeout'};
                    end
                    err_idx = find(startsWith(lines, 'ERR:'), 1, 'last');
                    if ~isempty(err_idx)
                        msg = strtrim(lines{err_idx});
                    else
                        msg = strtrim(lines{end});
                    end
                end
            end
            if rc == 124
                msg = sprintf('ERR:heartbeat timeout (shell timeout %ds)', shell_timeout_s);
            elseif strlength(string(msg)) == 0
                msg = sprintf('pymavlink command failed (rc=%d)', rc);
            end
            msg = regexprep(char(string(msg)), '\\s+', ' ');
            msg = strtrim(msg);
            err_messages(end+1) = sprintf('[%s] %s', master_candidates{i}, msg); %#ok<AGROW>
        end

        err = strjoin(err_messages, ' | ');
    end
    
    try
        switch lower(action)
            case 'arm'
                [ok, output, err] = run_pymav('arm', struct());
                result.output = output;
                if ok
                    result.status = 'armed';
                    result.is_success = true;
                else
                    result.status = 'arm_failed';
                    result.error_message = char(err);
                end
                
            case 'disarm'
                [ok, output, err] = run_pymav('disarm', struct());
                result.output = output;
                if ok
                    result.status = 'disarmed';
                    result.is_success = true;
                else
                    result.status = 'disarm_failed';
                    result.error_message = char(err);
                end
                
            case 'takeoff'
                % TAKEOFF to specified height
                if ~isfield(params, 'height')
                    error('takeoff requires params.height');
                end
                height = params.height;

                [ok, output, err] = run_pymav('takeoff', struct('height', height));
                result.output = output;
                if ok
                    result.status = 'takeoff_initiated';
                    result.is_success = true;
                else
                    result.status = 'takeoff_failed';
                    result.error_message = char(err);
                end
                
            case 'land'
                [ok, output, err] = run_pymav('land', struct());
                result.output = output;
                if ok
                    result.status = 'land_initiated';
                    result.is_success = true;
                else
                    result.status = 'land_failed';
                    result.error_message = char(err);
                end
                
            case 'set_velocity'
                % SET velocity command in GUIDED mode
                if ~isfield(params, 'vx') || ~isfield(params, 'vy') || ~isfield(params, 'vz')
                    error('set_velocity requires params.vx, params.vy, params.vz');
                end
                vx = params.vx;
                vy = params.vy;
                vz = params.vz;
                duration = 0.05;
                if isfield(params, 'duration')
                    duration = max(0.02, double(params.duration));
                end

                [ok, output, err] = run_pymav('set_velocity', struct('vx', vx, 'vy', vy, 'vz', vz, 'duration', duration));
                result.output = output;
                if ok
                    result.status = 'velocity_sent';
                    result.is_success = true;
                else
                    result.status = 'velocity_failed';
                    result.error_message = sprintf('Velocity command failed: %s', char(err));
                end
                % Do not block MATLAB loop further here; the Python helper already applies a tiny send hold.
                
            case 'set_mode'
                % SET flight mode
                if ~isfield(params, 'mode')
                    error('set_mode requires params.mode');
                end
                mode = params.mode;
                [ok, output, err] = run_pymav('set_mode', struct('mode', mode));
                result.output = output;
                if ok
                    result.status = sprintf('mode_set_%s', lower(mode));
                    result.is_success = true;
                else
                    result.status = 'mode_set_failed';
                    result.error_message = sprintf('Failed to set mode %s: %s', mode, char(err));
                end
                
            case 'status'
                % GET vehicle status
                [ok, output, err] = run_pymav('status', struct());
                result.output = output;
                result.status = 'status_obtained';
                result.is_success = ok;
                if ~ok
                    result.error_message = char(err);
                end
                
            otherwise
                result.status = 'unknown_action';
                result.error_message = sprintf('Unknown action: %s', action);
        end
        
    catch ME
        result.status = 'error';
        result.error_message = ME.message;
        result.is_success = false;
    end

    should_log_result = flow_log_all_actions || ~result.is_success;
    if should_log_result
        autlFlowLog(flow_log_file, 'autlMavproxyControl', 'action_result', autlFlowMerge(flow_ctx, struct( ...
            'action', char(string(action)), 'status', char(string(result.status)), ...
            'is_success', logical(result.is_success), 'error', char(string(result.error_message)))));
    end
end

function [result, ok] = autlRunMavrosControl(action, params, cfg, timeout_s)
% Run action via ROS2 MAVROS CLI path. Returns ok=true only on successful action completion.

result = struct('status', 'mavros_unavailable', 'output', '', 'error_message', '', 'is_success', false);
ok = false;

ns = '/mavros';
if isfield(cfg, 'control') && isstruct(cfg.control) && isfield(cfg.control, 'mavros_namespace') && ...
        strlength(string(cfg.control.mavros_namespace)) > 0
    ns = char(string(cfg.control.mavros_namespace));
end
if ~startsWith(ns, '/')
    ns = ['/' ns];
end
if endsWith(ns, '/')
    ns = ns(1:end-1);
end

repo_root = fileparts(fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))))));
iicc26_setup = fullfile(fileparts(repo_root), 'IICC26_ws', 'install', 'setup.bash');
source_overlay = sprintf('[ -f "%s" ] && source "%s" >/dev/null 2>&1; ', iicc26_setup, iicc26_setup);

check_cmd = ['bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; ' source_overlay 'set -u; ros2 pkg list 2>/dev/null | grep -q "^mavros$"'''];
[check_rc, ~] = system(check_cmd);
if check_rc ~= 0
    result.status = 'mavros_not_installed';
    result.error_message = 'mavros package is not installed in ROS2 environment';
    return;
end

shell_timeout_s = max(1, ceil(timeout_s + 1));

requires_connected_fcu = ismember(lower(action), {'status', 'arm', 'disarm', 'set_mode', 'takeoff', 'land'});
if requires_connected_fcu
    [connected_ok, connected_msg, connected_out] = autlWaitForMavrosConnected(ns, source_overlay, max(4.0, timeout_s + 2.0));
    if ~connected_ok
        result.status = 'mavros_not_connected';
        result.error_message = connected_msg;
        result.output = connected_out;
        return;
    end
end

switch lower(action)
    case 'status'
        result.status = 'status_obtained';
        result.is_success = true;
        result.output = sprintf('connected: true (%s)', ns);
        ok = true;
        return;
    case 'arm'
        cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 service call %s/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"''', ...
            source_overlay, shell_timeout_s, ns);
    case 'disarm'
        cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 service call %s/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"''', ...
            source_overlay, shell_timeout_s, ns);
    case 'set_mode'
        if ~isfield(params, 'mode')
            result.status = 'mode_set_failed';
            result.error_message = 'set_mode requires params.mode';
            return;
        end
        mode_name = char(string(params.mode));
        cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 service call %s/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: \"%s\"}"''', ...
            source_overlay, shell_timeout_s, ns, mode_name);
    case 'takeoff'
        h = 3.0;
        if isfield(params, 'height')
            h = double(params.height);
        end
        cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 service call %s/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: %.3f}"''', ...
            source_overlay, shell_timeout_s, ns, h);
    case 'land'
        cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 service call %s/cmd/land mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"''', ...
            source_overlay, shell_timeout_s, ns);
    case 'set_velocity'
        if ~isfield(params, 'vx') || ~isfield(params, 'vy') || ~isfield(params, 'vz')
            result.status = 'velocity_failed';
            result.error_message = 'set_velocity requires params.vx, params.vy, params.vz';
            return;
        end
        vx = double(params.vx);
        vy = double(params.vy);
        vz = double(params.vz);
        cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 topic pub --once %s/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: %.4f, y: %.4f, z: %.4f}, angular: {x: 0.0, y: 0.0, z: 0.0}}"''', ...
            source_overlay, shell_timeout_s, ns, vx, vy, vz);
    otherwise
        result.status = 'unknown_action';
        result.error_message = sprintf('Unknown action: %s', action);
        return;
end

[rc, out] = system(cmd);
result.output = out;
if rc == 0
    out_low = lower(char(string(out)));
    service_ok = true;
    if ismember(lower(action), {'arm', 'disarm', 'takeoff', 'land'})
        service_ok = ~isempty(regexp(out_low, 'success\s*:\s*true', 'once'));
    elseif strcmpi(action, 'set_mode')
        service_ok = ~isempty(regexp(out_low, 'mode_sent\s*:\s*true', 'once')) || ...
                     ~isempty(regexp(out_low, 'success\s*:\s*true', 'once'));
    end

    if service_ok
        result.is_success = true;
        switch lower(action)
            case 'arm'
                result.status = 'armed';
            case 'disarm'
                result.status = 'disarmed';
            case 'takeoff'
                result.status = 'takeoff_initiated';
            case 'land'
                result.status = 'land_initiated';
            case 'set_mode'
                result.status = 'mode_set';
            case 'set_velocity'
                result.status = 'velocity_sent';
            otherwise
                result.status = 'status_obtained';
        end
        ok = true;
    else
        result.is_success = false;
        result.status = 'mavros_action_failed';
        result.error_message = strtrim(out);
    end
else
    result.is_success = false;
    result.status = 'mavros_action_failed';
    if rc == 124
        result.error_message = sprintf('mavros action timeout (%ds): %s', shell_timeout_s, strtrim(out));
    else
        result.error_message = strtrim(out);
    end
end
end

function [ok, msg, out] = autlWaitForMavrosConnected(ns, source_overlay, timeout_s)
% Poll MAVROS state until FCU link is connected:true.

ok = false;
msg = 'mavros state unavailable';
out = '';

poll_s = 0.8;
start_t = tic;

while toc(start_t) < timeout_s
    remain_s = max(1, ceil(timeout_s - toc(start_t)));
    sample_timeout_s = min(3, remain_s);
    cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; timeout %d ros2 topic echo --once %s/state''', ...
        source_overlay, sample_timeout_s, ns);
    [rc, cmd_out] = system(cmd);
    out = cmd_out;

    if rc == 0
        out_low = lower(char(string(cmd_out)));
        if contains(out_low, 'connected: true')
            ok = true;
            msg = 'connected';
            return;
        end
        msg = 'connected=false';
    elseif rc == 124
        msg = sprintf('state echo timeout (%ds)', sample_timeout_s);
    else
        msg = strtrim(char(string(cmd_out)));
        if strlength(string(msg)) == 0
            msg = sprintf('state echo failed (rc=%d)', rc);
        end
    end

    pause(poll_s);
end

msg = sprintf('MAVROS FCU not connected on %s within %.1fs (%s)', ns, timeout_s, msg);
end

function out = autlFlowMerge(base_payload, extra_payload)
out = struct();
if nargin >= 1 && isstruct(base_payload)
    f1 = fieldnames(base_payload);
    for i = 1:numel(f1)
        out.(f1{i}) = base_payload.(f1{i});
    end
end
if nargin >= 2 && isstruct(extra_payload)
    f2 = fieldnames(extra_payload);
    for i = 1:numel(f2)
        out.(f2{i}) = extra_payload.(f2{i});
    end
end
end
