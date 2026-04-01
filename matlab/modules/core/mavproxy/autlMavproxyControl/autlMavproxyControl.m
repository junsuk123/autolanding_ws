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

    % Build a small fallback list to tolerate SITL port churn.
    master_candidates = {master_conn};
    if contains(master_conn, ':5762')
        master_candidates{end+1} = strrep(master_conn, ':5762', ':5760');
    elseif contains(master_conn, ':5760')
        master_candidates{end+1} = strrep(master_conn, ':5760', ':5762');
    end
    master_candidates = unique(master_candidates, 'stable');

    function [ok, out, err] = run_pymav(action_name, params_struct)
        if nargin < 2 || isempty(params_struct)
            params_struct = struct();
        end
        py_template = [ ...
            'import base64,json,sys,time' newline ...
            'from pymavlink import mavutil' newline ...
            'try:' newline ...
            '  d = json.loads(base64.b64decode("__PAYLOAD_B64__").decode())' newline ...
            '  a = d["action"]; p = d["params"]; t = float(d["timeout"])' newline ...
            '  m = mavutil.mavlink_connection(d["master"], source_system=255)' newline ...
            '  hb = m.wait_heartbeat(timeout=max(0.8, t))' newline ...
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
            cmd = sprintf('python3 - <<''PY''\n%s\nPY', py_code);
            [rc, cmd_out] = system(cmd);

            if (rc == 0) && contains(cmd_out, 'OK')
                ok = true;
                out = cmd_out;
                err = '';
                return;
            end

            msg = strtrim(cmd_out);
            if strlength(string(msg)) == 0
                msg = sprintf('pymavlink command failed (rc=%d)', rc);
            end
            err_messages(end+1) = sprintf('[%s] %s', master_candidates{i}, msg); %#ok<AGROW>
        end

        err = strjoin(err_messages, newline);
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
end
