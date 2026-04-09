function autlDumpMavrosDebugTop(control_cfg, log_prefix)
% Dump one-shot MAVROS state/statustext to help root-cause arm/takeoff rejection.

if nargin < 2
    log_prefix = '';
end

ns = '/mavros';
if isstruct(control_cfg) && isfield(control_cfg, 'control') && isstruct(control_cfg.control) && ...
        isfield(control_cfg.control, 'mavros_namespace')
    ns = char(string(control_cfg.control.mavros_namespace));
end

state_topic = sprintf('%s/state', ns);
text_topic = sprintf('%s/statustext/recv', ns);

state_cmd = sprintf(['bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u; ' ...
    'timeout 3 ros2 topic echo --once %s 2>/dev/null'''], state_topic);
[~, state_out] = system(state_cmd);
if strlength(string(strtrim(state_out))) > 0
    fprintf('%s[AutoLandingDataCollection] MAVROS state snapshot:\n%s\n', log_prefix, strtrim(state_out));
else
    fprintf('%s[AutoLandingDataCollection] MAVROS state snapshot unavailable on %s\n', log_prefix, state_topic);
end

text_cmd = sprintf(['bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u; ' ...
    'timeout 3 ros2 topic echo --qos-reliability best_effort --qos-durability volatile --once %s 2>/dev/null'''], text_topic);
[~, text_out] = system(text_cmd);
if strlength(string(strtrim(text_out))) > 0
    fprintf('%s[AutoLandingDataCollection] MAVROS statustext snapshot:\n%s\n', log_prefix, strtrim(text_out));
else
    fprintf('%s[AutoLandingDataCollection] MAVROS statustext snapshot unavailable on %s\n', log_prefix, text_topic);
end
end
