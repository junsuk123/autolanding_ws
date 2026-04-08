function [success, message] = autlResetGazeboSimulation(world_name, log_prefix)
% autlResetGazeboSimulation
% Perform a Gazebo soft reset using ROS2 services with safe fallbacks.

if nargin < 1 || strlength(string(world_name)) == 0
    world_name = 'default';
end
if nargin < 2
    log_prefix = '';
end

success = false;
message = '';

timeout_sec = 5.0;
retry_count = 2;
ros_setup = 'set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u; ';

service_targets = {
    struct('name', sprintf('/world/%s/control', char(world_name)), 'type', 'world_control'),
    struct('name', '/gazebo/reset_simulation', 'type', 'empty'),
    struct('name', '/reset_simulation', 'type', 'empty'),
    struct('name', '/gazebo/reset_world', 'type', 'empty'),
    struct('name', '/reset_world', 'type', 'empty')
};

for i = 1:numel(service_targets)
    target = service_targets{i};
    for k = 1:retry_count
        if strcmp(target.type, 'world_control')
            core = sprintf([ ...
                '%stimeout %.1f ros2 service call %s gz_msgs/srv/WorldControl ', ...
                '''{reset: {all: true}}'' 2>&1'], ...
                ros_setup, timeout_sec, target.name);
        else
            core = sprintf([ ...
                '%stimeout %.1f ros2 service call %s std_srvs/srv/Empty ', ...
                '''{}'' 2>&1'], ...
                ros_setup, timeout_sec, target.name);
        end

        cmd = sprintf('bash -lc "%s"', localEscapeDoubleQuotes(core));
        [st, out] = system(cmd);
        out_l = lower(string(out));
        if st == 0 && ~contains(out_l, 'error') && ~contains(out_l, 'failed')
            success = true;
            message = sprintf('Gazebo simulation reset via %s', target.name);
            fprintf('%s[AutoLandingDataCollection] Gazebo soft reset: %s\n', log_prefix, message);
            return;
        end

        if k < retry_count
            pause(0.5);
        end
    end
end

% Last-resort fallback: direct gz transport API
core = sprintf([ ...
    '%stimeout %.1f gz service -s /world/%s/control ', ...
    '--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean ', ...
    '--timeout 3000 --req ''reset: {all: true}'' 2>&1'], ...
    ros_setup, timeout_sec, char(world_name));
cmd = sprintf('bash -lc "%s"', localEscapeDoubleQuotes(core));
[st, out] = system(cmd);
out_l = lower(string(out));
if st == 0 && ~contains(out_l, 'error') && ~contains(out_l, 'failed')
    success = true;
    message = sprintf('Gazebo simulation reset via gz command (world=%s)', char(world_name));
    fprintf('%s[AutoLandingDataCollection] Gazebo soft reset: %s\n', log_prefix, message);
    return;
end

message = sprintf('Failed to reset Gazebo simulation for world=%s', char(world_name));
fprintf('%s[AutoLandingDataCollection] Warning: %s\n', log_prefix, message);
end

function out = localEscapeDoubleQuotes(in)
out = strrep(char(in), '"', '\"');
end
