function status = autlPublishTrajectory(rosCtx, trajTbl, topicName)
% autlPublishTrajectory
% Publish generated trajectory to ROS 2 topic.

if nargin < 3 || strlength(string(topicName)) == 0
    topicName = "/autolanding/trajectory";
end

status = struct('ok', false, 'topic', string(topicName));

if ~isstruct(rosCtx) || ~isfield(rosCtx, 'enabled') || ~logical(rosCtx.enabled)
    status.message = "ROS context is not enabled.";
    return;
end

try
    pub = ros2publisher(rosCtx.node, char(topicName), 'std_msgs/Float32MultiArray');
    msg = ros2message(pub);

    data = [double(trajTbl.x)'; double(trajTbl.y)'; double(trajTbl.z)'];
    msg.data = single(data(:));
    send(pub, msg);

    status.ok = true;
    status.points = height(trajTbl);
catch ME
    status.ok = false;
    status.message = string(ME.message);
end
end
