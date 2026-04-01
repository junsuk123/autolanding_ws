function obs = autlReadRosObservation(rosCtx, topicMap)
% autlReadRosObservation
% Read current observation from ROS 2 topics if available.

obs = struct();
obs.ok = false;
obs.source = "none";

if nargin < 2 || isempty(topicMap)
    topicMap = struct();
end

if ~isstruct(rosCtx) || ~isfield(rosCtx, 'enabled') || ~logical(rosCtx.enabled)
    obs.message = "ROS context is not enabled.";
    return;
end

% Placeholder interface: users can customize topic subscriptions here.
obs.ok = true;
obs.source = "ros2";
obs.topic_map = topicMap;
obs.timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
end
