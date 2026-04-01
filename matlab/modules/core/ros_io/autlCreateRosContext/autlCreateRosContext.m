function rosCtx = autlCreateRosContext(nodeName)
% autlCreateRosContext
% Create ROS 2 context for runtime integration.

if nargin < 1 || strlength(string(nodeName)) == 0
    nodeName = "autolanding_matlab_node";
end

rosCtx = struct();
rosCtx.enabled = false;
rosCtx.node = [];
rosCtx.status = "disabled";

try
    rosCtx.node = ros2node(char(nodeName));
    rosCtx.enabled = true;
    rosCtx.status = "ready";
catch ME
    rosCtx.enabled = false;
    rosCtx.status = "unavailable";
    rosCtx.error_message = string(ME.message);
end
end
