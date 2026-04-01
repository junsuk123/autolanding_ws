function autlReleaseRosContext(rosCtx)
% autlReleaseRosContext
% Release ROS 2 context resources.

if ~isstruct(rosCtx)
    return;
end
if isfield(rosCtx, 'node') && ~isempty(rosCtx.node)
    try
        clear rosCtx.node;
    catch
    end
end
end
