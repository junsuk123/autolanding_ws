function metrics = autlEvaluateTrajectoryMetrics(trajTbl, targetState)
% autlEvaluateTrajectoryMetrics
% Compute trajectory quality metrics from generated trajectory.

if isempty(trajTbl)
    metrics = struct('rmse_xyz', nan, 'rmse_xy', nan, 'touchdown_error_xy', nan, ...
        'touchdown_error_z', nan, 'path_length_xy', nan);
    return;
end

x = double(trajTbl.x);
y = double(trajTbl.y);
z = double(trajTbl.z);

tx = double(targetState.x);
ty = double(targetState.y);
tz = double(targetState.z);

ex = x - tx;
ey = y - ty;
ez = z - tz;

metrics = struct();
metrics.rmse_xyz = sqrt(mean(ex.^2 + ey.^2 + ez.^2));
metrics.rmse_xy = sqrt(mean(ex.^2 + ey.^2));
metrics.touchdown_error_xy = sqrt((x(end)-tx).^2 + (y(end)-ty).^2);
metrics.touchdown_error_z = abs(z(end)-tz);

if numel(x) >= 2
    dx = diff(x);
    dy = diff(y);
    metrics.path_length_xy = sum(sqrt(dx.^2 + dy.^2));
else
    metrics.path_length_xy = 0.0;
end
end
