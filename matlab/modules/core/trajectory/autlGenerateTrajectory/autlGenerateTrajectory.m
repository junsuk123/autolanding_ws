function trajTbl = autlGenerateTrajectory(initialState, targetState, fused, cfg)
% autlGenerateTrajectory
% Generate autonomous landing trajectory from fused semantic confidence.

dt = cfg.policy.dt;
steps = max(1, floor(cfg.policy.horizon_s / dt));

x = double(initialState.x);
y = double(initialState.y);
z = double(initialState.z);

tx = double(targetState.x);
ty = double(targetState.y);
tz = double(targetState.z);

maxXY = cfg.policy.max_xy_speed;
maxZ = cfg.policy.max_z_descent_speed;
minZ = cfg.policy.min_z_descent_speed;

fusedConfidence = fused.fused_confidence;

rows = zeros(steps + 1, 8);
rowCount = 0;
for k = 0:steps
    t = k * dt;

    dx = tx - x;
    dy = ty - y;
    dz = z - tz;

    vxCmd = autlClamp(0.8 * dx, -maxXY, maxXY);
    vyCmd = autlClamp(0.8 * dy, -maxXY, maxXY);

    baseDescent = minZ + (maxZ - minZ) * fusedConfidence;
    vzCmd = -autlClamp(baseDescent, minZ, maxZ);
    if dz < 0.2
        vzCmd = -0.4 * minZ;
    end

    x = x + vxCmd * dt;
    y = y + vyCmd * dt;
    z = max(tz, z + vzCmd * dt);

    rowCount = rowCount + 1;
    rows(rowCount, :) = [t, x, y, z, vxCmd, vyCmd, vzCmd, fusedConfidence];

    if z <= tz + 0.02 && abs(x - tx) < 0.05 && abs(y - ty) < 0.05
        break;
    end
end

rows = rows(1:rowCount, :);
trajTbl = array2table(rows, 'VariableNames', ...
    {'t', 'x', 'y', 'z', 'vx_cmd', 'vy_cmd', 'vz_cmd', 'fused_confidence'});
end
