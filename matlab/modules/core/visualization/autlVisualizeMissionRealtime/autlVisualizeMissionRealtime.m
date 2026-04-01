function autlVisualizeMissionRealtime(session_id, poll_interval)
% autlVisualizeMissionRealtime
% Real-time visualization of drone mission progress, state, and telemetry.
% 
% Usage:
%   autlVisualizeMissionRealtime('mission_001')
%   autlVisualizeMissionRealtime('mission_001', 0.5)  % 0.5 sec poll

if nargin < 1 || strlength(string(session_id)) == 0
    session_id = datestr(now, 'yyyymmdd_HHMMSS');
end
if nargin < 2 || ~isfinite(poll_interval)
    poll_interval = 0.5;  % seconds
end

% Create visualization figure with 6 subplots (similar to IICC26 monitor)
fig = figure('Name', 'AutoLanding Mission Realtime Monitor', 'NumberTitle', 'off', ...
    'Position', [200, 200, 1400, 900]);
tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% Plot 1: 3D Trajectory
ax1 = nexttile(tl, 1, [1, 2]);
hold(ax1, 'on');
title(ax1, 'Drone 3D Trajectory (Real-time)');
xlabel(ax1, 'X (m)');
ylabel(ax1, 'Y (m)');
zlabel(ax1, 'Z (m)');
view(ax1, 45, 30);
grid(ax1, 'on');
traj_line = animatedline(ax1, 'Color', [0.2 0.6 0.9], 'LineWidth', 2);
drone_marker = scatter(ax1, 0, 0, 0, 'r*', 'SizeData', 200, 'DisplayName', 'Current Position');

% Plot 2: Mission Phase Progress
ax2 = nexttile(tl, 3);
hold(ax2, 'on');
title(ax2, 'Mission Phase Progress');
ylabel(ax2, 'Phase');
ylim(ax2, [0.5, 5.5]);
yticks(ax2, 1:5);
yticklabels(ax2, {'ARM', 'TAKEOFF', 'TRAJECTORY', 'LAND', 'DISARM'});
xlim(ax2, [0, 100]);
grid(ax2, 'on');
phase_bars = barh(ax2, 1:5, [0 0 0 0 0], 'FaceColor', [0.3 0.7 0.3], 'BarWidth', 0.6);

% Plot 3: Distance to Target
ax3 = nexttile(tl, 4);
hold(ax3, 'on');
title(ax3, 'Distance to Landing Target');
xlabel(ax3, 'Time (s)');
ylabel(ax3, 'Distance (m)');
grid(ax3, 'on');
dist_line = animatedline(ax3, 'Color', [0.8 0.4 0.2], 'LineWidth', 1.5);
ax3_set_ylim = false;

% Plot 4: Altitude Profile
ax4 = nexttile(tl, 5);
hold(ax4, 'on');
title(ax4, 'Altitude vs Time');
xlabel(ax4, 'Time (s)');
ylabel(ax4, 'Z (m)');
grid(ax4, 'on');
alt_line = animatedline(ax4, 'Color', [0.2 0.8 0.2], 'LineWidth', 1.5);

% Plot 5: Velocity Vector
ax5 = nexttile(tl, 6);
hold(ax5, 'on');
title(ax5, 'Velocity Magnitude vs Time');
xlabel(ax5, 'Time (s)');
ylabel(ax5, 'Speed (m/s)');
grid(ax5, 'on');
vel_line = animatedline(ax5, 'Color', [0.9 0.6 0.0], 'LineWidth', 1.5);

% Plot 6: Battery and Thrust
ax6 = nexttile(tl, 7);
hold(ax6, 'on');
title(ax6, 'Battery Voltage');
xlabel(ax6, 'Time (s)');
ylabel(ax6, 'Voltage (V)');
grid(ax6, 'on');
bat_line = animatedline(ax6, 'Color', [0.6 0.2 0.8], 'LineWidth', 1.5);

% Plot 7: Thrust Percentage
ax7 = nexttile(tl, 8);
hold(ax7, 'on');
title(ax7, 'Motor Thrust');
xlabel(ax7, 'Time (s)');
ylabel(ax7, 'Throttle (%)');
ylim(ax7, [0, 100]);
grid(ax7, 'on');
thrust_line = animatedline(ax7, 'Color', [1.0 0.5 0.0], 'LineWidth', 1.5);

% Plot 8: Status Text
ax8 = nexttile(tl, 9);
axis(ax8, 'off');
status_text = text(ax8, 0.1, 0.5, '', 'FontSize', 10, 'FontName', 'monospaced', ...
    'VerticalAlignment', 'middle', 'UserData', {});

% Data logging
mission_data = struct();
mission_data.timestamp = [];
mission_data.position_xyz = [];
mission_data.velocity_xyz = [];
mission_data.altitude = [];
mission_data.battery_v = [];
mission_data.throttle_pct = [];
mission_data.current_phase = 'INIT';
mission_data.phase_progress = [0 0 0 0 0];
mission_data.target_xyz = [0 0 0];

t_start = tic;
stop_flag = false;

% Close figure handler
set(fig, 'CloseRequestFcn', @(~,~) set(fig, 'UserData', {'stop'}));

fprintf('[MissionMonitor] Realtime visualization started. Session: %s\n', session_id);
fprintf('[MissionMonitor] Connecting to MAVProxy telemetry...\n');

% Main visualization loop
while isgraphics(fig)
    try
        t_elapsed = toc(t_start);
        
        % Get current vehicle state
        vehicle_state = autlMissionQueryVehicleState();
        
        % Update mission data
        mission_data.timestamp = [mission_data.timestamp; t_elapsed];
        mission_data.position_xyz = [mission_data.position_xyz; vehicle_state.position];
        mission_data.velocity_xyz = [mission_data.velocity_xyz; vehicle_state.velocity];
        mission_data.altitude = [mission_data.altitude; vehicle_state.position(3)];
        mission_data.battery_v = [mission_data.battery_v; vehicle_state.battery_v];
        mission_data.throttle_pct = [mission_data.throttle_pct; vehicle_state.throttle];
        
        % Update 3D trajectory plot
        addpoints(traj_line, vehicle_state.position(1), vehicle_state.position(2), -vehicle_state.position(3));
        if ~isempty(mission_data.position_xyz)
            set(drone_marker, 'XData', vehicle_state.position(1), 'YData', vehicle_state.position(2), 'ZData', -vehicle_state.position(3));
        end
        
        % Update distance to target
        dist_to_target = norm(vehicle_state.position - mission_data.target_xyz);
        addpoints(dist_line, t_elapsed, dist_to_target);
        if dist_to_target > 20
            ax3_set_ylim = false;
        end
        if ~ax3_set_ylim
            set(ax3, 'YLim', [0, max(20, dist_to_target * 1.2)]);
            ax3_set_ylim = true;
        end
        
        % Update altitude
        addpoints(alt_line, t_elapsed, vehicle_state.position(3));
        
        % Update velocity
        vel_mag = norm(vehicle_state.velocity);
        addpoints(vel_line, t_elapsed, vel_mag);
        
        % Update battery
        addpoints(bat_line, t_elapsed, vehicle_state.battery_v);
        
        % Update thrust
        addpoints(thrust_line, t_elapsed, vehicle_state.throttle);
        
        % Update phase progress bars
        phase_progress = [20, 20, 40, 15, 5];  % Dummy progression
        if vehicle_state.armed && ~vehicle_state.in_air
            phase_progress(1) = 100;  % ARM complete
        end
        if vehicle_state.in_air && vehicle_state.position(3) < 1.5
            phase_progress(2) = 100;  % TAKEOFF starting
        elseif vehicle_state.position(3) >= 1.9
            phase_progress(2) = 100;  % TAKEOFF complete
            phase_progress(3) = 50;   % TRAJECTORY starting
        end
        
        % Draw bars
        for i = 1:5
            set(phase_bars(i), 'XData', phase_progress(i));
        end
        
        % Update status text
        status_str = sprintf(['Mission Monitor\n' ...
            '─────────────────────\n' ...
            'Time: %.1f s\n' ...
            'Phase: %s\n' ...
            'Position: [%.2f, %.2f, %.2f] m\n' ...
            'Velocity: [%.2f, %.2f, %.2f] m/s\n' ...
            'Speed: %.2f m/s\n' ...
            'Altitude: %.2f m\n' ...
            'Battery: %.2f V\n' ...
            'Throttle: %.1f %%\n' ...
            'Armed: %s\n' ...
            'GPS Fix: %s'], ...
            t_elapsed, vehicle_state.mode, ...
            vehicle_state.position(1), vehicle_state.position(2), vehicle_state.position(3), ...
            vehicle_state.velocity(1), vehicle_state.velocity(2), vehicle_state.velocity(3), ...
            vel_mag, vehicle_state.position(3), vehicle_state.battery_v, vehicle_state.throttle, ...
            char(string(vehicle_state.armed)), char(string(vehicle_state.gps_fix)));
        
        set(status_text, 'String', status_str);
        
        % Check for stop signal
        if ~isempty(get(fig, 'UserData'))
            break;
        end
        
        % Redraw and poll
        drawnow limitrate;
        pause(poll_interval);
        
    catch ME
        if ~isgraphics(fig)
            break;
        end
        warning('[MissionMonitor] Update error: %s', ME.message);
        pause(1);
    end
end

fprintf('[MissionMonitor] Visualization stopped.\n');
close(fig);
end

function vehicle_state = autlMissionQueryVehicleState()
% Query current vehicle telemetry state from MAVProxy

vehicle_state = struct();

cmd = sprintf('timeout 5 mavproxy.py --master tcp:127.0.0.1:5760 --cmd="status" 2>/dev/null');
[~, output] = system(cmd);

% Position (NED frame)
vehicle_state.position = [0, 0, 0];
pos_match = regexp(output, 'GPS:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', 'tokens');
if ~isempty(pos_match)
    vehicle_state.position = [str2double(pos_match{1}{1}), str2double(pos_match{1}{2}), str2double(pos_match{1}{3})];
end

% Velocity (from VFR_HUD)
vehicle_state.velocity = [0, 0, 0];
vel_match = regexp(output, 'airspeed=([-\d.]+).*groundspeed=([-\d.]+).*climb=([-\d.]+)', 'tokens');
if ~isempty(vel_match)
    vehicle_state.velocity(1) = str2double(vel_match{1}{1});
    vehicle_state.velocity(2) = str2double(vel_match{1}{2});
    vehicle_state.velocity(3) = str2double(vel_match{1}{3});
end

% Throttle
vehicle_state.throttle = 0;
thr_match = regexp(output, 'throttle=([-\d.]+)', 'tokens');
if ~isempty(thr_match)
    vehicle_state.throttle = str2double(thr_match{1}{1});
end

% Battery
vehicle_state.battery_v = 12.6;
bat_match = regexp(output, 'Vcc=([-\d.]+)V', 'tokens');
if ~isempty(bat_match)
    vehicle_state.battery_v = str2double(bat_match{1}{1});
end

% Armed state
vehicle_state.armed = false;
vehicle_state.mode = 'UNKNOWN';
mode_match = regexp(output, 'mode: (\w+)', 'tokens');
if ~isempty(mode_match)
    vehicle_state.mode = mode_match{1}{1};
    vehicle_state.armed = ~strcmpi(vehicle_state.mode, 'STABILIZE');
end

% In air
vehicle_state.in_air = vehicle_state.position(3) > 0.3;

% GPS fix
vehicle_state.gps_fix = contains(output, 'Detected vehicle');

end
