function collection_result = autlRunDataCollection(rootDir, mission_config)
% autlRunDataCollection
% Raw sensor and vehicle data collection without ontology processing.
% Collects: position, velocity, attitude, thrust, GPS, IMU raw, rotor speeds.
%
% Usage:
%   result = autlRunDataCollection(rootDir, mission_config)
%   result = autlRunDataCollection([], struct('max_duration', 120, 'sample_rate', 50))

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end
if nargin < 2 || isempty(mission_config)
    mission_config = struct();
end

% Default collection config
if ~isfield(mission_config, 'max_duration')
    mission_config.max_duration = 120;  % seconds
end
if ~isfield(mission_config, 'sample_rate')
    mission_config.sample_rate = 50;  % Hz
end
if ~isfield(mission_config, 'output_dir')
    mission_config.output_dir = fullfile(rootDir, 'data', 'collected');
end
if ~isfield(mission_config, 'session_id')
    mission_config.session_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~isfield(mission_config, 'gazebo_server_mode')
    mission_config.gazebo_server_mode = true;  % Default: server/headless mode
end
if ~isfield(mission_config, 'enable_visualization')
    mission_config.enable_visualization = true;  % Default: enable visualization
end
if ~isfield(mission_config, 'reuse_visualization_window')
    mission_config.reuse_visualization_window = true;
end
if ~isfield(mission_config, 'save_realtime_viz_snapshot')
    mission_config.save_realtime_viz_snapshot = false;
end
if ~isfield(mission_config, 'close_visualization_on_finish')
    mission_config.close_visualization_on_finish = true;
end
if ~isfield(mission_config, 'force_close_stale_visualizations')
    mission_config.force_close_stale_visualizations = true;
end
if ~isfield(mission_config, 'log_prefix')
    mission_config.log_prefix = '';
end
if ~isfield(mission_config, 'enable_auto_motion')
    mission_config.enable_auto_motion = true;
end
if ~isfield(mission_config, 'takeoff_height_m')
    mission_config.takeoff_height_m = 3.0;
end
if ~isfield(mission_config, 'control_interval_s')
    mission_config.control_interval_s = 5.0;
end
if ~isfield(mission_config, 'landing_pad_center')
    mission_config.landing_pad_center = [0.0, 0.0, 0.0];
end
if ~isfield(mission_config, 'landing_pad_size')
    mission_config.landing_pad_size = [1.2, 1.2];
end
if ~isfield(mission_config, 'landing_pad_topic')
    mission_config.landing_pad_topic = '/autolanding/landing_pad';
end
if ~isfield(mission_config, 'allow_simulated_fallback')
    mission_config.allow_simulated_fallback = false;
end
if ~isfield(mission_config, 'telemetry_query_interval_s')
    mission_config.telemetry_query_interval_s = 5.0;
end
if ~isfield(mission_config, 'mavlink_master')
    mission_config.mavlink_master = 'tcp:127.0.0.1:5762';
end
if ~isfield(mission_config, 'mavlink_master_fallback')
    mission_config.mavlink_master_fallback = 'tcp:127.0.0.1:5760';
end

log_prefix = char(string(mission_config.log_prefix));

% Create output directory
if ~exist(mission_config.output_dir, 'dir')
    mkdir(mission_config.output_dir);
end

sessionDir = fullfile(mission_config.output_dir, mission_config.session_id);
if ~exist(sessionDir, 'dir')
    mkdir(sessionDir);
end

% Initialize collection
sample_interval = 1 / mission_config.sample_rate;
max_samples = round(mission_config.max_duration * mission_config.sample_rate);

% Pre-allocate raw data arrays
raw_data = struct();
raw_data.timestamp = zeros(max_samples, 1);
raw_data.position_xyz = zeros(max_samples, 3);     % [x, y, z] in meters
raw_data.velocity_xyz = zeros(max_samples, 3);     % [vx, vy, vz] in m/s
raw_data.attitude_rpy = zeros(max_samples, 3);     % [roll, pitch, yaw] in radians
raw_data.attitude_quat = zeros(max_samples, 4);    % [qx, qy, qz, qw]
raw_data.thrust_percent = zeros(max_samples, 1);   % 0-100
raw_data.rotor_speeds = zeros(max_samples, 4);     % RPM for 4 rotors
raw_data.gps_fix = zeros(max_samples, 1);          % 0=no fix, 1=GPS
raw_data.imu_accel_xyz = zeros(max_samples, 3);    % [ax, ay, az] in m/s^2
raw_data.imu_gyro_xyz = zeros(max_samples, 3);     % [gx, gy, gz] in rad/s
raw_data.battery_voltage = zeros(max_samples, 1);  % volts
raw_data.battery_current = zeros(max_samples, 1);  % amps
raw_data.barometer_alt = zeros(max_samples, 1);    % meters
raw_data.rangefinder_dist = zeros(max_samples, 1); % meters
raw_data.armed_state = zeros(max_samples, 1);      % 0=disarmed, 1=armed
raw_data.flight_mode = cell(max_samples, 1);       % string mode names
raw_data.landing_pad_center_xyz = zeros(max_samples, 3);
raw_data.landing_pad_size_xy = zeros(max_samples, 2);
raw_data.landing_pad_distance_xy = zeros(max_samples, 1);
raw_data.sample_count = 0;

log_entries = {};
t_start = tic;

fprintf('%s[AutoLandingDataCollection] Starting raw data collection (%d Hz, max %.0f sec)\n', log_prefix, ...
    mission_config.sample_rate, mission_config.max_duration);
if mission_config.gazebo_server_mode
    fprintf('%s[AutoLandingDataCollection] Gazebo Mode: Server (Headless)\n', log_prefix);
else
    fprintf('%s[AutoLandingDataCollection] Gazebo Mode: GUI\n', log_prefix);
end
if mission_config.enable_visualization
    fprintf('%s[AutoLandingDataCollection] Real-time Visualization: ENABLED\n', log_prefix);
else
    fprintf('%s[AutoLandingDataCollection] Real-time Visualization: DISABLED\n', log_prefix);
end
fprintf('%s[AutoLandingDataCollection] Landing Pad Center: [%.2f, %.2f, %.2f], Size: [%.2f x %.2f] m\n', ...
    log_prefix, mission_config.landing_pad_center(1), mission_config.landing_pad_center(2), mission_config.landing_pad_center(3), ...
    mission_config.landing_pad_size(1), mission_config.landing_pad_size(2));

% Setup real-time visualization if enabled
fig_handle = [];
viz_state = struct('enabled', false);
persistent autl_viz_cache
if mission_config.enable_visualization
    try
        if mission_config.force_close_stale_visualizations
            keep_fig = [];
            if ~isempty(autl_viz_cache) && isstruct(autl_viz_cache) && isfield(autl_viz_cache, 'fig') && isgraphics(autl_viz_cache.fig)
                keep_fig = autl_viz_cache.fig;
            end
            autlCloseStaleRealtimeFigures(keep_fig);
        end

        need_new_figure = true;
        if mission_config.reuse_visualization_window && ~isempty(autl_viz_cache) && ...
                isstruct(autl_viz_cache) && isfield(autl_viz_cache, 'fig') && isgraphics(autl_viz_cache.fig)
            need_new_figure = false;
        end

        if need_new_figure
            fig_handle = figure('Name', 'AutoLanding Real-time Data Collection', 'NumberTitle', 'off', ...
                'Tag', 'autl_realtime_collection', 'Position', [100, 100, 1200, 600]);
            autl_viz_cache = autlInitRealtimeVizLayout(fig_handle);
        else
            fig_handle = autl_viz_cache.fig;
            set(fig_handle, 'Visible', 'on');
        end

        autl_viz_cache = autlResetRealtimeVizLayout(autl_viz_cache, mission_config.session_id);
        viz_state = autl_viz_cache;
        viz_state.enabled = true;
    catch
        fprintf('%s[AutoLandingDataCollection] Warning: Could not create visualization figure\n', log_prefix);
    end
end

% Cleanup handler for graceful interrupt (Ctrl+C)
collection_state = struct('session_dir', sessionDir, 'interrupted', false, 'last_saved_idx', 0, 'raw_data', raw_data);
cleanup_obj = onCleanup(@() autlDataCollectionCleanup(collection_state));

try
    % Connect to MAVProxy for telemetry
    mav_config = struct('master', mission_config.mavlink_master, 'timeout', 20, ...
        'fallback_master', mission_config.mavlink_master_fallback, ...
        'allow_simulated_fallback', mission_config.allow_simulated_fallback, ...
        'query_interval_s', mission_config.telemetry_query_interval_s);
    control_cfg = struct('mav', struct('master_connection', mission_config.mavlink_master));
    fprintf('%s[AutoLandingDataCollection] Connecting to vehicle via MAVProxy...\n', log_prefix);

    % Publish landing pad spec topic so downstream consumers can subscribe.
    landing_pad_publish_status = autlPublishLandingPadTopic(mission_config, sessionDir, log_prefix);

    % Arm/takeoff once so vehicle actually moves during collection.
    control_state = struct('arm_sent', false, 'takeoff_sent', false, 'last_control_time', -inf);
    if mission_config.enable_auto_motion
        fprintf('%s[AutoLandingDataCollection] Auto motion control: ENABLED\n', log_prefix);
        fprintf('%s[AutoLandingDataCollection] Auto motion: setting GUIDED mode...\n', log_prefix);
        mode_res = autlMavproxyControl('set_mode', struct('mode', 'GUIDED'), control_cfg);
        if ~mode_res.is_success
            fprintf('%s[AutoLandingDataCollection] Warning: GUIDED mode set failed: %s\n', log_prefix, mode_res.error_message);
        else
            fprintf('%s[AutoLandingDataCollection] Auto motion: GUIDED mode set\n', log_prefix);
        end

        fprintf('%s[AutoLandingDataCollection] Auto motion: arming...\n', log_prefix);
        arm_res = autlMavproxyControl('arm', struct(), control_cfg);
        control_state.arm_sent = arm_res.is_success;
        if ~arm_res.is_success
            fprintf('%s[AutoLandingDataCollection] Warning: arm failed: %s\n', log_prefix, arm_res.error_message);
        else
            fprintf('%s[AutoLandingDataCollection] Auto motion: armed\n', log_prefix);
        end

        if control_state.arm_sent
            fprintf('%s[AutoLandingDataCollection] Auto motion: takeoff %.2f m...\n', log_prefix, mission_config.takeoff_height_m);
            takeoff_res = autlMavproxyControl('takeoff', struct('height', mission_config.takeoff_height_m), control_cfg);
            control_state.takeoff_sent = takeoff_res.is_success;
            if ~takeoff_res.is_success
                fprintf('%s[AutoLandingDataCollection] Warning: takeoff failed: %s\n', log_prefix, takeoff_res.error_message);
            else
                fprintf('%s[AutoLandingDataCollection] Auto motion: takeoff command accepted\n', log_prefix);
                [hover_ok, hover_msg] = autlWaitForHoverState(mav_config, mission_config.takeoff_height_m, 20.0);
                if hover_ok
                    fprintf('%s[AutoLandingDataCollection] Hover stabilization ready: %s\n', log_prefix, hover_msg);
                else
                    fprintf('%s[AutoLandingDataCollection] Warning: hover stabilization timeout: %s\n', log_prefix, hover_msg);
                end
            end
        else
            control_state.takeoff_sent = false;
            fprintf('%s[AutoLandingDataCollection] Warning: skipping takeoff because arming failed\n', log_prefix);
        end
        pause(0.5);
    else
        fprintf('%s[AutoLandingDataCollection] Auto motion control: DISABLED\n', log_prefix);
        control_state = struct('arm_sent', false, 'takeoff_sent', false, 'last_control_time', -inf);
    end
    
    % Begin collection loop
    sample_idx = 0;
    t_last_sample = tic;
    save_interval = max(1, round(mission_config.sample_rate / 2));  % Save every 0.5 seconds
    last_vehicle_state = struct();
    last_vehicle_state.position = [0, 0, 0];
    last_vehicle_state.velocity = [0, 0, 0];
    last_vehicle_state.attitude_euler = [0, 0, 0];
    last_vehicle_state.attitude_quat = [0, 0, 0, 1];
    last_vehicle_state.throttle = 0;
    last_vehicle_state.rotor_speeds = [0, 0, 0, 0];
    last_vehicle_state.gps_fix = 0;
    last_vehicle_state.accel = [0, 0, 0];
    last_vehicle_state.gyro = [0, 0, 0];
    last_vehicle_state.battery_voltage = 0;
    last_vehicle_state.battery_current = 0;
    last_vehicle_state.barometer_alt = 0;
    last_vehicle_state.rangefinder_dist = 0;
    last_vehicle_state.armed = 0;
    last_vehicle_state.mode = 'NO_LINK';
    last_telemetry_query_time = -inf;
    
    effective_telemetry_interval = max(mission_config.telemetry_query_interval_s, 1.0);
    effective_control_interval = max(mission_config.control_interval_s, 2.0);

    while toc(t_start) < mission_config.max_duration
        t_elapsed = toc(t_start);
        
        % Sample timing
        if toc(t_last_sample) >= sample_interval
            sample_idx = sample_idx + 1;
            if sample_idx > max_samples
                break;
            end
            t_last_sample = tic;
            
            % Query telemetry at a lower rate and reuse latest state for high-rate sampling.
            if (t_elapsed - last_telemetry_query_time) >= effective_telemetry_interval
                queried_state = autlGetVehicleState(mav_config);
                if ~(isfield(queried_state, 'mode') && strcmpi(string(queried_state.mode), "NO_LINK"))
                    last_vehicle_state = queried_state;
                end
                last_telemetry_query_time = t_elapsed;
            end
            vehicle_state = last_vehicle_state;
            
            % Store raw data (no processing, no ontology)
            raw_data.timestamp(sample_idx) = t_elapsed;
            raw_data.position_xyz(sample_idx, :) = vehicle_state.position;
            raw_data.velocity_xyz(sample_idx, :) = vehicle_state.velocity;
            raw_data.attitude_rpy(sample_idx, :) = vehicle_state.attitude_euler;
            raw_data.attitude_quat(sample_idx, :) = vehicle_state.attitude_quat;
            raw_data.thrust_percent(sample_idx) = vehicle_state.throttle;
            raw_data.rotor_speeds(sample_idx, :) = vehicle_state.rotor_speeds;
            raw_data.gps_fix(sample_idx) = vehicle_state.gps_fix;
            raw_data.imu_accel_xyz(sample_idx, :) = vehicle_state.accel;
            raw_data.imu_gyro_xyz(sample_idx, :) = vehicle_state.gyro;
            raw_data.battery_voltage(sample_idx) = vehicle_state.battery_voltage;
            raw_data.battery_current(sample_idx) = vehicle_state.battery_current;
            raw_data.barometer_alt(sample_idx) = vehicle_state.barometer_alt;
            raw_data.rangefinder_dist(sample_idx) = vehicle_state.rangefinder_dist;
            raw_data.armed_state(sample_idx) = vehicle_state.armed;
            raw_data.flight_mode(sample_idx) = {vehicle_state.mode};
            raw_data.landing_pad_center_xyz(sample_idx, :) = mission_config.landing_pad_center;
            raw_data.landing_pad_size_xy(sample_idx, :) = mission_config.landing_pad_size;
            dx_pad = vehicle_state.position(1) - mission_config.landing_pad_center(1);
            dy_pad = vehicle_state.position(2) - mission_config.landing_pad_center(2);
            raw_data.landing_pad_distance_xy(sample_idx) = hypot(dx_pad, dy_pad);

            % Feed velocity setpoints so SITL keeps moving around landing pad.
            if mission_config.enable_auto_motion && control_state.takeoff_sent && ...
                    (t_elapsed - control_state.last_control_time) >= effective_control_interval
                control_cmd = autlComputeTrackingVelocity(vehicle_state.position, mission_config);
                vel_struct = struct('vx', control_cmd(1), 'vy', control_cmd(2), 'vz', control_cmd(3), ...
                    'duration', min(0.05, effective_control_interval));
                vel_result = autlMavproxyControl('set_velocity', vel_struct, control_cfg);
                if ~vel_result.is_success && mod(sample_idx, max(1, round(mission_config.sample_rate))) == 0
                    fprintf('%s[AutoLandingDataCollection] Warning: velocity command failed: %s\n', log_prefix, vel_result.error_message);
                elseif vel_result.is_success && mod(sample_idx, max(1, round(2 * mission_config.sample_rate))) == 0
                    fprintf('%s[AutoLandingDataCollection] Motion cmd vx=%.2f vy=%.2f vz=%.2f mode=%s\n', ...
                        log_prefix, control_cmd(1), control_cmd(2), control_cmd(3), char(string(vehicle_state.mode)));
                end
                control_state.last_control_time = t_elapsed;
            end
            
            % Log entry (simple timestamp marker)
            log_entries{end+1} = sprintf('[%.3f] Sample %d: pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]', ...
                t_elapsed, sample_idx, ...
                vehicle_state.position(1), vehicle_state.position(2), vehicle_state.position(3), ...
                vehicle_state.velocity(1), vehicle_state.velocity(2), vehicle_state.velocity(3));
            
            % Update real-time visualization every 10 samples
            if viz_state.enabled && mod(sample_idx, 10) == 0 && isgraphics(viz_state.fig)
                try
                    % Extract current trajectory
                    pos_xyz = raw_data.position_xyz(1:sample_idx, :);
                    vel_norm = vecnorm(raw_data.velocity_xyz(1:sample_idx, :), 2, 2);
                    alt = pos_xyz(:, 3);
                    battery_v = raw_data.battery_voltage(1:sample_idx);

                    set(viz_state.traj_line, 'XData', pos_xyz(:, 1), 'YData', pos_xyz(:, 2), 'ZData', pos_xyz(:, 3));
                    set(viz_state.traj_head, 'XData', pos_xyz(end, 1), 'YData', pos_xyz(end, 2), 'ZData', pos_xyz(end, 3));
                    set(viz_state.alt_line, 'XData', 1:sample_idx, 'YData', alt);
                    set(viz_state.vel_line, 'XData', 1:sample_idx, 'YData', vel_norm);
                    set(viz_state.bat_line, 'XData', 1:sample_idx, 'YData', battery_v);

                    status_text = sprintf(['Session: %s\nSample: %d\nElapsed: %.1fs\nActual Hz: %.2f\n', ...
                        'Mode: %s\nArmed: %d\nPos: [%.2f, %.2f, %.2f]\nVel: [%.2f, %.2f, %.2f]'], ...
                        mission_config.session_id, sample_idx, t_elapsed, sample_idx / max(t_elapsed, 1e-3), ...
                        char(string(vehicle_state.mode)), vehicle_state.armed, ...
                        vehicle_state.position(1), vehicle_state.position(2), vehicle_state.position(3), ...
                        vehicle_state.velocity(1), vehicle_state.velocity(2), vehicle_state.velocity(3));
                    set(viz_state.status_text, 'String', status_text);

                    title(viz_state.alt_ax, sprintf('Altitude (Current: %.2f m)', alt(end)));
                    title(viz_state.vel_ax, sprintf('Velocity Norm (Current: %.2f m/s)', vel_norm(end)));
                    title(viz_state.bat_ax, sprintf('Battery Voltage (Current: %.2f V)', battery_v(end)));

                    drawnow limitrate nocallbacks;
                catch
                    % Silent fail to avoid disrupting collection
                end
            end
            
            % Real-time save: Save collected data every N samples (streaming save)
            if mod(sample_idx, save_interval) == 0
                raw_data.sample_count = sample_idx;
                raw_data_trimmed = autlTrimRawData(raw_data, sample_idx);
                try
                    raw_mat_path = fullfile(sessionDir, 'raw_data.mat');
                    save(raw_mat_path, 'raw_data_trimmed', '-v7.3');
                    fprintf('%s[AutoLandingDataCollection] Checkpoint saved: %d samples\n', log_prefix, sample_idx);
                catch
                    % Silent fail to avoid disrupting collection
                end
            end
        end
        
        % Allow interruption
        pause(0.01);
    end
    
    raw_data.sample_count = sample_idx;
    
    % Trim arrays to actual sample count
    raw_data.timestamp = raw_data.timestamp(1:sample_idx);
    raw_data.position_xyz = raw_data.position_xyz(1:sample_idx, :);
    raw_data.velocity_xyz = raw_data.velocity_xyz(1:sample_idx, :);
    raw_data.attitude_rpy = raw_data.attitude_rpy(1:sample_idx, :);
    raw_data.attitude_quat = raw_data.attitude_quat(1:sample_idx, :);
    raw_data.thrust_percent = raw_data.thrust_percent(1:sample_idx);
    raw_data.rotor_speeds = raw_data.rotor_speeds(1:sample_idx, :);
    raw_data.gps_fix = raw_data.gps_fix(1:sample_idx);
    raw_data.imu_accel_xyz = raw_data.imu_accel_xyz(1:sample_idx, :);
    raw_data.imu_gyro_xyz = raw_data.imu_gyro_xyz(1:sample_idx, :);
    raw_data.battery_voltage = raw_data.battery_voltage(1:sample_idx);
    raw_data.battery_current = raw_data.battery_current(1:sample_idx);
    raw_data.barometer_alt = raw_data.barometer_alt(1:sample_idx);
    raw_data.rangefinder_dist = raw_data.rangefinder_dist(1:sample_idx);
    raw_data.armed_state = raw_data.armed_state(1:sample_idx);
    raw_data.flight_mode = raw_data.flight_mode(1:sample_idx);
    raw_data.landing_pad_center_xyz = raw_data.landing_pad_center_xyz(1:sample_idx, :);
    raw_data.landing_pad_size_xy = raw_data.landing_pad_size_xy(1:sample_idx, :);
    raw_data.landing_pad_distance_xy = raw_data.landing_pad_distance_xy(1:sample_idx);
    
    % Save raw data
    raw_mat_path = fullfile(sessionDir, 'raw_data.mat');
    save(raw_mat_path, 'raw_data');
    
    % Save CSV for inspection
    raw_csv_path = fullfile(sessionDir, 'raw_data.csv');
    autlSaveRawDataCsv(raw_csv_path, raw_data);
    
    % Save metadata
    metadata = struct();
    metadata.session_id = mission_config.session_id;
    metadata.collection_duration = t_elapsed;
    metadata.actual_samples = sample_idx;
    metadata.sample_rate_hz = mission_config.sample_rate;
    metadata.timestamp = datetime('now');
    metadata.mission_config = mission_config;
    metadata.landing_pad_topic_status = landing_pad_publish_status;
    
    metadata_path = fullfile(sessionDir, 'metadata.json');
    autlSaveJson(metadata_path, metadata);
    
    % Build result
    collection_result = struct();
    collection_result.session_id = mission_config.session_id;
    collection_result.session_dir = sessionDir;
    collection_result.raw_data_path = raw_mat_path;
    collection_result.csv_path = raw_csv_path;
    collection_result.metadata_path = metadata_path;
    collection_result.sample_count = sample_idx;
    collection_result.duration = t_elapsed;
    collection_result.status = 'completed';
    collection_result.landing_pad_topic_status = landing_pad_publish_status;

    if mission_config.enable_auto_motion
        autlMavproxyControl('land', struct(), control_cfg);
    end
    
    fprintf('%s[AutoLandingDataCollection] Collected %d samples in %.1f sec (%.1f Hz actual)\n', log_prefix, ...
        sample_idx, t_elapsed, sample_idx / t_elapsed);
    fprintf('%s[AutoLandingDataCollection] Output: %s\n', log_prefix, sessionDir);
    
    % Optionally save visualization snapshot for this scenario.
    if mission_config.enable_visualization && mission_config.save_realtime_viz_snapshot && ...
            ~isempty(fig_handle) && isgraphics(fig_handle)
        try
            viz_file = fullfile(sessionDir, 'realtime_collection_viz.png');
            saveas(fig_handle, viz_file);
            fprintf('%s[AutoLandingDataCollection] Real-time visualization saved: %s\n', log_prefix, viz_file);
        catch
            % Silent fail
        end
    end

    if mission_config.enable_visualization && mission_config.close_visualization_on_finish
        if ~isempty(fig_handle) && isgraphics(fig_handle)
            try
                delete(fig_handle);
            catch
            end
        end
        autl_viz_cache = [];
    end
    
catch ME
    % Check if this is user interruption (Ctrl+C)
    is_interrupt = autlIsUserInterrupt(ME);
    
    if is_interrupt
        fprintf('%s[AutoLandingDataCollection] User interrupted. Saving collected data...\n', log_prefix);
        
        % Save whatever data we have collected so far
        raw_data.sample_count = sample_idx;
        raw_data_trimmed = autlTrimRawData(raw_data, sample_idx);
        
        try
            raw_mat_path = fullfile(sessionDir, 'raw_data.mat');
            save(raw_mat_path, 'raw_data_trimmed', '-v7.3');
            raw_csv_path = fullfile(sessionDir, 'raw_data.csv');
            autlSaveRawDataCsv(raw_csv_path, raw_data_trimmed);
            fprintf('%s[AutoLandingDataCollection] Partial data saved (%d samples)\n', log_prefix, sample_idx);
        catch save_err
            fprintf('%s[AutoLandingDataCollection] Warning: Could not save partial data: %s\n', log_prefix, save_err.message);
        end
        
        collection_result = struct();
        collection_result.status = 'interrupted';
        collection_result.session_id = mission_config.session_id;
        collection_result.session_dir = sessionDir;
        collection_result.raw_data_path = raw_mat_path;
        collection_result.sample_count = sample_idx;
        collection_result.duration = toc(t_start);
        
        fprintf('%s[AutoLandingDataCollection] Collection interrupted. Partial data recovered.\n', log_prefix);
    else
        collection_result = struct();
        collection_result.status = 'error';
        collection_result.error_message = ME.message;
        fprintf('%s[AutoLandingDataCollection] ERROR: %s\n', log_prefix, ME.message);
end

if mission_config.enable_visualization && mission_config.close_visualization_on_finish
    if ~isempty(fig_handle) && isgraphics(fig_handle)
        try
            delete(fig_handle);
        catch
        end
    end
    autl_viz_cache = [];
end
end

function autlCloseStaleRealtimeFigures(keep_fig)
% Delete leaked realtime collection figures so worker iterations do not accumulate GUI handles.

if nargin < 1
    keep_fig = [];
end

try
    figs = findall(0, 'Type', 'figure', 'Tag', 'autl_realtime_collection');
    for i = 1:numel(figs)
        if isempty(keep_fig) || ~isequal(figs(i), keep_fig)
            delete(figs(i));
        end
    end
catch
    % Silent fail
end
end

function vehicle_state = autlGetVehicleState(mav_config)
% Query current vehicle state via pymavlink and return raw sensor values.
% By default, does NOT fake motion when telemetry link is unavailable.

vehicle_state = struct();
persistent cached_state last_query_tic

if ~isfield(mav_config, 'allow_simulated_fallback')
    mav_config.allow_simulated_fallback = false;
end
if ~isfield(mav_config, 'query_interval_s')
    mav_config.query_interval_s = 2.0;
end
if ~isfield(mav_config, 'fallback_master')
    mav_config.fallback_master = '';
end

if isempty(last_query_tic)
    last_query_tic = tic;
end
if ~isempty(cached_state) && toc(last_query_tic) < mav_config.query_interval_s
    vehicle_state = cached_state;
    return;
end

% Try pymavlink with short timeout and port fallback.
try
    masters = {mav_config.master};
    if strlength(string(mav_config.fallback_master)) > 0
        masters{end+1} = char(string(mav_config.fallback_master));
    end
    masters = unique(masters, 'stable');

    status = 1;
    telemetry = struct();
    for i = 1:numel(masters)
        payload = struct('master', masters{i}, 'timeout', 0.35);
        payload_json = jsonencode(payload);
        payload_b64 = matlab.net.base64encode(uint8(payload_json));
        py_code = [ ...
            'import base64,json,sys,time' newline ...
            'from pymavlink import mavutil' newline ...
            'd = json.loads(base64.b64decode("' payload_b64 '").decode())' newline ...
            'm = mavutil.mavlink_connection(d["master"], source_system=250)' newline ...
            'hb = m.wait_heartbeat(timeout=max(0.2,float(d["timeout"])))' newline ...
            'if hb is None:' newline ...
            '  raise RuntimeError("heartbeat timeout")' newline ...
            'msgs = {}' newline ...
            'end_t = time.time() + max(0.2, float(d["timeout"]))' newline ...
            'wanted = {"LOCAL_POSITION_NED","ATTITUDE","VFR_HUD","SYS_STATUS","HEARTBEAT"}' newline ...
            'while time.time() < end_t and len(msgs) < len(wanted):' newline ...
            '  msg = m.recv_match(type=list(wanted - set(msgs.keys())), blocking=True, timeout=0.08)' newline ...
            '  if msg is not None:' newline ...
            '    msgs[msg.get_type()] = msg' newline ...
            'lp = msgs.get("LOCAL_POSITION_NED")' newline ...
            'att = msgs.get("ATTITUDE")' newline ...
            'vfr = msgs.get("VFR_HUD")' newline ...
            'syss = msgs.get("SYS_STATUS")' newline ...
            'hbm = msgs.get("HEARTBEAT", hb)' newline ...
            'mode = mavutil.mode_string_v10(hbm) if hbm else "UNKNOWN"' newline ...
            'out = {' newline ...
            '  "position": [ (lp.x if lp else 0.0), (lp.y if lp else 0.0), (lp.z if lp else 0.0) ],' newline ...
            '  "velocity": [ (lp.vx if lp else 0.0), (lp.vy if lp else 0.0), (lp.vz if lp else 0.0) ],' newline ...
            '  "attitude_euler": [ (att.roll if att else 0.0), (att.pitch if att else 0.0), (att.yaw if att else 0.0) ],' newline ...
            '  "throttle": (vfr.throttle if vfr else 0.0),' newline ...
            '  "gps_fix": int((lp is not None)),' newline ...
            '  "battery_voltage": ((syss.voltage_battery/1000.0) if syss else 0.0),' newline ...
            '  "barometer_alt": (vfr.alt if vfr else 0.0),' newline ...
            '  "armed": int(bool(hbm.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hbm else False),' newline ...
            '  "mode": mode' newline ...
            '}' newline ...
            'print(json.dumps(out))' newline ...
            ];
        cmd = sprintf('python3 - <<''PY''\n%s\nPY', py_code);
        [status_i, output_i] = system(cmd);
        if status_i == 0 && ~isempty(strtrim(output_i))
            try
                telemetry = jsondecode(strtrim(output_i));
                status = 0;
            catch
                status = 1;
            end
        end
        if status == 0
            break;
        end
    end
    
    % If timeout or error, optionally use simulated data
    if status ~= 0
        if ~isempty(cached_state)
            vehicle_state = cached_state;
        elseif mav_config.allow_simulated_fallback
            vehicle_state = autlGenerateSimulatedVehicleState();
        else
            vehicle_state = autlGenerateNoLinkState();
        end
        cached_state = vehicle_state;
        last_query_tic = tic;
        return;
    end
catch
    if ~isempty(cached_state)
        vehicle_state = cached_state;
    elseif mav_config.allow_simulated_fallback
        vehicle_state = autlGenerateSimulatedVehicleState();
    else
        vehicle_state = autlGenerateNoLinkState();
    end
    cached_state = vehicle_state;
    last_query_tic = tic;
    return;
end

vehicle_state.position = [double(telemetry.position(1)), double(telemetry.position(2)), double(telemetry.position(3))];
vehicle_state.velocity = [double(telemetry.velocity(1)), double(telemetry.velocity(2)), double(telemetry.velocity(3))];
vehicle_state.attitude_euler = [double(telemetry.attitude_euler(1)), double(telemetry.attitude_euler(2)), double(telemetry.attitude_euler(3))];
vehicle_state.attitude_quat = [0, 0, 0, 1];
vehicle_state.throttle = double(telemetry.throttle);

% Dummy rotor speeds (would come from extended mode)
vehicle_state.rotor_speeds = [0, 0, 0, 0];

vehicle_state.gps_fix = double(telemetry.gps_fix);

% Dummy IMU data (would come from extended telemetry)
vehicle_state.accel = [0, 0, 9.81];
vehicle_state.gyro = [0, 0, 0];

vehicle_state.battery_voltage = double(telemetry.battery_voltage);
vehicle_state.battery_current = 0;

vehicle_state.barometer_alt = double(telemetry.barometer_alt);

% Dummy rangefinder (would come from extended telemetry)
vehicle_state.rangefinder_dist = 0;

vehicle_state.armed = double(telemetry.armed);
vehicle_state.mode = char(string(telemetry.mode));

cached_state = vehicle_state;
last_query_tic = tic;

function no_link = autlGenerateNoLinkState()
% Return explicit "no telemetry link" state instead of faking movement.

no_link = struct();
no_link.position = [0, 0, 0];
no_link.velocity = [0, 0, 0];
no_link.attitude_euler = [0, 0, 0];
no_link.attitude_quat = [0, 0, 0, 1];
no_link.throttle = 0;
no_link.rotor_speeds = [0, 0, 0, 0];
no_link.gps_fix = 0;
no_link.accel = [0, 0, 0];
no_link.gyro = [0, 0, 0];
no_link.battery_voltage = 0;
no_link.battery_current = 0;
no_link.barometer_alt = 0;
no_link.rangefinder_dist = 0;
no_link.armed = 0;
no_link.mode = 'NO_LINK';
end

function sim_state = autlGenerateSimulatedVehicleState()
% Generate realistic simulated vehicle state for data collection
% When MAVProxy is unavailable, uses physics-based simulation

persistent sim_time last_time
if isempty(sim_time)
    sim_time = 0;
    last_time = tic;
end

% Elapsed time
dt = toc(last_time);
last_time = tic;
sim_time = sim_time + dt;

% Simple drone simulation
t = sim_time;
sim_state = struct();

% Circular trajectory with realistic physics
radius = 5;  % meters
angular_rate = 0.5;  % rad/s
altitude_base = 2;   % meters
altitude_wobble = 0.5 * sin(2*t);

% Position (NED frame, z is down)
x = radius * sin(angular_rate * t);
y = radius * cos(angular_rate * t);
z = -(altitude_base + altitude_wobble);

sim_state.position = [x, y, z];

% Velocity (derivative position)
v_x = radius * angular_rate * cos(angular_rate * t);
v_y = -radius * angular_rate * sin(angular_rate * t);
v_z = -0.5 * 2 * cos(2*t);

sim_state.velocity = [v_x, v_y, v_z];

% Attitude (radiations)
sim_state.attitude_euler = [0.05*sin(t), 0.05*cos(t), atan2(v_y, v_x)];
sim_state.attitude_quat = [0, 0, 0, 1];

% Throttle (ramps up and holds)
if t < 3
    sim_state.throttle = 50 + 16.7 * t;
else
    sim_state.throttle = 50 + max(0, 20 - 2*(t-3));
end

% Dummy rotor speeds
sim_state.rotor_speeds = [800 + 50*sin(t), 800 + 50*cos(t), 800 - 50*sin(t), 800 - 50*cos(t)];

% GPS fix and sensors
sim_state.gps_fix = 1;
sim_state.accel = [0.1*sin(t), 0.1*cos(t), 9.81 - 0.5*sin(2*t)];
sim_state.gyro = [0.01*cos(angular_rate*t), 0.01*sin(angular_rate*t), 0.1];

% Battery (drains slowly)
sim_state.battery_voltage = 12.6 - 0.001*t;
sim_state.battery_current = 10 + 2*sin(t);

% Barometer
sim_state.barometer_alt = altitude_base + altitude_wobble;
sim_state.rangefinder_dist = altitude_base + altitude_wobble;

% Armed state
sim_state.armed = (t > 1);
sim_state.mode = 'GUIDED';
end
end

function autlSaveRawDataCsv(csv_path, raw_data)
% Save raw data to CSV file for easy inspection

try
    fid = fopen(csv_path, 'w');
    % Header
    fprintf(fid, 'timestamp_sec,pos_x_m,pos_y_m,pos_z_m,vel_x_ms,vel_y_ms,vel_z_ms,');
    fprintf(fid, 'roll_rad,pitch_rad,yaw_rad,qx,qy,qz,qw,');
    fprintf(fid, 'thrust_pct,rotor1_rpm,rotor2_rpm,rotor3_rpm,rotor4_rpm,');
    fprintf(fid, 'gps_fix,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,');
    fprintf(fid, 'battery_v,battery_a,baro_alt_m,rangefinder_m,armed,mode,');
    fprintf(fid, 'pad_x_m,pad_y_m,pad_z_m,pad_size_x_m,pad_size_y_m,pad_dist_xy_m\n');
    
    % Data rows
    for i = 1:raw_data.sample_count
        fprintf(fid, '%.3f,', raw_data.timestamp(i));
        fprintf(fid, '%.3f,%.3f,%.3f,', raw_data.position_xyz(i,:));
        fprintf(fid, '%.3f,%.3f,%.3f,', raw_data.velocity_xyz(i,:));
        fprintf(fid, '%.4f,%.4f,%.4f,', raw_data.attitude_rpy(i,:));
        fprintf(fid, '%.4f,%.4f,%.4f,%.4f,', raw_data.attitude_quat(i,:));
        fprintf(fid, '%.1f,%.0f,%.0f,%.0f,%.0f,', raw_data.thrust_percent(i), raw_data.rotor_speeds(i,:));
        fprintf(fid, '%d,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,', raw_data.gps_fix(i), raw_data.imu_accel_xyz(i,:), raw_data.imu_gyro_xyz(i,:));
        fprintf(fid, '%.2f,%.2f,%.2f,%.2f,%d,%s,', raw_data.battery_voltage(i), raw_data.battery_current(i), ...
            raw_data.barometer_alt(i), raw_data.rangefinder_dist(i), raw_data.armed_state(i), char(raw_data.flight_mode{i}));
        fprintf(fid, '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', raw_data.landing_pad_center_xyz(i, 1), ...
            raw_data.landing_pad_center_xyz(i, 2), raw_data.landing_pad_center_xyz(i, 3), ...
            raw_data.landing_pad_size_xy(i, 1), raw_data.landing_pad_size_xy(i, 2), raw_data.landing_pad_distance_xy(i));
    end
    fclose(fid);
catch ME
    warning('Failed to save raw CSV: %s', ME.message);
end

function viz = autlInitRealtimeVizLayout(fig_handle)
% Create once and update line handles each refresh to avoid flicker and dropped updates.

viz = struct();
viz.fig = fig_handle;

viz.traj_ax = subplot(2, 2, 1, 'Parent', fig_handle);
viz.traj_line = plot3(viz.traj_ax, nan, nan, nan, 'b-', 'LineWidth', 1.2);
hold(viz.traj_ax, 'on');
viz.traj_head = plot3(viz.traj_ax, nan, nan, nan, 'ro', 'MarkerSize', 8, 'LineWidth', 1.2);
xlabel(viz.traj_ax, 'X (m)'); ylabel(viz.traj_ax, 'Y (m)'); zlabel(viz.traj_ax, 'Z (m)');
title(viz.traj_ax, '3D Trajectory');
grid(viz.traj_ax, 'on');
viz.status_text = text(viz.traj_ax, 0.02, 0.98, '', 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'BackgroundColor', 'w', 'Margin', 4);

viz.alt_ax = subplot(2, 2, 2, 'Parent', fig_handle);
viz.alt_line = plot(viz.alt_ax, nan, nan, 'g-', 'LineWidth', 1.2);
xlabel(viz.alt_ax, 'Sample'); ylabel(viz.alt_ax, 'Altitude (m)');
title(viz.alt_ax, 'Altitude');
grid(viz.alt_ax, 'on');

viz.vel_ax = subplot(2, 2, 3, 'Parent', fig_handle);
viz.vel_line = plot(viz.vel_ax, nan, nan, 'r-', 'LineWidth', 1.2);
xlabel(viz.vel_ax, 'Sample'); ylabel(viz.vel_ax, 'Velocity (m/s)');
title(viz.vel_ax, 'Velocity Norm');
grid(viz.vel_ax, 'on');

viz.bat_ax = subplot(2, 2, 4, 'Parent', fig_handle);
viz.bat_line = plot(viz.bat_ax, nan, nan, 'm-', 'LineWidth', 1.2);
xlabel(viz.bat_ax, 'Sample'); ylabel(viz.bat_ax, 'Voltage (V)');
title(viz.bat_ax, 'Battery Voltage');
grid(viz.bat_ax, 'on');
end

function viz = autlResetRealtimeVizLayout(viz, session_id)
% Reset line buffers when starting a new scenario while reusing one figure.

if ~isstruct(viz) || ~isfield(viz, 'fig') || ~isgraphics(viz.fig)
    return;
end

set(viz.traj_line, 'XData', nan, 'YData', nan, 'ZData', nan);
set(viz.traj_head, 'XData', nan, 'YData', nan, 'ZData', nan);
set(viz.alt_line, 'XData', nan, 'YData', nan);
set(viz.vel_line, 'XData', nan, 'YData', nan);
set(viz.bat_line, 'XData', nan, 'YData', nan);
set(viz.status_text, 'String', sprintf('Session: %s\nWaiting for samples...', session_id));
title(viz.traj_ax, sprintf('3D Trajectory (%s)', session_id), 'Interpreter', 'none');
end
end

function raw_data_trimmed = autlTrimRawData(raw_data, num_samples)
% Trim raw data arrays to actual sample count

raw_data_trimmed = raw_data;
raw_data_trimmed.timestamp = raw_data.timestamp(1:num_samples);
raw_data_trimmed.position_xyz = raw_data.position_xyz(1:num_samples, :);
raw_data_trimmed.velocity_xyz = raw_data.velocity_xyz(1:num_samples, :);
raw_data_trimmed.attitude_rpy = raw_data.attitude_rpy(1:num_samples, :);
raw_data_trimmed.attitude_quat = raw_data.attitude_quat(1:num_samples, :);
raw_data_trimmed.thrust_percent = raw_data.thrust_percent(1:num_samples);
raw_data_trimmed.rotor_speeds = raw_data.rotor_speeds(1:num_samples, :);
raw_data_trimmed.gps_fix = raw_data.gps_fix(1:num_samples);
raw_data_trimmed.imu_accel_xyz = raw_data.imu_accel_xyz(1:num_samples, :);
raw_data_trimmed.imu_gyro_xyz = raw_data.imu_gyro_xyz(1:num_samples, :);
raw_data_trimmed.battery_voltage = raw_data.battery_voltage(1:num_samples);
raw_data_trimmed.battery_current = raw_data.battery_current(1:num_samples);
raw_data_trimmed.barometer_alt = raw_data.barometer_alt(1:num_samples);
raw_data_trimmed.rangefinder_dist = raw_data.rangefinder_dist(1:num_samples);
raw_data_trimmed.armed_state = raw_data.armed_state(1:num_samples);
raw_data_trimmed.flight_mode = raw_data.flight_mode(1:num_samples);
raw_data_trimmed.landing_pad_center_xyz = raw_data.landing_pad_center_xyz(1:num_samples, :);
raw_data_trimmed.landing_pad_size_xy = raw_data.landing_pad_size_xy(1:num_samples, :);
raw_data_trimmed.landing_pad_distance_xy = raw_data.landing_pad_distance_xy(1:num_samples);
raw_data_trimmed.sample_count = num_samples;
end

function vel_cmd = autlComputeTrackingVelocity(position_xyz, mission_config)
% Blend approach-to-pad and orbit terms to keep motion rich during data collection.

pad_center = double(mission_config.landing_pad_center(:)');
dx = pad_center(1) - double(position_xyz(1));
dy = pad_center(2) - double(position_xyz(2));
dist_xy = hypot(dx, dy);

kp_xy = 0.32;
k_orbit = 0.42;
vx = kp_xy * dx - k_orbit * dy;
vy = kp_xy * dy + k_orbit * dx;

% Force visible lateral movement even near the pad center.
if dist_xy < 0.35
    vx = 0.55;
    vy = 0.00;
end

target_alt_ned = -abs(double(mission_config.takeoff_height_m));
kz = 0.40;
vz = kz * (target_alt_ned - double(position_xyz(3)));

vx = autlClamp(vx, -1.2, 1.2);
vy = autlClamp(vy, -1.2, 1.2);
vz = autlClamp(vz, -0.6, 0.6);
vel_cmd = [vx, vy, vz];
end

function status = autlPublishLandingPadTopic(mission_config, sessionDir, log_prefix)
% Publish landing pad center/size as ROS2 topic if available and always save local JSON.

status = struct();
status.ok = false;
status.topic = string(mission_config.landing_pad_topic);
status.backend = "none";
status.message = "not published";

payload = single([mission_config.landing_pad_center(:); mission_config.landing_pad_size(:)]);

try
    rosCtx = autlCreateRosContext("autolanding_pad_pub");
    if isstruct(rosCtx) && isfield(rosCtx, 'enabled') && rosCtx.enabled
        pub = ros2publisher(rosCtx.node, char(mission_config.landing_pad_topic), 'std_msgs/Float32MultiArray');
        msg = ros2message(pub);
        msg.data = payload;
        send(pub, msg);
        status.ok = true;
        status.backend = "ros2";
        status.message = "published";
        fprintf('%s[AutoLandingDataCollection] Landing pad topic published: %s\n', log_prefix, char(status.topic));
        autlReleaseRosContext(rosCtx);
    else
        status.message = "ros2 unavailable";
    end
catch ME
    status.message = string(ME.message);
end

% Always persist spec for downstream consumers, even if ROS2 is unavailable.
pad_spec = struct();
pad_spec.topic = mission_config.landing_pad_topic;
pad_spec.center_xyz = mission_config.landing_pad_center;
pad_spec.size_xy = mission_config.landing_pad_size;
pad_spec.timestamp = char(datetime('now'));
autlSaveJson(fullfile(sessionDir, 'landing_pad_spec.json'), pad_spec);
end

function is_interrupt = autlIsUserInterrupt(ME)
% Check if exception is user interruption (Ctrl+C)

is_interrupt = false;
if nargin < 1 || isempty(ME)
    return;
end

try
    id = lower(string(ME.identifier));
catch
    id = "";
end

try
    msg = lower(string(ME.message));
catch
    msg = "";
end

is_interrupt = contains(id, "operationterminatedbyuser") || ...
    contains(id, "interrupted") || ...
    contains(msg, "operation terminated by user") || ...
    contains(msg, "operation terminated") || ...
    contains(msg, "terminated by user") || ...
    contains(msg, "interrupted") || ...
    contains(msg, "ctrl+c");

if is_interrupt
    return;
end

% Check nested causes
try
    causes = ME.cause;
    for i = 1:numel(causes)
        if autlIsUserInterrupt(causes{i})
            is_interrupt = true;
            return;
        end
    end
catch
end
end

function autlDataCollectionCleanup(collection_state)
% Cleanup handler called when collection ends (normally or via interrupt)
% Ensures final save of partial data if collection was interrupted

try
    if ~isempty(collection_state) && isfield(collection_state, 'session_dir')
        % This function is called automatically by onCleanup
        % Additional cleanup can be added here if needed
    end
catch
    % Silent fail to avoid interfering with cleanup process
end
end

function [ok, msg] = autlWaitForHoverState(mav_config, target_alt_m, timeout_s)
% Wait until the vehicle reaches and briefly holds near target altitude.

ok = false;
msg = '';
if nargin < 3
    timeout_s = 20.0;
end

alt_tol_m = 0.6;
vz_tol_ms = 0.5;
stable_required_s = 1.5;

t0 = tic;
stable_t0 = -inf;
last_alt = nan;
last_vz = nan;
last_mode = "UNKNOWN";

while toc(t0) < timeout_s
    st = autlGetVehicleState(mav_config);
    last_mode = string(st.mode);
    if strcmpi(last_mode, "NO_LINK")
        pause(0.2);
        continue;
    end

    alt_m = abs(double(st.position(3)));
    vz_ms = abs(double(st.velocity(3)));
    last_alt = alt_m;
    last_vz = vz_ms;

    near_alt = abs(alt_m - target_alt_m) <= alt_tol_m;
    near_hover = vz_ms <= vz_tol_ms;
    if near_alt && near_hover
        if stable_t0 < 0
            stable_t0 = toc(t0);
        elseif (toc(t0) - stable_t0) >= stable_required_s
            ok = true;
            msg = sprintf('alt=%.2fm vz=%.2fm/s mode=%s', alt_m, vz_ms, char(last_mode));
            return;
        end
    else
        stable_t0 = -inf;
    end

    pause(0.2);
end

if isnan(last_alt)
    msg = sprintf('no telemetry link within %.1fs', timeout_s);
else
    msg = sprintf('last alt=%.2fm vz=%.2fm/s mode=%s', last_alt, last_vz, char(last_mode));
end
end

% End of main autlRunDataCollection function
end
