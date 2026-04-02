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
if ~isfield(mission_config, 'require_mavlink_for_auto_motion')
    mission_config.require_mavlink_for_auto_motion = true;
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
if ~isfield(mission_config, 'landing_pad_follow_topic')
    mission_config.landing_pad_follow_topic = true;
end
if ~isfield(mission_config, 'landing_pad_follow_interval_s')
    mission_config.landing_pad_follow_interval_s = 0.2;
end
if ~isfield(mission_config, 'landing_pad_apply_set_pose')
    mission_config.landing_pad_apply_set_pose = true;
end
if ~isfield(mission_config, 'landing_pad_model_name')
    mission_config.landing_pad_model_name = 'aruco_landing_box';
end
if ~isfield(mission_config, 'landing_pad_enable_default_motion')
    mission_config.landing_pad_enable_default_motion = true;
end
if ~isfield(mission_config, 'landing_pad_motion_radius_m')
    mission_config.landing_pad_motion_radius_m = 0.35;
end
if ~isfield(mission_config, 'landing_pad_motion_rate_rad_s')
    mission_config.landing_pad_motion_rate_rad_s = 0.45;
end
if ~isfield(mission_config, 'allow_simulated_fallback')
    mission_config.allow_simulated_fallback = false;
end
if ~isfield(mission_config, 'telemetry_query_interval_s')
    mission_config.telemetry_query_interval_s = 5.0;
end
if ~isfield(mission_config, 'mavlink_precheck_timeout_s')
    mission_config.mavlink_precheck_timeout_s = 12.0;
end
if ~isfield(mission_config, 'mavlink_master')
    mission_config.mavlink_master = 'tcp:127.0.0.1:5762';
end
if ~isfield(mission_config, 'mavlink_master_fallback')
    mission_config.mavlink_master_fallback = 'tcp:127.0.0.1:5760';
end
if ~isfield(mission_config, 'motion_profile')
    mission_config.motion_profile = 'balanced';
end
if ~isfield(mission_config, 'motion_gain_xy')
    mission_config.motion_gain_xy = 0.32;
end
if ~isfield(mission_config, 'motion_gain_orbit')
    mission_config.motion_gain_orbit = 0.42;
end
if ~isfield(mission_config, 'motion_min_move_ms')
    mission_config.motion_min_move_ms = 0.55;
end
if ~isfield(mission_config, 'motion_alt_gain')
    mission_config.motion_alt_gain = 0.40;
end
if ~isfield(mission_config, 'motion_vxy_limit_ms')
    mission_config.motion_vxy_limit_ms = 1.2;
end
if ~isfield(mission_config, 'motion_vz_limit_ms')
    mission_config.motion_vz_limit_ms = 0.6;
end
if ~isfield(mission_config, 'drone_set_pose_fallback_enabled')
    mission_config.drone_set_pose_fallback_enabled = true;
end
if ~isfield(mission_config, 'drone_set_pose_interval_s')
    mission_config.drone_set_pose_interval_s = 0.25;
end
if ~isfield(mission_config, 'drone_set_pose_radius_m')
    mission_config.drone_set_pose_radius_m = 0.8;
end
if ~isfield(mission_config, 'drone_set_pose_rate_rad_s')
    mission_config.drone_set_pose_rate_rad_s = 0.45;
end
if ~isfield(mission_config, 'drone_set_pose_takeoff_height_m')
    mission_config.drone_set_pose_takeoff_height_m = mission_config.takeoff_height_m;
end
if ~isfield(mission_config, 'drone_set_pose_climb_rate_mps')
    mission_config.drone_set_pose_climb_rate_mps = 0.6;
end
if ~isfield(mission_config, 'drone_set_pose_model_name')
    if isfield(mission_config, 'reset_model_name') && strlength(string(mission_config.reset_model_name)) > 0
        mission_config.drone_set_pose_model_name = char(string(mission_config.reset_model_name));
    else
        mission_config.drone_set_pose_model_name = 'iris_with_gimbal';
    end
end
if ~isfield(mission_config, 'reset_spawn_z_m')
    mission_config.reset_spawn_z_m = 0.195;
end
if ~isfield(mission_config, 'reset_spawn_xy') || numel(mission_config.reset_spawn_xy) < 2
    mission_config.reset_spawn_xy = mission_config.landing_pad_center(1:2);
end
if ~isfield(mission_config, 'reset_spawn_yaw_deg')
    mission_config.reset_spawn_yaw_deg = 0.0;
end

log_prefix = char(string(mission_config.log_prefix));
flow_log_file = '';
if isfield(mission_config, 'flow_log_file')
    flow_log_file = char(string(mission_config.flow_log_file));
end
flow_ctx = struct();
if isfield(mission_config, 'flow_context') && isstruct(mission_config.flow_context)
    flow_ctx = mission_config.flow_context;
end

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
fprintf('%s[AutoLandingDataCollection] Motion Profile: %s (k_xy=%.2f, k_orbit=%.2f, min_move=%.2f)\n', ...
    log_prefix, char(string(mission_config.motion_profile)), mission_config.motion_gain_xy, mission_config.motion_gain_orbit, mission_config.motion_min_move_ms);
autlFlowLog(flow_log_file, 'autlRunDataCollection', 'collection_started', autlFlowMerge(flow_ctx, struct( ...
    'session_id', mission_config.session_id, 'sample_rate', mission_config.sample_rate, ...
    'max_duration_s', mission_config.max_duration, 'output_dir', sessionDir)));

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
    control_cfg.flow_log_file = flow_log_file;
    control_cfg.flow_context = autlFlowMerge(flow_ctx, struct('module_scope', 'autlRunDataCollection'));
    control_cfg.flow_log_all_actions = false;
    fprintf('%s[AutoLandingDataCollection] Connecting to vehicle via MAVProxy...\n', log_prefix);

    % Publish landing pad spec topic so downstream consumers can subscribe.
    landing_pad_publish_status = autlPublishLandingPadTopic(mission_config, sessionDir, log_prefix);
    landing_pad_runtime = autlInitLandingPadRuntime(mission_config, log_prefix);
    autlFlowLog(flow_log_file, 'autlRunDataCollection', 'landing_pad_runtime_ready', autlFlowMerge(flow_ctx, struct( ...
        'publish_ok', logical(landing_pad_publish_status.ok), 'publish_message', char(string(landing_pad_publish_status.message)), ...
        'follow_enabled', logical(landing_pad_runtime.follow_enabled), 'apply_set_pose', logical(landing_pad_runtime.apply_set_pose))));

    % Arm/takeoff once so vehicle actually moves during collection.
    control_state = struct('arm_sent', false, 'takeoff_sent', false, 'last_control_time', -inf);
    if mission_config.enable_auto_motion
        if mission_config.require_mavlink_for_auto_motion
            [precheck_res, boot_msg] = autlWaitForMavlinkPrecheckTop(mission_config, control_cfg);

            if ~precheck_res.is_success
                mission_config.enable_auto_motion = false;
                precheck_msg = autlCompactMavErrorTop(precheck_res.error_message);
                fprintf('%s[AutoLandingDataCollection] Warning: MAVLink precheck failed, auto motion disabled for this run: %s\n', ...
                    log_prefix, precheck_msg);
                autlFlowLog(flow_log_file, 'autlRunDataCollection', 'mavlink_precheck_failed', autlFlowMerge(flow_ctx, struct( ...
                    'message', precheck_msg, 'fallback_bootstrap', char(string(boot_msg)))));
                if exist('boot_msg', 'var') && strlength(string(boot_msg)) > 0 && ~strcmp(string(boot_msg), "not_used")
                    fprintf('%s[AutoLandingDataCollection] Info: fallback bootstrap result: %s\n', log_prefix, char(string(boot_msg)));
                end
            end
        end
    end

    if mission_config.enable_auto_motion
        fprintf('%s[AutoLandingDataCollection] Auto motion control: ENABLED\n', log_prefix);
        fprintf('%s[AutoLandingDataCollection] Auto motion: setting GUIDED mode...\n', log_prefix);
        mode_res = autlMavproxyControl('set_mode', struct('mode', 'GUIDED'), control_cfg);
        if ~mode_res.is_success
            fprintf('%s[AutoLandingDataCollection] Warning: GUIDED mode set failed: %s\n', log_prefix, ...
                autlCompactMavErrorTop(mode_res.error_message));
        else
            fprintf('%s[AutoLandingDataCollection] Auto motion: GUIDED mode set\n', log_prefix);
        end

        fprintf('%s[AutoLandingDataCollection] Auto motion: arming...\n', log_prefix);
        arm_res = autlMavproxyControl('arm', struct(), control_cfg);
        control_state.arm_sent = arm_res.is_success;
        if ~arm_res.is_success
            fprintf('%s[AutoLandingDataCollection] Warning: arm failed: %s\n', log_prefix, ...
                autlCompactMavErrorTop(arm_res.error_message));
        else
            fprintf('%s[AutoLandingDataCollection] Auto motion: armed\n', log_prefix);
        end

        if control_state.arm_sent
            fprintf('%s[AutoLandingDataCollection] Auto motion: takeoff %.2f m...\n', log_prefix, mission_config.takeoff_height_m);
            takeoff_res = autlMavproxyControl('takeoff', struct('height', mission_config.takeoff_height_m), control_cfg);
            control_state.takeoff_sent = takeoff_res.is_success;
            if ~takeoff_res.is_success
                fprintf('%s[AutoLandingDataCollection] Warning: takeoff failed: %s\n', log_prefix, ...
                    autlCompactMavErrorTop(takeoff_res.error_message));
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
        autlFlowLog(flow_log_file, 'autlRunDataCollection', 'auto_motion_disabled', autlFlowMerge(flow_ctx, struct('reason', 'precheck_or_config')));
        control_state = struct('arm_sent', false, 'takeoff_sent', false, 'last_control_time', -inf);
    end
    
    % Begin collection loop
    sample_idx = 0;
    t_last_sample = tic;
    elapsed_s = 0;  % Initialize so it's defined even if loop exits early
    collection_elapsed_s = 0;
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
    last_vehicle_state.has_local_position = 0;
    last_telemetry_query_time = -inf;
    
    effective_telemetry_interval = max(mission_config.telemetry_query_interval_s, 1.0);
    effective_control_interval = max(mission_config.control_interval_s, 2.0);
    flow_flags = struct('no_link_logged', false);
    try
        drone_fallback_runtime = autlInitDroneFallbackRuntime(mission_config, log_prefix);
    catch ME_fallback_init
        drone_fallback_runtime = struct('enabled', false);
        fprintf('%s[AutoLandingDataCollection] Warning: drone fallback init unavailable, disabled this scenario: %s\n', ...
            log_prefix, ME_fallback_init.message);
    end

    while toc(t_start) < mission_config.max_duration
        elapsed_s = toc(t_start);
        collection_elapsed_s = elapsed_s;
        
        % Sample timing
        if toc(t_last_sample) >= sample_interval
            sample_idx = sample_idx + 1;
            if sample_idx > max_samples
                break;
            end
            t_last_sample = tic;
            
            % Query telemetry at a lower rate and reuse latest state for high-rate sampling.
            if (elapsed_s - last_telemetry_query_time) >= effective_telemetry_interval
                queried_state = autlGetVehicleState(mav_config);
                if ~(isfield(queried_state, 'mode') && strcmpi(string(queried_state.mode), "NO_LINK"))
                    last_vehicle_state = queried_state;
                end
                last_telemetry_query_time = elapsed_s;
            end
            vehicle_state = last_vehicle_state;
            if isfield(vehicle_state, 'mode') && strcmpi(char(string(vehicle_state.mode)), 'NO_LINK')
                if ~flow_flags.no_link_logged
                    autlFlowLog(flow_log_file, 'autlRunDataCollection', 'telemetry_no_link', autlFlowMerge(flow_ctx, struct( ...
                        'sample_idx', sample_idx, 'elapsed_s', elapsed_s)));
                    flow_flags.no_link_logged = true;
                end
            else
                flow_flags.no_link_logged = false;
            end

            [mission_config, landing_pad_runtime] = autlUpdateLandingPadRuntime( ...
                mission_config, landing_pad_runtime, elapsed_s, log_prefix);
            
            % Store raw data (no processing, no ontology)
            raw_data.timestamp(sample_idx) = elapsed_s;
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
                    (elapsed_s - control_state.last_control_time) >= effective_control_interval
                if isfield(vehicle_state, 'has_local_position') && vehicle_state.has_local_position > 0
                    control_cmd = autlComputeTrackingVelocity(vehicle_state.position, mission_config);
                    vel_struct = struct('vx', control_cmd(1), 'vy', control_cmd(2), 'vz', control_cmd(3), ...
                        'duration', min(0.05, effective_control_interval));
                    vel_result = autlMavproxyControl('set_velocity', vel_struct, control_cfg);
                    if ~vel_result.is_success && mod(sample_idx, max(1, round(mission_config.sample_rate))) == 0
                        fprintf('%s[AutoLandingDataCollection] Warning: velocity command failed: %s\n', log_prefix, ...
                            autlCompactMavErrorTop(vel_result.error_message));
                    elseif vel_result.is_success && mod(sample_idx, max(1, round(2 * mission_config.sample_rate))) == 0
                        fprintf('%s[AutoLandingDataCollection] Motion cmd vx=%.2f vy=%.2f vz=%.2f mode=%s\n', ...
                            log_prefix, control_cmd(1), control_cmd(2), control_cmd(3), char(string(vehicle_state.mode)));
                    end
                elseif mod(sample_idx, max(1, round(mission_config.sample_rate))) == 0
                    fprintf('%s[AutoLandingDataCollection] Warning: LOCAL_POSITION_NED unavailable, skipping velocity command this cycle.\n', log_prefix);
                end
                control_state.last_control_time = elapsed_s;
            end

            if (~mission_config.enable_auto_motion || ~control_state.takeoff_sent)
                [mission_config, drone_fallback_runtime] = autlApplyDroneFallbackMotion( ...
                    mission_config, drone_fallback_runtime, elapsed_s, log_prefix);
            end
            
            % Log entry (simple timestamp marker)
            log_entries{end+1} = sprintf('[%.3f] Sample %d: pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]', ...
                elapsed_s, sample_idx, ...
                vehicle_state.position(1), vehicle_state.position(2), vehicle_state.position(3), ...
                vehicle_state.velocity(1), vehicle_state.velocity(2), vehicle_state.velocity(3));
            
            % Update real-time visualization every 10 samples
            if viz_state.enabled && mod(sample_idx, 10) == 0 && isgraphics(viz_state.fig)
                try
                    viz_state = autlEnsureRealtimeVizState(viz_state, mission_config.session_id);
                    if ~viz_state.enabled
                        continue;
                    end

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
                        'Mode: %s\nArmed: %d\nLocalPos: %d\nPos: [%.2f, %.2f, %.2f]\nVel: [%.2f, %.2f, %.2f]'], ...
                        mission_config.session_id, sample_idx, elapsed_s, sample_idx / max(elapsed_s, 1e-3), ...
                        char(string(vehicle_state.mode)), vehicle_state.armed, ...
                        double(vehicle_state.has_local_position), ...
                        vehicle_state.position(1), vehicle_state.position(2), vehicle_state.position(3), ...
                        vehicle_state.velocity(1), vehicle_state.velocity(2), vehicle_state.velocity(3));
                    set(viz_state.status_text, 'String', status_text);

                    title(viz_state.alt_ax, sprintf('Altitude (Current: %.2f m)', alt(end)));
                    title(viz_state.vel_ax, sprintf('Velocity Norm (Current: %.2f m/s)', vel_norm(end)));
                    title(viz_state.bat_ax, sprintf('Battery Voltage (Current: %.2f V)', battery_v(end)));

                    % Keep trajectory visually readable even when motion is very small.
                    if size(pos_xyz, 1) >= 2
                        span_x = max(pos_xyz(:, 1)) - min(pos_xyz(:, 1));
                        span_y = max(pos_xyz(:, 2)) - min(pos_xyz(:, 2));
                        span_z = max(pos_xyz(:, 3)) - min(pos_xyz(:, 3));
                        if max([span_x, span_y, span_z]) < 0.05
                            cx = pos_xyz(end, 1);
                            cy = pos_xyz(end, 2);
                            cz = pos_xyz(end, 3);
                            axis(viz_state.traj_ax, [cx-1, cx+1, cy-1, cy+1, cz-1, cz+1]);
                        else
                            axis(viz_state.traj_ax, 'auto');
                        end
                    end

                    drawnow limitrate nocallbacks;
                catch ME_viz
                    fprintf('%s[AutoLandingDataCollection] Visualization update warning: %s\n', log_prefix, ME_viz.message);
                    viz_state.enabled = false;
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
                    if mod(sample_idx, 250) == 0
                        autlFlowLog(flow_log_file, 'autlRunDataCollection', 'checkpoint_saved', autlFlowMerge(flow_ctx, struct( ...
                            'sample_idx', sample_idx, 'elapsed_s', elapsed_s)));
                    end
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
    metadata.collection_duration = collection_elapsed_s;
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
    collection_result.duration = collection_elapsed_s;
    collection_result.status = 'completed';
    collection_result.landing_pad_topic_status = landing_pad_publish_status;
    autlReleaseLandingPadRuntime(landing_pad_runtime);
    autlReleaseDroneFallbackRuntime(drone_fallback_runtime);
    autlFlowLog(flow_log_file, 'autlRunDataCollection', 'collection_completed', autlFlowMerge(flow_ctx, struct( ...
        'sample_count', sample_idx, 'duration_s', collection_elapsed_s, 'session_dir', sessionDir)));

    if mission_config.enable_auto_motion
        autlMavproxyControl('land', struct(), control_cfg);
    end
    
    fprintf('%s[AutoLandingDataCollection] Collected %d samples in %.1f sec (%.1f Hz actual)\n', log_prefix, ...
        sample_idx, collection_elapsed_s, sample_idx / max(collection_elapsed_s, 1e-3));
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
        autlFlowLog(flow_log_file, 'autlRunDataCollection', 'collection_interrupted', autlFlowMerge(flow_ctx, struct( ...
            'sample_count', sample_idx, 'duration_s', collection_result.duration)));
        
        fprintf('%s[AutoLandingDataCollection] Collection interrupted. Partial data recovered.\n', log_prefix);
    else
        collection_result = struct();
        collection_result.status = 'error';
        collection_result.error_message = ME.message;
        fprintf('%s[AutoLandingDataCollection] ERROR: %s\n', log_prefix, ME.message);
        autlFlowLog(flow_log_file, 'autlRunDataCollection', 'collection_error', autlFlowMerge(flow_ctx, struct( ...
            'error', ME.message, 'sample_count', sample_idx)));
end

try
    if exist('landing_pad_runtime', 'var')
        autlReleaseLandingPadRuntime(landing_pad_runtime);
    end
catch
end

try
    if exist('drone_fallback_runtime', 'var')
        autlReleaseDroneFallbackRuntime(drone_fallback_runtime);
    end
catch
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
persistent cached_state last_query_tic failed_masters failed_until_s

if ~isfield(mav_config, 'allow_simulated_fallback')
    mav_config.allow_simulated_fallback = false;
end
if ~isfield(mav_config, 'query_interval_s')
    mav_config.query_interval_s = 2.0;
end
if ~isfield(mav_config, 'fallback_master')
    mav_config.fallback_master = '';
end
if ~isfield(mav_config, 'use_fallback_master')
    mav_config.use_fallback_master = false;
end

if isempty(last_query_tic)
    last_query_tic = tic;
end
if isempty(failed_masters)
    failed_masters = {};
    failed_until_s = [];
end
if ~isempty(cached_state) && toc(last_query_tic) < mav_config.query_interval_s
    vehicle_state = cached_state;
    return;
end

% Try pymavlink with short timeout and port fallback.
try
    masters = {mav_config.master};
    if mav_config.use_fallback_master && strlength(string(mav_config.fallback_master)) > 0
        masters{end+1} = char(string(mav_config.fallback_master));
    end
    masters = unique(masters, 'stable');

    status = 1;
    telemetry = struct();
    now_s = posixtime(datetime('now'));
    for i = 1:numel(masters)
        master_i = masters{i};
        fail_idx = find(strcmp(failed_masters, master_i), 1);
        if ~isempty(fail_idx) && failed_until_s(fail_idx) > now_s
            continue;
        end

        payload = struct('master', masters{i}, 'timeout', 0.35);
        payload_json = jsonencode(payload);
        payload_b64 = matlab.net.base64encode(uint8(payload_json));
        py_code = [ ...
            'import base64,json,sys,time' newline ...
            'import contextlib,io' newline ...
            'from pymavlink import mavutil' newline ...
            'd = json.loads(base64.b64decode("' payload_b64 '").decode())' newline ...
            '_mav_stdout = io.StringIO()' newline ...
            '_mav_stderr = io.StringIO()' newline ...
            'with contextlib.redirect_stdout(_mav_stdout), contextlib.redirect_stderr(_mav_stderr):' newline ...
            '  m = mavutil.mavlink_connection(d["master"], source_system=250, autoreconnect=False)' newline ...
            '  hb = m.wait_heartbeat(timeout=max(0.2,float(d["timeout"])))' newline ...
            'if hb is None:' newline ...
            '  raise RuntimeError("heartbeat timeout")' newline ...
            'try:' newline ...
            '  m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20, 1)' newline ...
            '  m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)' newline ...
            'except Exception:' newline ...
            '  pass' newline ...
            'msgs = {}' newline ...
            'end_t = time.time() + max(0.2, float(d["timeout"]))' newline ...
            'wanted = {"LOCAL_POSITION_NED","ATTITUDE","VFR_HUD","SYS_STATUS","HEARTBEAT","GLOBAL_POSITION_INT"}' newline ...
            'while time.time() < end_t and len(msgs) < len(wanted):' newline ...
            '  msg = m.recv_match(type=list(wanted - set(msgs.keys())), blocking=True, timeout=0.08)' newline ...
            '  if msg is not None:' newline ...
            '    msgs[msg.get_type()] = msg' newline ...
            'lp = msgs.get("LOCAL_POSITION_NED")' newline ...
            'att = msgs.get("ATTITUDE")' newline ...
            'vfr = msgs.get("VFR_HUD")' newline ...
            'syss = msgs.get("SYS_STATUS")' newline ...
            'gpos = msgs.get("GLOBAL_POSITION_INT")' newline ...
            'hbm = msgs.get("HEARTBEAT", hb)' newline ...
            'mode = mavutil.mode_string_v10(hbm) if hbm else "UNKNOWN"' newline ...
            'has_lp = (lp is not None)' newline ...
            'alt_m = (vfr.alt if vfr else 0.0)' newline ...
            'z_ned = (lp.z if has_lp else -float(alt_m))' newline ...
            'gps_fix = int((getattr(gpos, "fix_type", 0) >= 3) if gpos else has_lp)' newline ...
            'out = {' newline ...
            '  "position": [ (lp.x if has_lp else 0.0), (lp.y if has_lp else 0.0), z_ned ],' newline ...
            '  "velocity": [ (lp.vx if has_lp else 0.0), (lp.vy if has_lp else 0.0), (lp.vz if has_lp else 0.0) ],' newline ...
            '  "attitude_euler": [ (att.roll if att else 0.0), (att.pitch if att else 0.0), (att.yaw if att else 0.0) ],' newline ...
            '  "throttle": (vfr.throttle if vfr else 0.0),' newline ...
            '  "gps_fix": gps_fix,' newline ...
            '  "battery_voltage": ((syss.voltage_battery/1000.0) if syss else 0.0),' newline ...
            '  "barometer_alt": alt_m,' newline ...
            '  "armed": int(bool(hbm.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hbm else False),' newline ...
            '  "mode": mode,' newline ...
            '  "has_local_position": int(has_lp)' newline ...
            '}' newline ...
            'print(json.dumps(out))' newline ...
            ];
        cmd = sprintf('python3 - <<''PY''\n%s\nPY', py_code);
        [status_i, output_i] = system(cmd);
        if status_i == 0 && ~isempty(strtrim(output_i))
            try
                telemetry = jsondecode(strtrim(output_i));
                status = 0;
                if ~isempty(fail_idx)
                    failed_until_s(fail_idx) = 0;
                end
            catch
                status = 1;
            end
        else
            if isempty(fail_idx)
                failed_masters{end+1} = master_i; %#ok<AGROW>
                failed_until_s(end+1) = now_s + 8.0; %#ok<AGROW>
            else
                failed_until_s(fail_idx) = now_s + 8.0;
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
if isfield(telemetry, 'has_local_position')
    vehicle_state.has_local_position = double(telemetry.has_local_position);
else
    vehicle_state.has_local_position = 0;
end

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
no_link.has_local_position = 0;
end

function msg = autlCompactMavError(raw_msg)
% Compact pymavlink/mavproxy multiline stderr into one short line.

msg = strtrim(string(raw_msg));
if strlength(msg) == 0
    msg = "unknown";
    return;
end

parts = regexp(char(msg), '\\r?\\n', 'split');
parts = parts(~cellfun('isempty', strtrim(parts)));
if isempty(parts)
    msg = "unknown";
    return;
end

keep = strings(0, 1);
for i = 1:numel(parts)
    p = strtrim(string(parts{i}));
    p_low = lower(p);
    if contains(p_low, "eof") && contains(p_low, "tcp socket")
        continue;
    end
    if contains(p_low, "connection refused sleeping") || contains(p_low, "connection refused")
        continue;
    end
    if contains(p_low, "connection reset by peer") || contains(p_low, "connection timed out")
        continue;
    end
    if contains(p_low, "socket closed")
        continue;
    end
    keep(end+1) = p; %#ok<AGROW>
end

if isempty(keep)
    msg = "heartbeat timeout";
else
    msg = regexprep(char(keep(end)), '\\s+', ' ');
end
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
sim_state.has_local_position = 1;
end
end

function msg = autlCompactMavErrorTop(raw_msg)
% Compact pymavlink/mavproxy multiline stderr into one short line.

msg = strtrim(string(raw_msg));
if strlength(msg) == 0
    msg = "unknown";
    return;
end

parts = regexp(char(msg), '\\r?\\n', 'split');
parts = parts(~cellfun('isempty', strtrim(parts)));
if isempty(parts)
    msg = "unknown";
    return;
end

keep = strings(0, 1);
for i = 1:numel(parts)
    p = strtrim(string(parts{i}));
    p_low = lower(p);
    if contains(p_low, "eof") && contains(p_low, "tcp socket")
        continue;
    end
    if contains(p_low, "connection refused sleeping") || contains(p_low, "connection refused")
        continue;
    end
    if contains(p_low, "connection reset by peer") || contains(p_low, "connection timed out")
        continue;
    end
    if contains(p_low, "socket closed")
        continue;
    end
    keep(end+1) = p; %#ok<AGROW>
end

if isempty(keep)
    msg = "heartbeat timeout";
else
    msg = regexprep(char(keep(end)), '\\s+', ' ');
end
end

function [res, boot_msg] = autlWaitForMavlinkPrecheckTop(mission_config, control_cfg)
% Retry MAVLink status for a short window before disabling auto motion.

wait_timeout_s = 40.0;
if isfield(mission_config, 'mavlink_precheck_timeout_s') && isfinite(mission_config.mavlink_precheck_timeout_s)
    wait_timeout_s = max(2.0, double(mission_config.mavlink_precheck_timeout_s));
end
poll_interval_s = 1.5;
boot_msg = "not_used";
did_bootstrap = false;

res = autlMavproxyControl('status', struct(), control_cfg);
if res.is_success
    return;
end

start_t = tic;
while toc(start_t) < wait_timeout_s
    if ~did_bootstrap && isfield(mission_config, 'mavlink_master_fallback') && ...
            strlength(string(mission_config.mavlink_master_fallback)) > 0
        [~, boot_msg] = autlMaybeBootstrapMavlinkFallbackTop(char(string(mission_config.mavlink_master_fallback)));
        did_bootstrap = true;
    end

    pause(poll_interval_s);
    res = autlMavproxyControl('status', struct(), control_cfg);
    if res.is_success
        return;
    end
end

if ~res.is_success
    res.error_message = sprintf('%s (waited %.1fs)', char(string(res.error_message)), wait_timeout_s);
end
end

function [ok, msg] = autlMaybeBootstrapMavlinkFallbackTop(fallback_conn)
% One-shot throttled fallback ping to trigger SITL serial initialization.

ok = false;
msg = "not_used";
if nargin < 1 || strlength(string(fallback_conn)) == 0
    msg = "no_fallback";
    return;
end

persistent boot_keys boot_until_s
if isempty(boot_keys)
    boot_keys = {};
    boot_until_s = [];
end

key = char(string(fallback_conn));
now_s = posixtime(datetime('now'));
idx = find(strcmp(boot_keys, key), 1);
if ~isempty(idx) && boot_until_s(idx) > now_s
    msg = "throttled";
    return;
end

cfg = struct('mav', struct('master_connection', key, 'allow_port_fallback', false));
res = autlMavproxyControl('status', struct(), cfg);
ok = logical(res.is_success);
if ok
    msg = "ok";
else
    msg = "failed:" + autlCompactMavErrorTop(res.error_message);
end

if isempty(idx)
    boot_keys{end+1} = key; %#ok<AGROW>
    boot_until_s(end+1) = now_s + 10.0; %#ok<AGROW>
else
    boot_until_s(idx) = now_s + 10.0;
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
viz.traj_line = plot3(viz.traj_ax, nan, nan, nan, 'b.-', 'LineWidth', 1.2, 'MarkerSize', 9);
hold(viz.traj_ax, 'on');
viz.traj_head = plot3(viz.traj_ax, nan, nan, nan, 'ro', 'MarkerSize', 14, 'LineWidth', 1.5);
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

function viz = autlEnsureRealtimeVizState(viz, session_id)
% Ensure the cached figure/axes/line handles are still valid and re-create if needed.

required_fields = {'fig','traj_ax','traj_line','traj_head','alt_ax','alt_line','vel_ax','vel_line','bat_ax','bat_line','status_text'};
is_valid = isstruct(viz);
if is_valid
    for i = 1:numel(required_fields)
        f = required_fields{i};
        if ~isfield(viz, f) || ~isgraphics(viz.(f))
            is_valid = false;
            break;
        end
    end
end

if is_valid
    viz.enabled = true;
    return;
end

fig = [];
if isstruct(viz) && isfield(viz, 'fig') && isgraphics(viz.fig)
    fig = viz.fig;
end
if isempty(fig)
    fig = figure('Name', 'AutoLanding Real-time Data Collection', 'NumberTitle', 'off', ...
        'Tag', 'autl_realtime_collection', 'Position', [100, 100, 1200, 600]);
else
    clf(fig);
end

viz = autlInitRealtimeVizLayout(fig);
viz = autlResetRealtimeVizLayout(viz, session_id);
viz.enabled = true;
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

profile_name = lower(char(string(mission_config.motion_profile)));
kp_xy = double(mission_config.motion_gain_xy);
k_orbit = double(mission_config.motion_gain_orbit);

if strcmp(profile_name, 'aggressive')
    kp_xy = max(kp_xy, 0.50);
    k_orbit = max(k_orbit, 0.60);
elseif strcmp(profile_name, 'conservative')
    kp_xy = min(kp_xy, 0.24);
    k_orbit = min(k_orbit, 0.28);
elseif strcmp(profile_name, 'orbit-heavy')
    k_orbit = max(k_orbit, 0.75);
end

vx = kp_xy * dx - k_orbit * dy;
vy = kp_xy * dy + k_orbit * dx;

% Force visible lateral movement even near the pad center.
if dist_xy < 0.35
    vx = double(mission_config.motion_min_move_ms);
    vy = 0.00;
end

target_alt_ned = -abs(double(mission_config.takeoff_height_m));
kz = double(mission_config.motion_alt_gain);
vz = kz * (target_alt_ned - double(position_xyz(3)));

vxy_lim = max(0.2, double(mission_config.motion_vxy_limit_ms));
vz_lim = max(0.1, double(mission_config.motion_vz_limit_ms));
vx = autlClamp(vx, -vxy_lim, vxy_lim);
vy = autlClamp(vy, -vxy_lim, vxy_lim);
vz = autlClamp(vz, -vz_lim, vz_lim);
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

function runtime = autlInitLandingPadRuntime(mission_config, log_prefix)
% Initialize landing-pad topic follower and Gazebo pose updater.

runtime = struct();
runtime.follow_enabled = false;
runtime.ros_ctx = [];
runtime.sub = [];
runtime.topic = string(mission_config.landing_pad_topic);
runtime.last_follow_t = -inf;
runtime.follow_interval_s = max(0.05, double(mission_config.landing_pad_follow_interval_s));
runtime.last_center = double(mission_config.landing_pad_center(:)');
runtime.last_size = double(mission_config.landing_pad_size(:)');
runtime.base_center = runtime.last_center;
runtime.apply_set_pose = logical(mission_config.landing_pad_apply_set_pose);
runtime.pad_model_name = char(string(mission_config.landing_pad_model_name));
runtime.set_pose_service = '';
runtime.last_pose_t = -inf;
runtime.pose_interval_s = runtime.follow_interval_s;
runtime.warned_follow = false;
runtime.warned_pose = false;
runtime.default_motion_enabled = logical(mission_config.landing_pad_enable_default_motion);
runtime.default_motion_radius_m = max(0.0, double(mission_config.landing_pad_motion_radius_m));
runtime.default_motion_rate_rad_s = max(0.0, double(mission_config.landing_pad_motion_rate_rad_s));
runtime.motion_started = false;

if mission_config.landing_pad_follow_topic
    try
        rosCtx = autlCreateRosContext("autolanding_pad_follow");
        if isstruct(rosCtx) && isfield(rosCtx, 'enabled') && rosCtx.enabled
            runtime.ros_ctx = rosCtx;
            runtime.sub = ros2subscriber(rosCtx.node, char(runtime.topic), 'std_msgs/Float32MultiArray');
            runtime.follow_enabled = true;
            fprintf('%s[AutoLandingDataCollection] Landing pad topic follower enabled: %s\n', log_prefix, char(runtime.topic));
        else
            fprintf('%s[AutoLandingDataCollection] Landing pad topic follower disabled (ROS2 unavailable).\n', log_prefix);
        end
    catch ME
        fprintf('%s[AutoLandingDataCollection] Landing pad topic follower unavailable: %s\n', log_prefix, ME.message);
    end
end

if runtime.apply_set_pose
    [svc_status, svc_out] = system('bash -lc ''gz service -l 2>/dev/null | grep -E "^/world/.*/set_pose$" | head -n1''');
    if svc_status == 0 && strlength(string(strtrim(svc_out))) > 0
        runtime.set_pose_service = char(string(strtrim(svc_out)));
    else
        runtime.apply_set_pose = false;
        fprintf('%s[AutoLandingDataCollection] Landing pad set_pose disabled: /world/*/set_pose not found.\n', log_prefix);
    end
end
end

function [mission_config, runtime] = autlUpdateLandingPadRuntime(mission_config, runtime, elapsed_s, log_prefix)
% Follow landing_pad_topic values and move Gazebo landing-pad model accordingly.

if runtime.follow_enabled && ((elapsed_s - runtime.last_follow_t) >= runtime.follow_interval_s)
    runtime.last_follow_t = elapsed_s;
    got_topic_update = false;
    try
        msg = receive(runtime.sub, 0.01);
        payload = double(msg.data(:)');
        if numel(payload) >= 3
            mission_config.landing_pad_center = payload(1:3);
            runtime.last_center = mission_config.landing_pad_center;
            got_topic_update = true;
            if numel(payload) >= 5
                mission_config.landing_pad_size = payload(4:5);
                runtime.last_size = mission_config.landing_pad_size;
            end
        end
    catch ME
        if ~runtime.warned_follow
            fprintf('%s[AutoLandingDataCollection] Landing pad topic follow waiting: %s\n', log_prefix, ME.message);
            runtime.warned_follow = true;
        end
    end

    % Keep pad moving even when no external topic publisher is active.
    if runtime.default_motion_enabled && ~got_topic_update
        theta = runtime.default_motion_rate_rad_s * elapsed_s;
        motion_xy = runtime.default_motion_radius_m * [cos(theta), sin(theta)];
        mission_config.landing_pad_center(1) = runtime.base_center(1) + motion_xy(1);
        mission_config.landing_pad_center(2) = runtime.base_center(2) + motion_xy(2);
        mission_config.landing_pad_center(3) = runtime.base_center(3);
        runtime.last_center = mission_config.landing_pad_center;
        if ~runtime.motion_started
            fprintf('%s[AutoLandingDataCollection] Landing pad default motion active (r=%.2fm, w=%.2frad/s)\n', ...
                log_prefix, runtime.default_motion_radius_m, runtime.default_motion_rate_rad_s);
            runtime.motion_started = true;
        end
    end
end

if runtime.apply_set_pose && ((elapsed_s - runtime.last_pose_t) >= runtime.pose_interval_s)
    runtime.last_pose_t = elapsed_s;
    marker_width_ref_m = 0.4;
    marker_scale = max(0.25, mission_config.landing_pad_size(1) / marker_width_ref_m);
    pose_z = mission_config.landing_pad_center(3) + (0.25 * marker_scale);
    req = sprintf(['name: "%s", position: {x: %.3f, y: %.3f, z: %.3f}, ' ...
        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'], ...
        runtime.pad_model_name, mission_config.landing_pad_center(1), mission_config.landing_pad_center(2), pose_z);
    cmd = sprintf('bash -lc ''gz service -s "%s" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req ''''''%s'''''' 2>/dev/null''', ...
        runtime.set_pose_service, req);
    [rc, out] = system(cmd);
    if ~(rc == 0 && ~contains(lower(string(out)), 'false')) && ~runtime.warned_pose
        fprintf('%s[AutoLandingDataCollection] Landing pad set_pose warning: rc=%d out=%s\n', log_prefix, rc, strtrim(string(out)));
        runtime.warned_pose = true;
    end
end
end

function autlReleaseLandingPadRuntime(runtime)
% Release ROS resources allocated by landing-pad runtime.

if ~isstruct(runtime)
    return;
end

try
    if isfield(runtime, 'sub') && ~isempty(runtime.sub)
        clear runtime.sub;
    end
catch
end

try
    if isfield(runtime, 'ros_ctx') && ~isempty(runtime.ros_ctx)
        autlReleaseRosContext(runtime.ros_ctx);
    end
catch
end
end

function runtime = autlInitDroneFallbackRuntime(mission_config, log_prefix)
% Initialize fallback set_pose motion for the drone model.

runtime = struct();
runtime.enabled = logical(mission_config.drone_set_pose_fallback_enabled);
runtime.model_name = char(string(mission_config.drone_set_pose_model_name));
runtime.model_candidates = {runtime.model_name};
if isfield(mission_config, 'reset_model_name') && strlength(string(mission_config.reset_model_name)) > 0
    runtime.model_candidates{end+1} = char(string(mission_config.reset_model_name));
end
runtime.model_candidates = [runtime.model_candidates, {'iris_with_gimbal', 'iris_with_gimbal_0', 'iris_with_gimbal_1', 'iris_with_gimbal_2', 'iris'}];
runtime.model_candidates = unique(runtime.model_candidates, 'stable');
runtime.active_model_idx = 1;
runtime.set_pose_service = '';
runtime.interval_s = max(0.05, double(mission_config.drone_set_pose_interval_s));
runtime.radius_m = max(0.0, double(mission_config.drone_set_pose_radius_m));
runtime.rate_rad_s = max(0.0, double(mission_config.drone_set_pose_rate_rad_s));
runtime.base_xy = double(mission_config.reset_spawn_xy(1:2));
runtime.base_z = double(mission_config.reset_spawn_z_m);
runtime.base_yaw_deg = double(mission_config.reset_spawn_yaw_deg);
runtime.takeoff_height_m = max(0.0, double(mission_config.drone_set_pose_takeoff_height_m));
runtime.climb_rate_mps = max(0.05, double(mission_config.drone_set_pose_climb_rate_mps));
runtime.last_t = -inf;
runtime.warned = false;
runtime.started = false;
runtime.t0 = -inf;
runtime.reached_takeoff_alt = false;

if ~runtime.enabled
    return;
end

svc_deadline = tic;
while toc(svc_deadline) < 12.0
    [svc_status, svc_out] = system('bash -lc ''gz service -l 2>/dev/null | grep -E "^/world/.*/set_pose$" | head -n1''');
    if svc_status == 0 && strlength(string(strtrim(svc_out))) > 0
        runtime.set_pose_service = char(string(strtrim(svc_out)));
        break;
    end
    pause(0.4);
end
if strlength(string(runtime.set_pose_service)) == 0
    runtime.enabled = false;
    fprintf('%s[AutoLandingDataCollection] Drone fallback motion disabled: /world/*/set_pose not found.\n', log_prefix);
end
end

function [mission_config, runtime] = autlApplyDroneFallbackMotion(mission_config, runtime, elapsed_s, log_prefix)
% Move the drone model on a small circle when MAVLink control is unavailable.

if ~runtime.enabled
    return;
end
if (elapsed_s - runtime.last_t) < runtime.interval_s
    return;
end
runtime.last_t = elapsed_s;
if runtime.t0 < 0
    runtime.t0 = elapsed_s;
end

theta = runtime.rate_rad_s * elapsed_s;
target_x = runtime.base_xy(1) + runtime.radius_m * cos(theta);
target_y = runtime.base_xy(2) + runtime.radius_m * sin(theta);
climb_elapsed = max(0.0, elapsed_s - runtime.t0);
climb_m = min(runtime.takeoff_height_m, runtime.climb_rate_mps * climb_elapsed);
target_z = runtime.base_z + climb_m;

target_yaw_rad = deg2rad(runtime.base_yaw_deg) + theta + pi/2;
qz = sin(target_yaw_rad / 2.0);
qw = cos(target_yaw_rad / 2.0);

req = sprintf(['name: "%s", position: {x: %.3f, y: %.3f, z: %.3f}, ' ...
    'orientation: {x: 0.0, y: 0.0, z: %.6f, w: %.6f}'], ...
    runtime.model_candidates{runtime.active_model_idx}, target_x, target_y, target_z, qz, qw);
cmd = sprintf('bash -lc ''gz service -s "%s" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req ''''''%s'''''' 2>/dev/null''', ...
    runtime.set_pose_service, req);
[rc, out] = system(cmd);

ok = (rc == 0 && ~contains(lower(string(out)), 'false'));
if ~ok
    for mi = 1:numel(runtime.model_candidates)
        req_i = sprintf(['name: "%s", position: {x: %.3f, y: %.3f, z: %.3f}, ' ...
            'orientation: {x: 0.0, y: 0.0, z: %.6f, w: %.6f}'], ...
            runtime.model_candidates{mi}, target_x, target_y, target_z, qz, qw);
        cmd_i = sprintf('bash -lc ''gz service -s "%s" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req ''''''%s'''''' 2>/dev/null''', ...
            runtime.set_pose_service, req_i);
        [rc_i, out_i] = system(cmd_i);
        if rc_i == 0 && ~contains(lower(string(out_i)), 'false')
            runtime.active_model_idx = mi;
            runtime.model_name = runtime.model_candidates{mi};
            rc = rc_i;
            out = out_i;
            ok = true;
            break;
        end
    end
end

if ~ok
    if ~runtime.warned
        fprintf('%s[AutoLandingDataCollection] Drone fallback set_pose warning: rc=%d out=%s\n', ...
            log_prefix, rc, strtrim(string(out)));
        runtime.warned = true;
    end
else
    mission_config.reset_spawn_xy = [target_x, target_y];
    mission_config.reset_spawn_z_m = target_z;
    if ~runtime.started
        fprintf('%s[AutoLandingDataCollection] Drone fallback motion active (model=%s, r=%.2fm, w=%.2frad/s, takeoff=%.2fm @ %.2fm/s)\n', ...
            log_prefix, runtime.model_name, runtime.radius_m, runtime.rate_rad_s, runtime.takeoff_height_m, runtime.climb_rate_mps);
        runtime.started = true;
    end
    if ~runtime.reached_takeoff_alt && climb_m >= (runtime.takeoff_height_m - 0.05)
        fprintf('%s[AutoLandingDataCollection] Drone fallback reached takeoff altitude: z=%.2fm\n', ...
            log_prefix, target_z);
        runtime.reached_takeoff_alt = true;
    end
end
end

function autlReleaseDroneFallbackRuntime(~)
% Reserved for symmetry if runtime cleanup is needed later.
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

function out = autlFlowMerge(base_payload, extra_payload)
out = struct();
if nargin >= 1 && isstruct(base_payload)
    fields = fieldnames(base_payload);
    for i = 1:numel(fields)
        out.(fields{i}) = base_payload.(fields{i});
    end
end
if nargin >= 2 && isstruct(extra_payload)
    fields = fieldnames(extra_payload);
    for i = 1:numel(fields)
        out.(fields{i}) = extra_payload.(fields{i});
    end
end
end

% End of main autlRunDataCollection function
end
