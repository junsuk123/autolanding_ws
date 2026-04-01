function mission_result = autlAutonomousMission(trajectory_table, initial_state, target_state, cfg, root_dir)
% AUTLAUTONOMOUSMISSION Execute autonomous landing mission with MATLAB trajectory
%
%   mission_result = autlAutonomousMission(trajectory_table, initial_state, target_state, cfg, root_dir)
%
% Inputs:
%   trajectory_table : table with columns (t, x, y, z, vx_cmd, vy_cmd, vz_cmd, fused_confidence, ...)
%                     OR struct with similar fields
%   initial_state    : struct with x0, y0, z0, yaw0 (drone current position)
%   target_state     : struct with x_target, y_target, z_target (target landing position)
%   cfg              : config struct with mission parameters
%   root_dir        : workspace root for logging (optional)
%
% Outputs:
%   mission_result   : struct with mission_status, trajectory_followed, metrics, log

    % Default rootDir if not provided
    if nargin < 5 || isempty(root_dir)
        root_dir = pwd;
    end
    
    % Initialize mission result
    mission_result = struct();
    mission_result.mission_status = 'unknown';
    mission_result.trajectory_id = '';
    mission_result.is_success = false;
    mission_result.phases = struct();
    mission_result.log = {};
    mission_result.telemetry_log = [];
    
    % Default mission config
    if ~isfield(cfg, 'mission')
        cfg.mission = struct();
    end
    if ~isfield(cfg.mission, 'takeoff_height')
        cfg.mission.takeoff_height = 2.0; % meters
    end
    if ~isfield(cfg.mission, 'trajectory_tracking_enabled')
        cfg.mission.trajectory_tracking_enabled = true;
    end
    if ~isfield(cfg.mission, 'use_ros_control')
        cfg.mission.use_ros_control = false;
    end
    if ~isfield(cfg.mission, 'mission_timeout')
        cfg.mission.mission_timeout = 300; % seconds
    end
    if ~isfield(cfg.mission, 'velocity_scale_factor')
        cfg.mission.velocity_scale_factor = 1.0; % scale trajectory velocity
    end
    
    % Start mission timer
    mission_start_time = tic;
    
    try
        % Convert trajectory to table if struct
        if isstruct(trajectory_table)
            if isfield(trajectory_table, 'trajectory_points')
                traj_cell = trajectory_table.trajectory_points;
            else
                % Try to convert fields to table
                traj_cell = trajectory_table;
            end
        else
            traj_cell = trajectory_table;
        end
        
        % Log mission start
        log_msg = sprintf('[MISSION] Starting autonomous landing mission at %s', datetime('now'));
        mission_result.log{end+1} = log_msg;
        disp(log_msg);
        
        % ====== PHASE 1: ARM VEHICLE ======
        mission_result.phases.arm = struct('status', 'pending', 'error', '');
        log_msg = '[PHASE 1] Arming vehicle...';
        mission_result.log{end+1} = log_msg;
        disp(log_msg);
        
        arm_result = autlMavproxyControl('arm', struct(), cfg);
        if arm_result.is_success
            mission_result.phases.arm.status = 'success';
            log_msg = sprintf('[PHASE 1] Vehicle armed successfully');
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
        else
            mission_result.phases.arm.status = 'failed';
            mission_result.phases.arm.error = arm_result.error_message;
            log_msg = sprintf('[PHASE 1] FAILED to arm: %s', arm_result.error_message);
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
            mission_result.mission_status = 'arm_failed';
            return;
        end
        
        % ====== PHASE 2: TAKEOFF ======
        mission_result.phases.takeoff = struct('status', 'pending', 'height', cfg.mission.takeoff_height, 'error', '');
        log_msg = sprintf('[PHASE 2] Taking off to height %.2f m...', cfg.mission.takeoff_height);
        mission_result.log{end+1} = log_msg;
        disp(log_msg);
        
        takeoff_result = autlMavproxyControl('takeoff', struct('height', cfg.mission.takeoff_height), cfg);
        if takeoff_result.is_success
            mission_result.phases.takeoff.status = 'success';
            log_msg = '[PHASE 2] Takeoff initiated, waiting for vehicle to reach altitude...';
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
            pause(5); % Wait for takeoff to complete
        else
            mission_result.phases.takeoff.status = 'failed';
            mission_result.phases.takeoff.error = takeoff_result.error_message;
            log_msg = sprintf('[PHASE 2] FAILED takeoff: %s', takeoff_result.error_message);
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
            mission_result.mission_status = 'takeoff_failed';
            return;
        end
        
        % ====== PHASE 3: TRAJECTORY FOLLOWING (if enabled) ======
        mission_result.phases.trajectory_tracking = struct('status', 'pending', 'points_sent', 0, 'error', '');
        
        if cfg.mission.trajectory_tracking_enabled
            log_msg = sprintf('[PHASE 3] Beginning trajectory tracking with %d waypoints...', ...
                             size(traj_cell, 1));
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
            
            % Initialize trajectory tracking
            n_points = size(traj_cell, 1);
            traj_telemetry = [];
            
            for i = 1:min(n_points, 50) % Track up to 50 waypoints for safety
                % Check mission timeout
                elapsed = toc(mission_start_time);
                if elapsed > cfg.mission.mission_timeout
                    mission_result.phases.trajectory_tracking.status = 'timeout';
                    log_msg = sprintf('[PHASE 3] Mission timeout after %.1f seconds', elapsed);
                    mission_result.log{end+1} = log_msg;
                    disp(log_msg);
                    break;
                end
                
                % Extract trajectory point
                if istable(traj_cell)
                    traj_point = traj_cell(i, :);
                    t = traj_point.t;
                    x = traj_point.x;
                    y = traj_point.y;
                    z = traj_point.z;
                    vx = traj_point.vx_cmd * cfg.mission.velocity_scale_factor;
                    vy = traj_point.vy_cmd * cfg.mission.velocity_scale_factor;
                    vz = traj_point.vz_cmd * cfg.mission.velocity_scale_factor;
                    confidence = traj_point.fused_confidence;
                else
                    % Assume cell array structure
                    t = traj_cell{i, 1};
                    x = traj_cell{i, 2};
                    y = traj_cell{i, 3};
                    z = traj_cell{i, 4};
                    vx = traj_cell{i, 5} * cfg.mission.velocity_scale_factor;
                    vy = traj_cell{i, 6} * cfg.mission.velocity_scale_factor;
                    vz = traj_cell{i, 7} * cfg.mission.velocity_scale_factor;
                    confidence = traj_cell{i, 8};
                end
                
                % Log trajectory waypoint
                if i <= 5 || mod(i, 10) == 0
                    log_msg = sprintf('[PHASE 3.%d] WP %d/%d: pos=[%.2f, %.2f, %.2f], vel=[%.2f, %.2f, %.2f], conf=%.3f', ...
                                     i, i, n_points, x, y, z, vx, vy, vz, confidence);
                    mission_result.log{end+1} = log_msg;
                    if i <= 5 || mod(i, 10) == 0
                        disp(log_msg);
                    end
                end
                
                % Send velocity command via ROS/MAVProxy
                if cfg.mission.use_ros_control
                    % Use ROS2 for velocity control (requires autlPublishTrajectory)
                    % For now, log the command
                    telemetry_record = struct('time', t, 'position', [x, y, z], ...
                                             'velocity_cmd', [vx, vy, vz], 'confidence', confidence);
                else
                    % Use MAVProxy for mode setting (velocity control requires mavros)
                    vel_cmd_struct = struct('vx', vx, 'vy', vy, 'vz', vz, 'duration', 0.1);
                    vel_result = autlMavproxyControl('set_velocity', vel_cmd_struct, cfg);
                    telemetry_record = struct('time', t, 'position', [x, y, z], ...
                                             'velocity_cmd', [vx, vy, vz], 'confidence', confidence, ...
                                             'mavproxy_output', vel_result.output);
                end
                
                traj_telemetry = [traj_telemetry; telemetry_record];
                mission_result.phases.trajectory_tracking.points_sent = i;
                
                % Small pause between commands
                pause(0.05);
            end
            
            mission_result.phases.trajectory_tracking.status = 'completed';
            mission_result.telemetry_log = traj_telemetry;
            log_msg = sprintf('[PHASE 3] Trajectory tracking completed: %d points', ...
                             mission_result.phases.trajectory_tracking.points_sent);
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
        end
        
        % ====== PHASE 4: LAND ======
        mission_result.phases.land = struct('status', 'pending', 'error', '');
        log_msg = '[PHASE 4] Initiating landing sequence...';
        mission_result.log{end+1} = log_msg;
        disp(log_msg);
        
        land_result = autlMavproxyControl('land', struct(), cfg);
        if land_result.is_success
            mission_result.phases.land.status = 'success';
            log_msg = '[PHASE 4] Landing mode activated, vehicle descending...';
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
            pause(10); % Wait for landing to complete
        else
            mission_result.phases.land.status = 'failed';
            mission_result.phases.land.error = land_result.error_message;
            log_msg = sprintf('[PHASE 4] FAILED land: %s', land_result.error_message);
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
        end
        
        % ====== PHASE 5: DISARM ======
        mission_result.phases.disarm = struct('status', 'pending', 'error', '');
        log_msg = '[PHASE 5] Disarming vehicle...';
        mission_result.log{end+1} = log_msg;
        disp(log_msg);
        
        disarm_result = autlMavproxyControl('disarm', struct(), cfg);
        if disarm_result.is_success
            mission_result.phases.disarm.status = 'success';
            log_msg = '[PHASE 5] Vehicle disarmed successfully';
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
        else
            mission_result.phases.disarm.status = 'failed';
            mission_result.phases.disarm.error = disarm_result.error_message;
            log_msg = sprintf('[PHASE 5] WARNING: Failed to disarm: %s', disarm_result.error_message);
            mission_result.log{end+1} = log_msg;
            disp(log_msg);
        end
        
        % Mission completed
        mission_result.mission_status = 'completed';
        mission_result.is_success = true;
        mission_result.elapsed_time = toc(mission_start_time);
        
        log_msg = sprintf('[MISSION] Autonomous mission completed in %.2f seconds', mission_result.elapsed_time);
        mission_result.log{end+1} = log_msg;
        disp(log_msg);
        
        % Save mission log
        if ~isempty(root_dir)
            log_dir = fullfile(root_dir, 'data', 'processed');
            if ~exist(log_dir, 'dir')
                mkdir(log_dir);
            end
            
            % Save telemetry
            if ~isempty(mission_result.telemetry_log)
                telem_file = fullfile(log_dir, 'mission_telemetry_log.mat');
                save(telem_file, 'mission_result');
            end
            
            % Save log
            log_file = fullfile(log_dir, 'mission_log.txt');
            fid = fopen(log_file, 'w');
            for j = 1:length(mission_result.log)
                fprintf(fid, '%s\n', mission_result.log{j});
            end
            fclose(fid);
            
            mission_result.log_file = log_file;
        end
        
    catch ME
        mission_result.mission_status = 'error';
        mission_result.is_success = false;
        mission_result.error_message = ME.message;
        mission_result.log{end+1} = sprintf('[ERROR] %s', ME.message);
        disp(mission_result.log{end});
    end
end
