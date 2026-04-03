function AutoLandingMain(varargin)
% AUTOLANDINGMAIN
% Top-level MATLAB entry point for semantic autonomous landing.
%
% Usage:
%   AutoLandingMain                   - Run pipeline and validation
%   AutoLandingMain('mission')        - Run pipeline then autonomous mission with real-time viz
%   AutoLandingMain('collect')        - Collect raw sensor data (no ontology)
%   AutoLandingMain('collect_parallel', N) - Parallel data collection with N workers
%   AutoLandingMain('sim')            - Run full simulation with mission and telemetry

    % Parse inputs
    runMode = 'pipeline'; % 'pipeline', 'mission', 'collect', 'collect_parallel', 'sim'
    if nargin > 0
        runMode = varargin{1};
    end
    
    % For collect_parallel mode, get worker count
    num_workers = 4;
    if nargin > 1 && strcmpi(runMode, 'collect_parallel')
        num_workers = varargin{2};
    end
    
    rootDir = fileparts(fileparts(mfilename('fullpath')));
    
    modDir = fullfile(rootDir, 'matlab', 'modules');
    if exist(modDir, 'dir')
        addpath(modDir);
    end
    coreDir = fullfile(modDir, 'core');
    if exist(coreDir, 'dir')
        addpath(genpath(coreDir));
    end
    
    % Register global cleanup for safe shutdown
    cleanup_obj = onCleanup(@() autlMainCleanup());
    
    try
        % Run pipeline
        semanticInputPath = fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json');
        summary = autlRunPipeline(rootDir, semanticInputPath);
        validationSummary = autlRunValidation(rootDir, semanticInputPath);
    
    switch lower(runMode)
        case 'pipeline'
            disp('[AutoLandingMain] Running pipeline and validation only.');
            disp(summary);
            disp(validationSummary);
            
        case 'collect'
            disp('[AutoLandingMain] Running raw sensor data collection (no ontology).');
            mission_cfg = struct();
            mission_cfg.max_duration = 120;  % Collect for 2 minutes
            mission_cfg.sample_rate = 50;    % 50 Hz sampling
            result = autlRunDataCollection(rootDir, mission_cfg);
            disp(result);
            
        case 'collect_parallel'
            disp('[AutoLandingMain] Running parallel data collection.');
            if nargin < 2
                num_workers = 4;
                scenarios_per_worker = 10;
            else
                num_workers = varargin{2};
                scenarios_per_worker = 10;
                if nargin > 2
                    scenarios_per_worker = varargin{3};
                end
            end
            AutoLandingDataParallel(num_workers, scenarios_per_worker);
            
        case 'mission'
            disp('[AutoLandingMain] Running pipeline and autonomous mission with real-time visualization.');
            disp(summary);
            
            % Extract trajectory/state from pipeline summary
            trajectory = summary.landing_trajectory;
            initial_state = struct('x0', summary.initial_state(1), 'y0', summary.initial_state(2), ...
                                  'z0', summary.initial_state(3), 'yaw0', 0);
            target_state = struct('x_target', summary.target_state(1), 'y_target', summary.target_state(2), ...
                                 'z_target', summary.target_state(3));
            
            % Load config
            cfg = autlDefaultConfig();
            cfg.mission.use_ros_control = true;
            cfg.mav.allow_port_fallback = true;
            
            % Start real-time visualization in background
            session_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            
            % Create visualization thread (non-blocking)
            disp('[AutoLandingMain] Starting real-time visualization...');
            try
                % Try to launch visualization (may require separate thread)
                autlVisualizeMissionRealtime(session_id, 0.5);
            catch ME
                disp(sprintf('[AutoLandingMain] Visualization warning (continuing without viz): %s', ME.message));
            end
            
            % Execute autonomous mission
            mission_result = autlAutonomousMission(trajectory, initial_state, target_state, cfg, rootDir);
            
            % Display mission results
            disp(' ');
            disp('======================================');
            disp('    AUTONOMOUS MISSION RESULTS');
            disp('======================================');
            disp(sprintf('Mission Status: %s', mission_result.mission_status));
            disp(sprintf('Success: %s', char(string(mission_result.is_success))));
            if isfield(mission_result, 'elapsed_time')
                disp(sprintf('Elapsed Time: %.2f seconds', mission_result.elapsed_time));
            end
            disp(sprintf('Waypoints Sent: %d', mission_result.phases.trajectory_tracking.points_sent));
            
            % Save mission results
            mission_output_file = fullfile(rootDir, 'data', 'processed', 'mission_result_matlab.json');
            try
                mission_json = jsonencode(mission_result);
                mission_fid = fopen(mission_output_file, 'w');
                fprintf(mission_fid, mission_json);
                fclose(mission_fid);
                disp(sprintf('[AutoLandingMain] Mission results saved to %s', mission_output_file));
            catch
                disp('[AutoLandingMain] Warning: Could not save mission results to JSON');
            end
            
        case 'sim'
            disp('[AutoLandingMain] Running full simulation with real-time telemetry.');
            disp(summary);
            
            % For sim mode, run the mission
            trajectory = summary.landing_trajectory;
            initial_state = struct('x0', summary.initial_state(1), 'y0', summary.initial_state(2), ...
                                  'z0', summary.initial_state(3), 'yaw0', 0);
            target_state = struct('x_target', summary.target_state(1), 'y_target', summary.target_state(2), ...
                                 'z_target', summary.target_state(3));
            
            cfg = autlDefaultConfig();
            cfg.mission.trajectory_tracking_enabled = true;
            cfg.mission.use_ros_control = false; % Use MAVProxy control
            cfg.mav.allow_port_fallback = true;
            
            mission_result = autlAutonomousMission(trajectory, initial_state, target_state, cfg, rootDir);
            
            disp(' ');
            disp('======================================');
            disp('    SIMULATION MISSION RESULTS');
            disp('======================================');
            disp(sprintf('Mission Status: %s', mission_result.mission_status));
            disp(sprintf('Success: %s', char(string(mission_result.is_success))));
            if isfield(mission_result, 'elapsed_time')
                disp(sprintf('Elapsed Time: %.2f seconds', mission_result.elapsed_time));
            end
            
        otherwise
            error('[AutoLandingMain] Unknown run mode: %s', runMode);
    end
    
    disp('[AutoLandingMain] Complete.');
    
    catch ME
        % Handle interruption or errors
        if contains(lower(ME.message), 'interrupt') || contains(lower(ME.identifier), 'interrupt')
            disp('[AutoLandingMain] User interrupted. Performing cleanup...');
        else
            disp(sprintf('[AutoLandingMain] ERROR: %s', ME.message));
        end
        rethrow(ME);
    end
end

function autlMainCleanup()
% Cleanup function called on any exit (Ctrl+C or normal termination)

try
    fprintf('[AutoLandingMain.cleanup] Cleaning up resources...\n');
    
    % Kill background processes to prevent port conflicts
    system('pkill -f "gz sim" 2>/dev/null');
    system('pkill -f "mavproxy.py" 2>/dev/null');
    
    % Delete parallel pool if exists
    pool = gcp('nocreate');
    if ~isempty(pool)
        delete(pool);
    end
    
    fprintf('[AutoLandingMain.cleanup] Cleanup complete.\n');
catch
    % Silent fail to not interfere with cleanup
end
end
