function AutoLandingMainFull(varargin)
% AUTOLANDINGMAINFULL
% Complete autonomous landing pipeline with configurable parameters.
%
% WORKFLOW:
% 1. Parallel data collection (raw sensor data)
% 2. Ontology + AI model training
% 3. Pure AI model training (baseline)
% 4. Model validation and comparison
% 5. Plot generation and analysis
%
% CONFIGURATION PARAMETERS (edit below to customize)

    %% ============ CONFIGURATION (EDIT HERE) ============
    
    % Simulation Settings
    % NOTE: GUI mode requires proper GPU-accelerated X11 display.
    % If Gazebo doesn't appear, use server_mode=true (recommended for data collection)
    % Real-time visualization is available during data collection regardless of mode.
    gazebo_server_mode = false;           % true = server (headless, -s), false = GUI mode
    enable_visualization = true;        % true = real-time plots/monitoring (works in both modes)

    % Optional override from call:
    %   AutoLandingMainFull('gui')
    %   AutoLandingMainFull('server')
    if nargin >= 1
        mode_arg = lower(char(string(varargin{1})));
        if strcmp(mode_arg, 'gui')
            gazebo_server_mode = false;
        elseif strcmp(mode_arg, 'server') || strcmp(mode_arg, 'headless')
            gazebo_server_mode = true;
        end
    end
    
    % Data Collection Settings
    num_workers = 3;                    % Number of parallel workers (use 1 to avoid MAVProxy contention)
    scenarios_per_worker = 200;           % Scenarios per worker (total = num_workers * scenarios_per_worker)
    enable_auto_motion = true;          % Send MAVLink velocity setpoints during collection
    takeoff_height_m = 3.0;             % Takeoff altitude used before motion commands
    enable_multi_drone_profiles = true; % Enable worker-specific MAVLink/model/spawn/motion profiles

    % RViz / ROS observability
    enable_rviz_monitor = true;
    rviz_ros_domain_id = 'auto';        % 'auto' or numeric string (e.g., '21')

    % ArUco landing pad configuration
    use_aruco_landing_pad = true;
    aruco_marker_id = 23;
    drone_body_size_m = 0.1;            % Reference body size for marker scaling (0.1 * 2^2 = 0.4 m)
    marker_size_multiplier = 3^2;       % Requested: 2 squared times drone size
    marker_size_m = drone_body_size_m * marker_size_multiplier;
    landing_pad_center = [0.0, 0.0, 0.0];   % Fixed pad position [x, y, z]
    landing_pad_size = [marker_size_m, marker_size_m];
    landing_pad_topic = '/autolanding/landing_pad';
    spawn_radius_m = 0.8;               % Must remain within 1 m of marker center
    spawn_angle_deg = 35.0;             % Fixed angle to keep start deterministic

    % Train/Validation Split
    total_training_samples = 20;         % Total samples for training (including both train+val)
    train_ratio = 0.75;                 % 75% train, 25% validation
    
    % Model Configuration
    use_ontology_model = true;          % Train ontology+AI hybrid model
    use_pure_ai_model = true;           % Train pure AI baseline model
    
    % Visualization
    enable_plots = true;                % Generate comparison plots
    
    %% ============ DERIVED PARAMETERS (auto-calculated) ============
    
    num_train = round(total_training_samples * train_ratio);
    num_val = total_training_samples - num_train;
    
    fprintf('\n');
    fprintf('═════════════════════════════════════════════════════════════\n');
    fprintf('  AutoLanding Complete Pipeline (Ontology vs Pure AI)\n');
    fprintf('═════════════════════════════════════════════════════════════\n');
    fprintf('\nCONFIGURATION:\n');
    fprintf('  Simulation:\n');
    gz_mode_str = 'GUI (Headless: OFF)';
    if gazebo_server_mode
        gz_mode_str = 'Server (Headless: ON)';
    end
    fprintf('    - Gazebo Mode: %s\n', gz_mode_str);
    fprintf('    - Real-time Visualization: %s\n', char(string(enable_visualization)));
    fprintf('  Data Collection:\n');
    fprintf('    - Workers: %d\n', num_workers);
    fprintf('    - Scenarios/Worker: %d\n', scenarios_per_worker);
    fprintf('    - Total Scenarios: %d\n', num_workers * scenarios_per_worker);
    fprintf('    - Auto Motion: %s (takeoff=%.1fm)\n', char(string(enable_auto_motion)), takeoff_height_m);
    fprintf('    - Multi-drone Profiles: %s\n', char(string(enable_multi_drone_profiles)));
    fprintf('    - Landing Pad Topic: %s\n', landing_pad_topic);
    fprintf('    - ArUco Enabled: %s (id=%d)\n', char(string(use_aruco_landing_pad)), aruco_marker_id);
    fprintf('    - RViz Monitor: %s (ROS_DOMAIN_ID=%s)\n', char(string(enable_rviz_monitor)), char(string(rviz_ros_domain_id)));
    fprintf('    - Landing Pad Center: [%.2f, %.2f, %.2f]\n', landing_pad_center(1), landing_pad_center(2), landing_pad_center(3));
    fprintf('    - Landing Pad Size: [%.2f x %.2f] m\n', landing_pad_size(1), landing_pad_size(2));
    fprintf('    - Drone Spawn Radius: %.2f m (<= 1.0 m)\n', spawn_radius_m);
    fprintf('  Training:\n');
    fprintf('    - Train Samples: %d (%.0f%%)\n', num_train, train_ratio*100);
    fprintf('    - Val Samples: %d (%.0f%%)\n', num_val, (1-train_ratio)*100);
    fprintf('    - Ontology+AI Model: %s\n', char(string(use_ontology_model)));
    fprintf('    - Pure AI Model: %s\n', char(string(use_pure_ai_model)));
    fprintf('    - Plots: %s\n', char(string(enable_plots)));
    fprintf('═════════════════════════════════════════════════════════════\n\n');
    
    rootDir = fileparts(fileparts(mfilename('fullpath')));
    
    % Setup paths
    modDir = fullfile(rootDir, 'matlab', 'modules');
    if exist(modDir, 'dir')
        addpath(modDir);
    end
    coreDir = fullfile(modDir, 'core');
    if exist(coreDir, 'dir')
        addpath(genpath(coreDir));
    end
    
    % Register cleanup
    cleanup_obj = onCleanup(@() autlMainCleanup());
    
    try
        %% STEP 0: Initialize Simulation Environment
        fprintf('\n[Pipeline] STEP 0: Initialize Simulation Environment\n');
        fprintf('════════════════════════════════════════════\n');
        
        % Kill any existing processes
        fprintf('[Pipeline] Cleaning up any existing processes...\n');
        system('pkill -f "gz sim" 2>/dev/null');
        system('pkill -f "arducopter" 2>/dev/null');
        system('pkill -f "mavproxy" 2>/dev/null');
        pause(2);
        
        % Start Gazebo
        % Use the same working command style as manual launch:
        %   cd ~/gz_ws/src/ardupilot_gazebo/worlds
        %   gz sim -v4 -r iris_runway.sdf
        home_dir = getenv('HOME');
        gazebo_pkg_dir = fullfile(home_dir, 'gz_ws', 'src', 'ardupilot_gazebo');
        aruco_pkg_dir = fullfile(home_dir, 'gz_ros2_aruco_ws', 'src', 'ros2-gazebo-aruco');
        world_dir = fullfile(gazebo_pkg_dir, 'worlds');
        base_world_path = fullfile(world_dir, 'iris_runway.sdf');

        spawn_angle_rad = deg2rad(spawn_angle_deg);
        spawn_xy = [landing_pad_center(1) + spawn_radius_m * cos(spawn_angle_rad), ...
                    landing_pad_center(2) + spawn_radius_m * sin(spawn_angle_rad)];
        spawn_yaw_deg = rad2deg(atan2(landing_pad_center(2) - spawn_xy(2), landing_pad_center(1) - spawn_xy(1)));
        if norm(spawn_xy - landing_pad_center(1:2)) > 1.0
            error('Spawn position exceeds 1 m radius from marker center.');
        end

        worker_profiles = struct([]);
        if enable_multi_drone_profiles
            worker_profiles = autlBuildWorkerProfiles(num_workers, spawn_xy, spawn_yaw_deg, takeoff_height_m, landing_pad_center, landing_pad_size);
        end

        world_path_to_launch = base_world_path;
        if use_aruco_landing_pad
            generated_world_path = fullfile('/tmp', 'iris_runway_aruco_landing.sdf');
            autlCreateArucoLandingWorld(base_world_path, generated_world_path, aruco_pkg_dir, ...
                landing_pad_center, marker_size_m, spawn_xy, spawn_yaw_deg, aruco_marker_id, num_workers, worker_profiles);
            world_path_to_launch = generated_world_path;
            fprintf('[Pipeline] ArUco world generated: %s\n', world_path_to_launch);
        end

        launch_world_dir = fileparts(world_path_to_launch);
        launch_world_name = world_path_to_launch;
        display_env = getenv('DISPLAY');
        if strlength(string(display_env)) == 0
            display_env = ':0';
        end
        xauth_env = getenv('XAUTHORITY');

        % Resource path is required so model:// URIs resolve (runway, iris_with_gimbal, ...)
        gz_resource_path = sprintf('%s:%s:%s:%s:%s', gazebo_pkg_dir, fullfile(gazebo_pkg_dir, 'models'), world_dir, ...
            launch_world_dir, fullfile(aruco_pkg_dir, 'gz-world'));
        gz_system_plugin_path = fullfile(gazebo_pkg_dir, 'build');
        gz_clean_env = 'unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME;';
        xauth_export = '';
        if strlength(string(xauth_env)) > 0
            xauth_export = sprintf('export XAUTHORITY="%s";', xauth_env);
        end
        
        if gazebo_server_mode
            fprintf('[Pipeline] Starting Gazebo (server mode)...\n');
            gz_cmd = sprintf('bash -lc ''%s export GZ_SIM_SYSTEM_PLUGIN_PATH="%s:${GZ_SIM_SYSTEM_PLUGIN_PATH}"; export GZ_SIM_RESOURCE_PATH="%s"; cd "%s" && gz sim -s -v4 -r "%s" > /tmp/gz_sim.log 2>&1 &''', gz_clean_env, gz_system_plugin_path, gz_resource_path, launch_world_dir, launch_world_name);
        else
            fprintf('[Pipeline] Starting Gazebo (GUI mode)...\n');
            fprintf('[Pipeline] Display: %s\n', display_env);
            gz_cmd = sprintf('bash -lc ''%s export DISPLAY="%s"; %s export QT_QPA_PLATFORM=xcb; export GZ_SIM_SYSTEM_PLUGIN_PATH="%s:${GZ_SIM_SYSTEM_PLUGIN_PATH}"; export GZ_SIM_RESOURCE_PATH="%s"; xhost +local: 2>/dev/null || true; cd "%s" && gz sim -v4 -r "%s" > /tmp/gz_sim.log 2>&1 &''', gz_clean_env, display_env, xauth_export, gz_system_plugin_path, gz_resource_path, launch_world_dir, launch_world_name);
            fprintf('[Pipeline] Launch command: cd %s && gz sim -v4 -r %s\n', launch_world_dir, launch_world_name);
        end
        system(gz_cmd);

        % Give Gazebo time to load plugins before process checks
        pause(2);

        % Detect known Qt symbol mismatch from inherited MATLAB environment
        [qt_err_status, ~] = system('grep -E "Library error for|libQt5Quick|Qt_5_PRIVATE_API" /tmp/gz_sim.log > /dev/null 2>&1');
        if qt_err_status == 0
            fprintf('[Pipeline] ERROR: Gazebo failed due to Qt library conflict. Last log lines:\n');
            system('tail -n 40 /tmp/gz_sim.log');
            error(['Gazebo GUI failed from Qt conflict (MATLAB runtime libs vs system Qt). ', ...
                   'The launcher now sanitizes LD_LIBRARY_PATH/QT_* vars; rerun with clear functions and AutoLandingMainFull(''gui'').']);
        end

        % Verify Gazebo actually started before moving on
        [gz_status, ~] = system('pgrep -f "gz sim" > /dev/null');
        if gz_status ~= 0
            fprintf('[Pipeline] ERROR: Gazebo failed to start. Last log lines:\n');
            system('tail -n 20 /tmp/gz_sim.log');
            error('Gazebo startup failed.');
        end

        % In GUI mode, ensure GUI process is alive (not server-only fallback)
        if ~gazebo_server_mode
            pause(1);
            [gui_status, ~] = system('pgrep -f "gz sim gui" > /dev/null');
            if gui_status ~= 0
                fprintf('[Pipeline] ERROR: Gazebo GUI process not detected. Last log lines:\n');
                system('tail -n 30 /tmp/gz_sim.log');
                error('Gazebo GUI did not start. Check DISPLAY and GZ_SIM_RESOURCE_PATH.');
            end
        end
        pause(10);
        
        % Start ArduPilot SITL (single or multi-drone)
        fprintf('[Pipeline] Starting ArduPilot SITL...\n');
        if enable_multi_drone_profiles
            launch_multi_sitl = fullfile(rootDir, 'scripts', 'launch_multi_drone_sitl.sh');
            if isfile(launch_multi_sitl)
                ap_cmd = sprintf('bash -lc ''"%s" %d > /tmp/autolanding_multi_sitl.log 2>&1''', launch_multi_sitl, num_workers);
            else
                warning('Multi-drone SITL launcher not found: %s. Falling back to single instance.', launch_multi_sitl);
                ap_cmd = sprintf('nohup $HOME/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 --defaults $HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I0 > /tmp/ardupilot_sitl.log 2>&1 &');
            end
        else
            ap_cmd = sprintf('nohup $HOME/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 --defaults $HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I0 > /tmp/ardupilot_sitl.log 2>&1 &');
        end
        system(ap_cmd);
        pause(5);
        
        % Extra wait time to ensure processes started
        fprintf('[Pipeline] Waiting for ArduPilot initialization...\n');
        pause(10);

        [cam_topic_ok, ~] = system('bash -lc ''timeout 3 gz topic -l | grep -E "^/camera$" > /dev/null''');
        if cam_topic_ok == 0
            fprintf('[Pipeline] Camera topic check: /camera available in Gazebo transport\n');
        else
            fprintf('[Pipeline] WARNING: /camera not found in Gazebo transport topics\n');
        end

        [aruco_topic_ok, ~] = system('bash -lc ''timeout 3 ros2 topic list 2>/dev/null | grep -E "^/aruco_markers$" > /dev/null''');
        if aruco_topic_ok == 0
            fprintf('[Pipeline] ArUco topic check: /aruco_markers available in ROS2\n');
        else
            fprintf('[Pipeline] WARNING: /aruco_markers not detected. Marker-vision loop is not active yet (bridge/detector launch needed).\n');
        end

        if enable_rviz_monitor
            autlLaunchRvizMonitor(rootDir, rviz_ros_domain_id, num_workers);
        end
        fprintf('[Pipeline] ✓ Simulation environment ready\n');
        
        %% STEP 1: Parallel Data Collection
        fprintf('\n[Pipeline] STEP 1: Parallel Data Collection\n');
        fprintf('════════════════════════════════════════════\n');
        mission_overrides = struct();
        mission_overrides.enable_auto_motion = enable_auto_motion;
        mission_overrides.takeoff_height_m = takeoff_height_m;
        mission_overrides.landing_pad_center = landing_pad_center;
        mission_overrides.landing_pad_size = landing_pad_size;
        mission_overrides.landing_pad_topic = landing_pad_topic;
        mission_overrides.telemetry_query_interval_s = 1.0;
        mission_overrides.control_interval_s = 1.0;
        mission_overrides.reset_pose_each_scenario = true;
        mission_overrides.reset_spawn_xy = spawn_xy;
        mission_overrides.reset_spawn_z_m = 0.195;
        mission_overrides.reset_spawn_yaw_deg = spawn_yaw_deg;
        mission_overrides.reset_model_name = 'iris_with_gimbal';
        mission_overrides.reset_ardupilot_each_scenario = true;
        mission_overrides.reset_mode_sequence = {'STABILIZE', 'GUIDED'};

        if enable_multi_drone_profiles
            mission_overrides.worker_profiles = worker_profiles;
        end
        collection_result = AutoLandingDataParallel(num_workers, scenarios_per_worker, gazebo_server_mode, enable_visualization, mission_overrides);
        collection_dir = collection_result{1};  % Session directory
        num_collected = collection_result{2};   % Number of scenarios collected
        
        fprintf('[Pipeline] Collected %d scenarios\n', num_collected);
        
        %% STEP 2: Load and Prepare Training Data
        fprintf('\n[Pipeline] STEP 2: Load and Prepare Data\n');
        fprintf('════════════════════════════════════════════\n');
        
        [X_train, y_train, X_val, y_val, data_summary] = ...
            autlLoadAndPrepareTrainingData(rootDir, collection_dir, num_train, num_val);
        
        fprintf('[Pipeline] Loaded training data: %d samples\n', size(X_train, 1));
        fprintf('[Pipeline] Loaded validation data: %d samples\n', size(X_val, 1));
        
        %% STEP 3: Train Ontology+AI Hybrid Model
        model_onto_ai = [];
        if use_ontology_model
            fprintf('\n[Pipeline] STEP 3A: Training Ontology+AI Hybrid Model\n');
            fprintf('════════════════════════════════════════════\n');
            
            model_onto_ai = autlTrainHybridModel(X_train, y_train, X_val, y_val, rootDir);
            
            % Validation
            [acc_onto_ai, metrics_onto_ai] = autlEvaluateModel(model_onto_ai, X_val, y_val);
            fprintf('[Pipeline] Ontology+AI Model Validation Accuracy: %.4f\n', acc_onto_ai);
        end
        
        %% STEP 4: Train Pure AI Baseline Model
        model_pure_ai = [];
        if use_pure_ai_model
            fprintf('\n[Pipeline] STEP 3B: Training Pure AI Baseline Model\n');
            fprintf('════════════════════════════════════════════\n');
            
            model_pure_ai = autlTrainPureAiModel(X_train, y_train, X_val, y_val, rootDir);
            
            % Validation
            [acc_pure_ai, metrics_pure_ai] = autlEvaluateModel(model_pure_ai, X_val, y_val);
            fprintf('[Pipeline] Pure AI Model Validation Accuracy: %.4f\n', acc_pure_ai);
        end
        
        %% STEP 5: Model Comparison and Analysis
        fprintf('\n[Pipeline] STEP 4: Model Comparison and Analysis\n');
        fprintf('════════════════════════════════════════════\n');
        
        comparison_result = autlCompareModels(model_onto_ai, model_pure_ai, X_val, y_val, ...
                                              use_ontology_model, use_pure_ai_model);
        
        fprintf('\n%s\n', comparison_result.summary_text);
        
        %% STEP 6: Generate Plots
        if enable_plots
            fprintf('\n[Pipeline] STEP 5: Generating Plots\n');
            fprintf('════════════════════════════════════════════\n');
            
            autlGenerateComparisonPlots(model_onto_ai, model_pure_ai, X_val, y_val, ...
                                        comparison_result, rootDir, use_ontology_model, use_pure_ai_model);
        end

        %% STEP 7: Generate model-specific hover-to-land trajectories and validation report
        fprintf('\n[Pipeline] STEP 6: Generate Hover-to-Land Trajectories\n');
        fprintf('════════════════════════════════════════════\n');
        cfg_traj = autlDefaultConfig();
        hover_initial = struct('x', spawn_xy(1), 'y', spawn_xy(2), 'z', takeoff_height_m);
        pad_target = struct('x', landing_pad_center(1), 'y', landing_pad_center(2), 'z', landing_pad_center(3));

        processed_dir = fullfile(rootDir, 'data', 'processed');
        if ~exist(processed_dir, 'dir')
            mkdir(processed_dir);
        end

        ref_feature = autlBuildReferenceFeatureVector(X_train, X_val);
        traj_reports = struct([]);
        report_idx = 0;

        if use_ontology_model && ~isempty(model_onto_ai)
            [onto_conf, onto_src] = autlEstimateModelConfidence(model_onto_ai, ref_feature, 0.90);
            traj_onto = autlGenerateTrajectory(hover_initial, pad_target, struct('fused_confidence', onto_conf), cfg_traj);
            out_csv = fullfile(processed_dir, 'landing_trajectory_ontology_ai.csv');
            out_json = fullfile(processed_dir, 'landing_trajectory_ontology_ai.json');
            autlWriteTrajectoryCsv(out_csv, traj_onto);
            onto_metrics = autlEvaluateLandingTrajectory(traj_onto, pad_target);
            autlSaveJson(out_json, struct('model', 'ontology_ai', 'confidence', onto_conf, ...
                'confidence_source', onto_src, 'spawn_xy', spawn_xy, 'hover_alt_m', takeoff_height_m, ...
                'pad_center', landing_pad_center, 'pad_size', landing_pad_size, ...
                'metrics', onto_metrics, 'trajectory', table2struct(traj_onto)));
            fprintf('[Pipeline] Ontology+AI trajectory saved: %s (confidence=%.3f, source=%s)\n', out_csv, onto_conf, onto_src);

            report_idx = report_idx + 1;
            traj_reports(report_idx).model = 'ontology_ai';
            traj_reports(report_idx).confidence = onto_conf;
            traj_reports(report_idx).confidence_source = onto_src;
            traj_reports(report_idx).csv_path = out_csv;
            traj_reports(report_idx).json_path = out_json;
            traj_reports(report_idx).metrics = onto_metrics;
            traj_reports(report_idx).trajectory = traj_onto;
        end

        if use_pure_ai_model && ~isempty(model_pure_ai)
            [pure_conf, pure_src] = autlEstimateModelConfidence(model_pure_ai, ref_feature, 0.70);
            traj_pure = autlGenerateTrajectory(hover_initial, pad_target, struct('fused_confidence', pure_conf), cfg_traj);
            out_csv = fullfile(processed_dir, 'landing_trajectory_pure_ai.csv');
            out_json = fullfile(processed_dir, 'landing_trajectory_pure_ai.json');
            autlWriteTrajectoryCsv(out_csv, traj_pure);
            pure_metrics = autlEvaluateLandingTrajectory(traj_pure, pad_target);
            autlSaveJson(out_json, struct('model', 'pure_ai', 'confidence', pure_conf, ...
                'confidence_source', pure_src, 'spawn_xy', spawn_xy, 'hover_alt_m', takeoff_height_m, ...
                'pad_center', landing_pad_center, 'pad_size', landing_pad_size, ...
                'metrics', pure_metrics, 'trajectory', table2struct(traj_pure)));
            fprintf('[Pipeline] Pure AI trajectory saved: %s (confidence=%.3f, source=%s)\n', out_csv, pure_conf, pure_src);

            report_idx = report_idx + 1;
            traj_reports(report_idx).model = 'pure_ai';
            traj_reports(report_idx).confidence = pure_conf;
            traj_reports(report_idx).confidence_source = pure_src;
            traj_reports(report_idx).csv_path = out_csv;
            traj_reports(report_idx).json_path = out_json;
            traj_reports(report_idx).metrics = pure_metrics;
            traj_reports(report_idx).trajectory = traj_pure;
        end

        if isempty(traj_reports)
            fprintf('[Pipeline] No trained model available. Falling back to baseline tracking trajectory.\n');
            base_conf = cfg_traj.validation.decision_threshold;
            base_traj = autlGenerateTrajectory(hover_initial, pad_target, struct('fused_confidence', base_conf), cfg_traj);
            out_csv = fullfile(processed_dir, 'landing_trajectory_baseline_tracking.csv');
            out_json = fullfile(processed_dir, 'landing_trajectory_baseline_tracking.json');
            autlWriteTrajectoryCsv(out_csv, base_traj);
            base_metrics = autlEvaluateLandingTrajectory(base_traj, pad_target);
            autlSaveJson(out_json, struct('model', 'baseline_tracking', 'confidence', base_conf, ...
                'confidence_source', 'fallback_threshold', 'spawn_xy', spawn_xy, 'hover_alt_m', takeoff_height_m, ...
                'pad_center', landing_pad_center, 'pad_size', landing_pad_size, ...
                'metrics', base_metrics, 'trajectory', table2struct(base_traj)));

            traj_reports = struct('model', 'baseline_tracking', 'confidence', base_conf, ...
                'confidence_source', 'fallback_threshold', 'csv_path', out_csv, 'json_path', out_json, ...
                'metrics', base_metrics, 'trajectory', base_traj);
            fprintf('[Pipeline] Baseline tracking trajectory saved: %s\n', out_csv);
        end

        % Keep legacy single-file output for downstream compatibility.
        preferred_idx = 1;
        for i = 1:numel(traj_reports)
            if strcmpi(traj_reports(i).model, 'ontology_ai')
                preferred_idx = i;
                break;
            end
        end
        aruco_traj = traj_reports(preferred_idx).trajectory;
        aruco_traj_csv = fullfile(processed_dir, 'landing_trajectory_aruco_hover.csv');
        aruco_traj_json = fullfile(processed_dir, 'landing_trajectory_aruco_hover.json');
        autlWriteTrajectoryCsv(aruco_traj_csv, aruco_traj);
        autlSaveJson(aruco_traj_json, struct('model', traj_reports(preferred_idx).model, ...
            'confidence', traj_reports(preferred_idx).confidence, ...
            'confidence_source', traj_reports(preferred_idx).confidence_source, ...
            'marker_id', aruco_marker_id, 'spawn_xy', spawn_xy, 'hover_alt_m', takeoff_height_m, ...
            'pad_center', landing_pad_center, 'pad_size', landing_pad_size, ...
            'metrics', traj_reports(preferred_idx).metrics, ...
            'trajectory', table2struct(aruco_traj)));

        validation_report = struct();
        validation_report.experiment = 'model_trajectory_landing_validation';
        validation_report.models = traj_reports;
        validation_report.comparison = autlBuildTrajectoryComparison(traj_reports);
        validation_report_path = fullfile(processed_dir, 'landing_trajectory_model_validation.json');
        autlSaveJson(validation_report_path, validation_report);
        fprintf('[Pipeline] Trajectory validation report saved: %s\n', validation_report_path);
        
        %% FINAL SUMMARY
        fprintf('\n');
        fprintf('═════════════════════════════════════════════════════════════\n');
        fprintf('  Pipeline Complete!\n');
        fprintf('═════════════════════════════════════════════════════════════\n');
        fprintf('\nOUTPUT FILES:\n');
        fprintf('  - Raw Data: %s\n', collection_dir);
        fprintf('  - ArUco Trajectory: %s\n', aruco_traj_csv);
        fprintf('  - Trajectory Validation: %s\n', validation_report_path);
        fprintf('  - Models: %s/models/\n', rootDir);
        fprintf('  - Plots: %s/plots/\n', rootDir);
        fprintf('  - Analysis: %s/analysis/\n', rootDir);
        fprintf('\n');
        
    catch ME
        % Handle interruption or errors
        if contains(lower(ME.message), 'interrupt') || contains(lower(ME.identifier), 'interrupt')
            fprintf('[Pipeline] User interrupted. Performing cleanup...\n');
        else
            fprintf('[Pipeline] ERROR: %s\n', ME.message);
            fprintf('%s\n', getReport(ME));
        end
        rethrow(ME);
    end
end

function autlCreateArucoLandingWorld(base_world_path, output_world_path, aruco_pkg_dir, marker_center_xyz, marker_size_m, spawn_xy, spawn_yaw_deg, marker_id, num_drones, worker_profiles)
% Generate a temporary world using ros2-gazebo-aruco marker model and configurable multi-drone spawn.

if ~isfile(base_world_path)
    error('Base world not found: %s', base_world_path);
end
if ~isfolder(fullfile(aruco_pkg_dir, 'gz-world', 'aruco_box'))
    error('ArUco model directory not found: %s', fullfile(aruco_pkg_dir, 'gz-world', 'aruco_box'));
end

if nargin < 9 || isempty(num_drones)
    num_drones = 1;
end
if nargin < 10
    worker_profiles = struct([]);
end

world_text = fileread(base_world_path);

% Remove the default single-drone include; we will inject explicit includes for all drones.
world_text = regexprep(world_text, '<include>\s*<uri>model://iris_with_gimbal</uri>[\s\S]*?</include>', '', 'once');

drone_includes = '';
for i = 1:num_drones
    if ~isempty(worker_profiles) && numel(worker_profiles) >= i
        p = worker_profiles(i);
        if isfield(p, 'reset_spawn_xy') && numel(p.reset_spawn_xy) >= 2
            spawn_i = double(p.reset_spawn_xy(1:2));
        else
            ang_i = deg2rad(spawn_yaw_deg + 18.0 * (i - 1));
            spawn_i = [spawn_xy(1) + 0.35 * cos(ang_i), spawn_xy(2) + 0.35 * sin(ang_i)];
        end
        if isfield(p, 'reset_spawn_yaw_deg')
            yaw_i = double(p.reset_spawn_yaw_deg);
        else
            yaw_i = spawn_yaw_deg;
        end
        if isfield(p, 'reset_model_name')
            name_i = char(string(p.reset_model_name));
        elseif i == 1
            name_i = 'iris_with_gimbal';
        else
            name_i = sprintf('iris_with_gimbal_%d', i - 1);
        end
    else
        ang_i = deg2rad(spawn_yaw_deg + 18.0 * (i - 1));
        spawn_i = [spawn_xy(1) + 0.35 * cos(ang_i), spawn_xy(2) + 0.35 * sin(ang_i)];
        yaw_i = spawn_yaw_deg;
        if i == 1
            name_i = 'iris_with_gimbal';
        else
            name_i = sprintf('iris_with_gimbal_%d', i - 1);
        end
    end

    drone_pose_i = sprintf('%.3f %.3f 0.195 0 0 %.2f', spawn_i(1), spawn_i(2), yaw_i);
    drone_includes = [drone_includes, sprintf([ ...
        '    <include>\n' ...
        '      <name>%s</name>\n' ...
        '      <uri>model://iris_with_gimbal</uri>\n' ...
        '      <pose degrees="true">%s</pose>\n' ...
        '    </include>\n'], name_i, drone_pose_i)]; %#ok<AGROW>
end

    % ros2-gazebo-aruco tutorial uses ~0.4 m marker; keep scale near 1 for texture fidelity.
    marker_width_ref_m = 0.4;
aruco_scale = max(0.25, marker_size_m / marker_width_ref_m);
aruco_pose_z = marker_center_xyz(3) + (0.25 * aruco_scale);

aruco_model = sprintf([ ...
    '\n    <include>\n' ...
    '      <name>aruco_landing_box</name>\n' ...
    '      <uri>model://aruco_box</uri>\n' ...
    '      <pose>%.3f %.3f %.3f 0 0 0</pose>\n' ...
    '      <scale>%.3f %.3f %.3f</scale>\n' ...
    '    </include>\n' ...
    '    <!-- aruco_marker_id: %d -->\n'], ...
    marker_center_xyz(1), marker_center_xyz(2), aruco_pose_z, ...
    aruco_scale, aruco_scale, aruco_scale, marker_id);

cam_x = marker_center_xyz(1) + 4.0;
cam_y = marker_center_xyz(2);
cam_z = marker_center_xyz(3) + 1.6;
cam_yaw = pi;
cam_pitch = -atan2(cam_z - marker_center_xyz(3), max(0.1, hypot(marker_center_xyz(1) - cam_x, marker_center_xyz(2) - cam_y)));

camera_model = sprintf([ ...
    '\n    <model name="aruco_observer_camera">\n' ...
    '      <static>true</static>\n' ...
    '      <pose>%.3f %.3f %.3f 0 %.6f %.6f</pose>\n' ...
    '      <link name="link">\n' ...
    '        <sensor name="camera" type="camera">\n' ...
    '          <camera>\n' ...
    '            <horizontal_fov>1.047</horizontal_fov>\n' ...
    '            <image><width>640</width><height>480</height></image>\n' ...
    '            <clip><near>0.1</near><far>100</far></clip>\n' ...
    '          </camera>\n' ...
    '          <always_on>1</always_on>\n' ...
    '          <update_rate>30</update_rate>\n' ...
    '          <topic>camera</topic>\n' ...
    '        </sensor>\n' ...
    '      </link>\n' ...
    '    </model>\n'], ...
    cam_x, cam_y, cam_z, cam_pitch, cam_yaw);

world_text = strrep(world_text, '</world>', [drone_includes aruco_model camera_model '  </world>']);

fid = fopen(output_world_path, 'w');
if fid < 0
    error('Failed to write generated world: %s', output_world_path);
end
fprintf(fid, '%s', world_text);
fclose(fid);
end

function profiles = autlBuildWorkerProfiles(num_workers, base_spawn_xy, base_spawn_yaw_deg, base_takeoff_height_m, landing_pad_center, landing_pad_size)
% Build worker-specific profiles so each worker can represent a distinct drone state.

profiles = repmat(struct(), num_workers, 1);
state_cycle = {'aggressive', 'conservative', 'orbit-heavy', 'balanced'};

for i = 1:num_workers
    phase = (i - 1) * (2*pi / max(1, num_workers));
    spawn_offset = 0.35 * [cos(phase), sin(phase)];
    spawn_xy_i = base_spawn_xy + spawn_offset;
    yaw_i = base_spawn_yaw_deg + (i - 1) * 12.0;

    profiles(i).worker_state_tag = sprintf('drone_%d_%s', i, state_cycle{mod(i-1, numel(state_cycle)) + 1});
    profiles(i).mavlink_master = sprintf('tcp:127.0.0.1:%d', 5762 + 10 * (i - 1));
    profiles(i).mavlink_master_fallback = sprintf('tcp:127.0.0.1:%d', 5760 + 10 * (i - 1));
    if i == 1
        profiles(i).reset_model_name = 'iris_with_gimbal';
    else
        profiles(i).reset_model_name = sprintf('iris_with_gimbal_%d', i - 1);
    end
    profiles(i).reset_spawn_xy = spawn_xy_i;
    profiles(i).reset_spawn_yaw_deg = yaw_i;
    profiles(i).takeoff_height_m = base_takeoff_height_m + 0.3 * (i - 1);
    profiles(i).landing_pad_center = landing_pad_center;
    profiles(i).landing_pad_size = landing_pad_size;

    state_name = state_cycle{mod(i-1, numel(state_cycle)) + 1};
    profiles(i).motion_profile = state_name;
    switch state_name
        case 'aggressive'
            profiles(i).motion_gain_xy = 0.55;
            profiles(i).motion_gain_orbit = 0.70;
            profiles(i).motion_min_move_ms = 0.85;
            profiles(i).motion_vxy_limit_ms = 1.8;
            profiles(i).motion_vz_limit_ms = 0.8;
        case 'conservative'
            profiles(i).motion_gain_xy = 0.22;
            profiles(i).motion_gain_orbit = 0.24;
            profiles(i).motion_min_move_ms = 0.25;
            profiles(i).motion_vxy_limit_ms = 0.8;
            profiles(i).motion_vz_limit_ms = 0.4;
        case 'orbit-heavy'
            profiles(i).motion_gain_xy = 0.28;
            profiles(i).motion_gain_orbit = 0.90;
            profiles(i).motion_min_move_ms = 0.60;
            profiles(i).motion_vxy_limit_ms = 1.4;
            profiles(i).motion_vz_limit_ms = 0.5;
        otherwise
            profiles(i).motion_gain_xy = 0.32;
            profiles(i).motion_gain_orbit = 0.42;
            profiles(i).motion_min_move_ms = 0.55;
            profiles(i).motion_vxy_limit_ms = 1.2;
            profiles(i).motion_vz_limit_ms = 0.6;
    end
end
end

function autlLaunchRvizMonitor(rootDir, ros_domain_id, num_workers)
% Launch RViz monitor in background with robust local environment loading.

launch_script = fullfile(rootDir, 'scripts', 'launch_rviz_monitor.sh');
if ~isfile(launch_script)
    fprintf('[Pipeline] WARNING: RViz launcher not found: %s\n', launch_script);
    return;
end

if nargin < 2 || strlength(string(ros_domain_id)) == 0
    ros_domain_id = 'auto';
end
if nargin < 3 || isempty(num_workers)
    num_workers = 1;
end

cmd = sprintf('bash -lc ''nohup "%s" --domain "%s" --workers %d > /tmp/autolanding_rviz.log 2>&1 &''', ...
    launch_script, char(string(ros_domain_id)), num_workers);
[rc, ~] = system(cmd);
if rc == 0
    fprintf('[Pipeline] RViz monitor launch requested (log: /tmp/autolanding_rviz.log)\n');
else
    fprintf('[Pipeline] WARNING: Failed to launch RViz monitor (rc=%d)\n', rc);
end
end

function autlMainCleanup()
% Cleanup function called on any exit (Ctrl+C or normal termination)

try
    fprintf('\n[Pipeline.cleanup] Cleaning up resources...\n');
    
    % Kill background processes to prevent port conflicts
    system('pkill -f "gz sim" 2>/dev/null');
    system('pkill -f "arducopter" 2>/dev/null');
    system('pkill -f "mavproxy.py" 2>/dev/null');
    
    % Delete parallel pool if exists
    pool = gcp('nocreate');
    if ~isempty(pool)
        delete(pool);
    end
    
    fprintf('[Pipeline.cleanup] Cleanup complete.\n');
catch
    % Silent fail to not interfere with cleanup
end

end

function x_ref = autlBuildReferenceFeatureVector(X_train, X_val)
% Build a representative normalized feature vector for model-conditioned trajectory generation.

if nargin < 1
    X_train = [];
end
if nargin < 2
    X_val = [];
end

if ~isempty(X_train)
    x_ref = mean(X_train, 1);
elseif ~isempty(X_val)
    x_ref = X_val(1, :);
else
    x_ref = zeros(1, 14);
end
end

function [confidence, source] = autlEstimateModelConfidence(model, x_ref, default_conf)
% Estimate landing confidence from model output, with safe fallback.

if nargin < 3
    default_conf = 0.70;
end

confidence = autlClamp(default_conf, 0.05, 0.98);
source = 'default';

if isempty(model) || ~isfield(model, 'classifier') || isempty(model.classifier) || isempty(x_ref)
    return;
end

try
    if isfield(model, 'is_nn') && model.is_nn
        nn_out = model.classifier(double(x_ref(:))');
        nn_out = double(nn_out(:));
        if numel(nn_out) >= 2
            positive = max(nn_out(2), 0);
            total = max(sum(max(nn_out, 0)), 1e-6);
            confidence = autlClamp(positive / total, 0.05, 0.98);
            source = 'nn_score';
            return;
        end
    end

    try
        [pred_label, score] = predict(model.classifier, x_ref);
        if ~isempty(score) && size(score, 2) >= 2
            confidence = autlClamp(double(score(1, 2)), 0.05, 0.98);
            source = 'classifier_score';
            return;
        end
        pred_label = double(pred_label(1));
        if pred_label >= 1
            confidence = autlClamp(default_conf + 0.10, 0.05, 0.98);
        else
            confidence = autlClamp(default_conf - 0.20, 0.05, 0.98);
        end
        source = 'classifier_label';
    catch
        % Keep default if classifier does not expose score API.
    end
catch
    % Keep default on estimation failures.
end
end

function metrics = autlEvaluateLandingTrajectory(trajTbl, targetState)
% Compute landing-centric metrics for paper-ready model comparison.

metrics = autlEvaluateTrajectoryMetrics(trajTbl, targetState);

if isempty(trajTbl)
    metrics.total_time_s = nan;
    metrics.mean_descent_speed_ms = nan;
    metrics.touchdown_vertical_speed_ms = nan;
    metrics.lateral_speed_stability = nan;
    return;
end

metrics.total_time_s = double(trajTbl.t(end));

if ismember('vz_cmd', trajTbl.Properties.VariableNames)
    vz = double(trajTbl.vz_cmd);
    descent_mask = vz < 0;
    if any(descent_mask)
        metrics.mean_descent_speed_ms = mean(abs(vz(descent_mask)));
    else
        metrics.mean_descent_speed_ms = 0.0;
    end
    metrics.touchdown_vertical_speed_ms = abs(vz(end));
else
    metrics.mean_descent_speed_ms = nan;
    metrics.touchdown_vertical_speed_ms = nan;
end

if ismember('vx_cmd', trajTbl.Properties.VariableNames) && ismember('vy_cmd', trajTbl.Properties.VariableNames)
    vxy = hypot(double(trajTbl.vx_cmd), double(trajTbl.vy_cmd));
    metrics.lateral_speed_stability = std(vxy);
else
    metrics.lateral_speed_stability = nan;
end
end

function comp = autlBuildTrajectoryComparison(traj_reports)
% Build summary comparison values across model trajectories.

comp = struct();
comp.num_models = numel(traj_reports);
comp.best_touchdown_error_model = '';
comp.best_touchdown_error_xy_m = nan;
comp.fastest_landing_model = '';
comp.fastest_total_time_s = nan;

if isempty(traj_reports)
    return;
end

touchdown_err = nan(numel(traj_reports), 1);
landing_time = nan(numel(traj_reports), 1);
for i = 1:numel(traj_reports)
    touchdown_err(i) = traj_reports(i).metrics.touchdown_error_xy;
    landing_time(i) = traj_reports(i).metrics.total_time_s;
end

[min_err, idx_err] = min(touchdown_err);
[min_time, idx_time] = min(landing_time);

if ~isempty(idx_err) && isfinite(min_err)
    comp.best_touchdown_error_model = traj_reports(idx_err).model;
    comp.best_touchdown_error_xy_m = min_err;
end
if ~isempty(idx_time) && isfinite(min_time)
    comp.fastest_landing_model = traj_reports(idx_time).model;
    comp.fastest_total_time_s = min_time;
end
end
