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
    num_workers = 3;                    % Number of parallel workers
    scenarios_per_worker = 200;           % Scenarios per worker (total = num_workers * scenarios_per_worker)
    enable_auto_motion = true;          % Send MAVLink velocity setpoints during collection
    takeoff_height_m = 3.0;             % Takeoff altitude used before motion commands
    enable_multi_drone_profiles = true; % Enable worker-specific MAVLink/model/spawn/motion profiles
    control_backend = 'mavros';         % 'mavproxy' or 'mavros' (default: mavros)
    control_backend_fallback = false;   % MAVROS 모드에서는 TCP 폴백을 기본 비활성화(링크 churn 방지)
    mavros_namespace = '/mavros';       % ROS2 MAVROS namespace when control_backend='mavros'
    mavros_namespace_prefix = '/mavros_w'; % Worker-specific MAVROS namespaces: /mavros_w1, /mavros_w2, ...
    auto_launch_mavros_bridge = true;   % Launch MAVROS bridge automatically in STEP 0 when backend=mavros

    % RViz / ROS observability
    enable_rviz_monitor = true;
    ros_domain_id = '0';                % Unified default ROS domain

    % ArUco landing pad configuration
    use_aruco_landing_pad = true;
    aruco_marker_id = 23;
    drone_body_size_m = 0.1;            % Reference body size for marker scaling (0.1 * 2^2 = 0.4 m)
    marker_size_multiplier = 3^2;       % Requested: 2 squared times drone size
    marker_size_m = drone_body_size_m * marker_size_multiplier;
    landing_pad_center = [0.0, 0.0, 0.0];   % World center / primary ArUco pad position [x, y, z]
    landing_pad_size = [marker_size_m, marker_size_m];
    landing_pad_topic = '/autolanding/landing_pad';
    aruco_markers_topic = '/aruco_markers';
    aruco_visibility_probe_timeout_s = 1.0;
    aruco_visibility_poll_interval_s = 2.0;
    aruco_visibility_min_markers = 1;
    spawn_layout_mode = 'auto';          % auto = square-ish, square = near-square, rectangle = wider rectangle
    spawn_layout_spacing_m = 12.2;        % Tighter spacing so drones stay near the central landing pad
    spawn_layout_center_xy = landing_pad_center(1:2);
    mavlink_init_timeout_s = 120.0;     % Startup heartbeat readiness budget (seconds)
    mavlink_poll_interval_s = 1.0;      % Polling interval for startup heartbeat checks
    sitl_json_ready_timeout_s = 90.0;   % Wait budget for SITL JSON sensor bridge readiness

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
    fprintf('    - Control Backend: %s (fallback=%s, ns=%s)\n', ...
        char(string(control_backend)), char(string(control_backend_fallback)), char(string(mavros_namespace)));
    fprintf('    - MAVROS Auto Launch: %s (prefix=%s)\n', ...
        char(string(auto_launch_mavros_bridge)), char(string(mavros_namespace_prefix)));
    fprintf('    - Landing Pad Topic: %s\n', landing_pad_topic);
    fprintf('    - ArUco Enabled: %s (id=%d)\n', char(string(use_aruco_landing_pad)), aruco_marker_id);
    fprintf('    - ArUco Markers Topic: %s\n', aruco_markers_topic);
    fprintf('    - ArUco Visibility Poll: %.1fs every %.1fs (%d marker minimum)\n', ...
        aruco_visibility_probe_timeout_s, aruco_visibility_poll_interval_s, aruco_visibility_min_markers);
    fprintf('    - RViz Monitor: %s (ROS_DOMAIN_ID=%s)\n', char(string(enable_rviz_monitor)), char(string(ros_domain_id)));
    fprintf('    - Landing Pad Center: [%.2f, %.2f, %.2f]\n', landing_pad_center(1), landing_pad_center(2), landing_pad_center(3));
    fprintf('    - Landing Pad Size: [%.2f x %.2f] m\n', landing_pad_size(1), landing_pad_size(2));
    fprintf('    - Formation Layout: %s\n', char(string(spawn_layout_mode)));
    fprintf('    - Formation Spacing: %.2f m\n', spawn_layout_spacing_m);
    fprintf('    - MAVLink Init Timeout: %.1f s\n', mavlink_init_timeout_s);
    fprintf('    - SITL JSON Ready Timeout: %.1f s\n', sitl_json_ready_timeout_s);
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
        system('pkill -f "ros_gz_bridge parameter_bridge" 2>/dev/null');
        system('pkill -f "ros2_aruco" 2>/dev/null');
        system('pkill -f "publish_multi_drone_odom.py" 2>/dev/null');
        system('pkill -f "mavros_node" 2>/dev/null');
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

        [drone_spawn_xy, pad_spawn_xy, layout_dims] = autlBuildFormationLayout(num_workers, spawn_layout_center_xy, ...
            spawn_layout_spacing_m, spawn_layout_mode);
        spawn_xy = drone_spawn_xy(1, :);
        spawn_yaw_deg = rad2deg(atan2(pad_spawn_xy(1, 2) - drone_spawn_xy(1, 2), pad_spawn_xy(1, 1) - drone_spawn_xy(1, 1)));
        layout_extent_m = max(vecnorm(drone_spawn_xy - spawn_layout_center_xy, 2, 2));
        fprintf('[Pipeline] Formation grid: %d x %d, extent=%.2f m\n', layout_dims(1), layout_dims(2), layout_extent_m);

        worker_profiles = struct([]);
        if enable_multi_drone_profiles
            worker_profiles = autlBuildWorkerProfiles(num_workers, drone_spawn_xy, pad_spawn_xy, takeoff_height_m, landing_pad_center, landing_pad_size);
        end

        world_path_to_launch = base_world_path;
        if use_aruco_landing_pad
            generated_world_path = fullfile('/tmp', 'iris_runway_aruco_landing.sdf');
            autlCreateArucoLandingWorld(base_world_path, generated_world_path, aruco_pkg_dir, ...
                landing_pad_center, marker_size_m, spawn_xy, spawn_yaw_deg, aruco_marker_id, num_workers, worker_profiles);
            world_path_to_launch = generated_world_path;
            fprintf('[Pipeline] ArUco world generated: %s\n', world_path_to_launch);
            autlAssertDroneIncludeCount(world_path_to_launch, num_workers);
        end

        launch_world_dir = fileparts(world_path_to_launch);
        launch_world_name = world_path_to_launch;
        display_env = getenv('DISPLAY');
        if strlength(string(display_env)) == 0
            display_env = ':0';
        end
        xauth_env = getenv('XAUTHORITY');

        % Resource path is required so model:// URIs resolve (iris_with_gimbal, ArUco, ...)
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
            gz_cmd = sprintf('bash -lc ''%s export GZ_SIM_SYSTEM_PLUGIN_PATH="%s:${GZ_SIM_SYSTEM_PLUGIN_PATH}"; export GZ_SIM_RESOURCE_PATH="%s"; cd "%s" && setsid gz sim -s -v4 -r "%s" </dev/null > /tmp/gz_sim.log 2>&1 &''', gz_clean_env, gz_system_plugin_path, gz_resource_path, launch_world_dir, launch_world_name);
        else
            fprintf('[Pipeline] Starting Gazebo (GUI mode)...\n');
            fprintf('[Pipeline] Display: %s\n', display_env);
            gz_cmd = sprintf('bash -lc ''%s export DISPLAY="%s"; %s export QT_QPA_PLATFORM=xcb; export GZ_SIM_SYSTEM_PLUGIN_PATH="%s:${GZ_SIM_SYSTEM_PLUGIN_PATH}"; export GZ_SIM_RESOURCE_PATH="%s"; xhost +local: 2>/dev/null || true; cd "%s" && setsid gz sim -v4 -r "%s" </dev/null > /tmp/gz_sim.log 2>&1 &''', gz_clean_env, display_env, xauth_export, gz_system_plugin_path, gz_resource_path, launch_world_dir, launch_world_name);
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
        pause(5);
        
        % Start ArduPilot SITL (single or multi-drone)
        fprintf('[Pipeline] Starting ArduPilot SITL...\n');
        if enable_multi_drone_profiles
            launch_multi_sitl = fullfile(rootDir, 'scripts', 'launch_multi_drone_sitl.sh');
            if isfile(launch_multi_sitl)
                ap_cmd = sprintf('bash -lc ''setsid "%s" %d </dev/null > /tmp/autolanding_multi_sitl.log 2>&1 &''', launch_multi_sitl, num_workers);
            else
                warning('Multi-drone SITL launcher not found: %s. Falling back to single instance.', launch_multi_sitl);
                ap_cmd = sprintf('nohup setsid $HOME/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 --defaults $HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I0 </dev/null > /tmp/ardupilot_sitl.log 2>&1 &');
            end
        else
            ap_cmd = sprintf('nohup setsid $HOME/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 --defaults $HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I0 </dev/null > /tmp/ardupilot_sitl.log 2>&1 &');
        end
        system(ap_cmd);
        pause(5);
        
        % Give SITL and Gazebo a brief overlap window before readiness probes.
        fprintf('[Pipeline] Waiting for ArduPilot initialization before readiness probes...\n');
        pause(15);  % Brief boot window before polling JSON and MAVLink readiness

        [cam_topic_ok, ~] = system('bash -lc ''timeout 3 gz topic -l | grep -E "^/camera$" > /dev/null''');
        if cam_topic_ok == 0
            fprintf('[Pipeline] Camera topic check: /camera available in Gazebo transport\n');
        else
            fprintf('[Pipeline] WARNING: /camera not found in Gazebo transport topics\n');
        end

        fprintf('[Pipeline] ArUco topic pre-check deferred until RViz/bridge launch completes\n');

        resolved_ros_domain_id = autlResolveRosDomainId(ros_domain_id);
        setenv('ROS_DOMAIN_ID', resolved_ros_domain_id);
        fprintf('[Pipeline] ROS domain resolved for MATLAB/workers: %s\n', resolved_ros_domain_id);

        if strcmpi(control_backend, 'mavros') && auto_launch_mavros_bridge
            autlLaunchMavrosBridge(rootDir, resolved_ros_domain_id, num_workers, mavros_namespace_prefix);
        end

        if enable_rviz_monitor
            autlLaunchRvizMonitor(rootDir, ros_domain_id, num_workers);

            aruco_wait_t0 = tic;
            aruco_wait_timeout_s = 12.0;
            aruco_wait_ok = false;
            while toc(aruco_wait_t0) < aruco_wait_timeout_s
                drawnow limitrate;
                [aruco_topic_ok, ~] = system('bash -lc ''timeout 2 ros2 topic list 2>/dev/null | grep -E "^/aruco_markers$" > /dev/null''');
                if aruco_topic_ok == 0
                    aruco_wait_ok = true;
                    break;
                end
                pause(1.0);
            end

            if aruco_wait_ok
                fprintf('[Pipeline] ArUco topic check: /aruco_markers available in ROS2\n');
            else
                fprintf('[Pipeline] WARNING: /aruco_markers still not detected after %.0fs. Check /tmp/autolanding_aruco.log\n', aruco_wait_timeout_s);
            end
        else
            fprintf('[Pipeline] RViz monitor disabled: skipping /aruco_markers availability check\n');
        end

        % Wait for JSON bridge readiness markers in SITL logs before heartbeat probes.
        [json_ready, json_report] = autlWaitForSitlJsonReady(num_workers, enable_multi_drone_profiles, sitl_json_ready_timeout_s);
        fprintf('%s', json_report);
        if ~json_ready
            error(['SITL JSON bridge is not ready. Check /tmp/autolanding_sitl/arducopter_I*.log and ' ...
                   '/tmp/gz_sim.log for sensor path/port conflicts.']);
        end

        % Fail fast if MAVLink heartbeat is unavailable before parallel workers start.
        [mav_ok, mav_report] = autlVerifyMavlinkHeartbeatReady(num_workers, enable_multi_drone_profiles, ...
            mavlink_init_timeout_s, mavlink_poll_interval_s);
        fprintf('%s', mav_report);
        if ~mav_ok
            error(['MAVLink endpoints are not ready. Check /tmp/autolanding_sitl/arducopter_I*.log and ' ...
                   '/tmp/gz_sim.log for JSON sensor bridge or port-client conflicts.']);
        end
        fprintf('[Pipeline] ✓ Simulation environment ready\n');
        
        %% STEP 1: Parallel Data Collection
        fprintf('\n[Pipeline] STEP 1: Parallel Data Collection\n');
        fprintf('════════════════════════════════════════════\n');
        mission_overrides = struct();
        mission_overrides.enable_auto_motion = enable_auto_motion;
        mission_overrides.control_backend = control_backend;
        mission_overrides.control_backend_fallback = control_backend_fallback;
        mission_overrides.mavros_namespace = mavros_namespace;
        mission_overrides.mavros_namespace_prefix = mavros_namespace_prefix;
        mission_overrides.takeoff_height_m = takeoff_height_m;
        mission_overrides.landing_pad_center = landing_pad_center;
        mission_overrides.landing_pad_size = landing_pad_size;
        mission_overrides.landing_pad_topic = landing_pad_topic;
        mission_overrides.aruco_markers_topic = aruco_markers_topic;
        mission_overrides.aruco_visibility_probe_timeout_s = aruco_visibility_probe_timeout_s;
        mission_overrides.aruco_visibility_poll_interval_s = aruco_visibility_poll_interval_s;
        mission_overrides.aruco_visibility_min_markers = aruco_visibility_min_markers;
        mission_overrides.telemetry_query_interval_s = 2.0;
        mission_overrides.control_interval_s = 2.0;
        mission_overrides.mavlink_precheck_timeout_s = 30.0;
        mission_overrides.mavlink_control_timeout_s = 30.0;
        mission_overrides.mavlink_ready_timeout_s = 10.0;
        mission_overrides.mavlink_ready_poll_interval_s = 0.8;
        mission_overrides.reset_pose_each_scenario = true;
        mission_overrides.reset_spawn_xy = spawn_xy;
        mission_overrides.reset_spawn_z_m = 0.195;
        mission_overrides.reset_spawn_yaw_deg = spawn_yaw_deg;
        mission_overrides.reset_model_name = 'iris_with_gimbal';
        mission_overrides.reset_ardupilot_each_scenario = true;
        mission_overrides.reset_mode_sequence = {'STABILIZE', 'GUIDED'};
        mission_overrides.drone_spawn_distance_m = spawn_layout_spacing_m;
        mission_overrides.landing_pad_distance_m = spawn_layout_spacing_m;

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
        if autlMainIsUserInterrupt(ME)
            fprintf('[Pipeline] User interrupted. Partial outputs were preserved by cleanup.\n');
            return;
        else
            fprintf('[Pipeline] ERROR: %s\n', ME.message);
            fprintf('%s\n', getReport(ME));
        end
        rethrow(ME);
    end
end

function tf = autlMainIsUserInterrupt(ME)
% True when exception corresponds to Ctrl+C/user cancellation.

tf = false;
if nargin < 1 || isempty(ME)
    return;
end

id = lower(string(ME.identifier));
msg = lower(string(ME.message));
tf = contains(id, "operationterminatedbyuser") || ...
     contains(id, "interrupted") || ...
     contains(msg, "operation terminated by user") || ...
     contains(msg, "terminated by user") || ...
     contains(msg, "interrupted") || ...
     contains(msg, "ctrl+c");

if tf
    return;
end

try
    causes = ME.cause;
    for i = 1:numel(causes)
        if autlMainIsUserInterrupt(causes{i})
            tf = true;
            return;
        end
    end
catch
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

% Remove the runway from the generated world so data collection happens over a
% landing-pad scene rather than a runway scene.
world_text = regexprep(world_text, '<include>\s*<uri>model://runway</uri>[\s\S]*?</include>\s*', '', 'once');

% Add a flat ground plane back so the drones do not fall through the scene.
if ~contains(world_text, "<model name='ground_plane'>") && ~contains(world_text, '<model name="ground_plane">')
    ground_plane_block = sprintf([ ...
        '    <model name=''ground_plane''>\n' ...
        '      <static>1</static>\n' ...
        '      <link name=''link''>\n' ...
        '        <collision name=''collision''>\n' ...
        '          <geometry>\n' ...
        '            <plane>\n' ...
        '              <normal>0 0 1</normal>\n' ...
        '              <size>100 100</size>\n' ...
        '            </plane>\n' ...
        '          </geometry>\n' ...
        '          <surface>\n' ...
        '            <friction>\n' ...
        '              <ode>\n' ...
        '                <mu>100</mu>\n' ...
        '                <mu2>50</mu2>\n' ...
        '              </ode>\n' ...
        '              <torsional>\n' ...
        '                <ode/>\n' ...
        '              </torsional>\n' ...
        '            </friction>\n' ...
        '            <contact>\n' ...
        '              <ode/>\n' ...
        '            </contact>\n' ...
        '            <bounce/>\n' ...
        '          </surface>\n' ...
        '          <max_contacts>10</max_contacts>\n' ...
        '        </collision>\n' ...
        '        <visual name=''visual''>\n' ...
        '          <cast_shadows>0</cast_shadows>\n' ...
        '          <geometry>\n' ...
        '            <plane>\n' ...
        '              <normal>0 0 1</normal>\n' ...
        '              <size>100 100</size>\n' ...
        '            </plane>\n' ...
        '          </geometry>\n' ...
        '          <material>\n' ...
        '            <script>\n' ...
        '              <uri>file://media/materials/scripts/gazebo.material</uri>\n' ...
        '              <name>Gazebo/Grey</name>\n' ...
        '            </script>\n' ...
        '          </material>\n' ...
        '        </visual>\n' ...
        '        <self_collide>0</self_collide>\n' ...
        '        <enable_wind>0</enable_wind>\n' ...
        '        <kinematic>0</kinematic>\n' ...
        '      </link>\n' ...
        '    </model>\n']);
    world_text = regexprep(world_text, '</world>', [ground_plane_block '  </world>'], 'once');
end

% Remove the default single-drone include; we will inject explicit includes for all drones.
world_text = regexprep(world_text, '<include>\s*<uri>model://iris_with_gimbal</uri>[\s\S]*?</include>', '', 'once');

% ros2-gazebo-aruco tutorial uses ~0.4 m marker; keep scale near 1 for texture fidelity.
marker_width_ref_m = 0.4;
aruco_scale = max(0.25, marker_size_m / marker_width_ref_m);
aruco_pose_z = marker_center_xyz(3) + (0.25 * aruco_scale);

drone_includes = '';
camera_models = '';
aruco_models = '';
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

    fdm_port_in_i = 9002 + 10 * (i - 1);
    fdm_port_out_i = 9003 + 10 * (i - 1);
    % Use single loopback address for all instances and isolate by UDP port.
    % This avoids host-dependent routing quirks with 127.0.0.2/127.0.0.3.
    fdm_addr_i = '127.0.0.1';
    model_uri_i = autlPrepareMultiDroneModelVariant(base_world_path, i, fdm_addr_i, fdm_port_in_i, fdm_port_out_i);

    drone_pose_i = sprintf('%.3f %.3f 0.195 0 0 %.2f', spawn_i(1), spawn_i(2), yaw_i);
    drone_includes = [drone_includes, sprintf([ ...
        '    <include>\n' ...
        '      <name>%s</name>\n' ...
        '      <uri>%s</uri>\n' ...
        '      <pose degrees="true">%s</pose>\n' ...
        '    </include>\n'], name_i, model_uri_i, drone_pose_i)]; %#ok<AGROW>

    % Place one marker near each drone (worker-specific landing pad when available).
    if ~isempty(worker_profiles) && numel(worker_profiles) >= i && ...
            isfield(worker_profiles(i), 'landing_pad_center') && numel(worker_profiles(i).landing_pad_center) >= 2
        marker_xy_i = double(worker_profiles(i).landing_pad_center(1:2));
    else
        marker_offset_m = 1.15;
        marker_xy_i = spawn_i + marker_offset_m * [cosd(yaw_i), sind(yaw_i)];
    end
    marker_id_i = marker_id + (i - 1);
    aruco_models = [aruco_models, sprintf([ ...
        '\n    <include>\n' ...
        '      <name>aruco_landing_box_%d</name>\n' ...
        '      <uri>model://aruco_box</uri>\n' ...
        '      <pose>%.3f %.3f %.3f 0 0 0</pose>\n' ...
        '      <scale>%.3f %.3f %.3f</scale>\n' ...
        '    </include>\n' ...
        '    <!-- aruco_marker_id: %d, drone=%d -->\n'], ...
        i, marker_xy_i(1), marker_xy_i(2), aruco_pose_z, aruco_scale, aruco_scale, aruco_scale, marker_id_i, i)]; %#ok<AGROW>

    % Add worker-specific observer camera near each marker and point to that marker.
    cam_ring_phase = deg2rad(mod((i - 1) * 47.0, 360.0));
    cam_radius = 1.9;
    cam_i_x = marker_xy_i(1) + cam_radius * cos(cam_ring_phase);
    cam_i_y = marker_xy_i(2) + cam_radius * sin(cam_ring_phase);
    cam_i_z = marker_center_xyz(3) + 1.9;
    cam_i_yaw = atan2(marker_xy_i(2) - cam_i_y, marker_xy_i(1) - cam_i_x);
    cam_i_pitch = -atan2(cam_i_z - marker_center_xyz(3), max(0.1, hypot(marker_xy_i(1) - cam_i_x, marker_xy_i(2) - cam_i_y)));
    camera_models = [camera_models, sprintf([ ...
        '\n    <model name="drone%d_observer_camera">\n' ...
        '      <static>true</static>\n' ...
        '      <pose>%.3f %.3f %.3f 0 %.6f %.6f</pose>\n' ...
        '      <link name="link">\n' ...
        '        <sensor name="camera" type="camera">\n' ...
        '          <camera>\n' ...
        '            <horizontal_fov>1.047</horizontal_fov>\n' ...
        '            <image><width>640</width><height>480</height></image>\n' ...
        '            <clip><near>0.1</near><far>120</far></clip>\n' ...
        '          </camera>\n' ...
        '          <always_on>1</always_on>\n' ...
        '          <update_rate>30</update_rate>\n' ...
        '          <topic>/drone%d/camera</topic>\n' ...
        '        </sensor>\n' ...
        '      </link>\n' ...
        '    </model>\n'], ...
        i, cam_i_x, cam_i_y, cam_i_z, cam_i_pitch, cam_i_yaw, i)]; %#ok<AGROW>
end

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

world_text = strrep(world_text, '</world>', [drone_includes aruco_models camera_model camera_models '  </world>']);

fid = fopen(output_world_path, 'w');
if fid < 0
    error('Failed to write generated world: %s', output_world_path);
end
fprintf(fid, '%s', world_text);
fclose(fid);
end

function model_uri = autlPrepareMultiDroneModelVariant(base_world_path, drone_index, fdm_addr, fdm_port_in, fdm_port_out)
% Create per-drone iris model variant with isolated ArduPilotPlugin FDM ports.

% Keep signature stable for callers; out-port is intentionally unused.
if nargin >= 6 %#ok<*INUSD>
    % no-op
end

model_uri = sprintf('model://iris_with_gimbal_w%d', drone_index);

gazebo_pkg_dir = fileparts(fileparts(base_world_path));
template_model_dir = fullfile(gazebo_pkg_dir, 'models', 'iris_with_gimbal');
template_sdf = fullfile(template_model_dir, 'model.sdf');
template_cfg = fullfile(template_model_dir, 'model.config');
gimbal_template_dir = fullfile(gazebo_pkg_dir, 'models', 'gimbal_small_3d');
gimbal_template_sdf = fullfile(gimbal_template_dir, 'model.sdf');
gimbal_template_cfg = fullfile(gimbal_template_dir, 'model.config');

if ~isfile(template_sdf)
    error('Template iris model not found: %s', template_sdf);
end

variant_name = sprintf('iris_with_gimbal_w%d', drone_index);
variant_dir = fullfile('/tmp', variant_name);
if ~isfolder(variant_dir)
    mkdir(variant_dir);
end

gimbal_variant_name = sprintf('gimbal_small_3d_w%d', drone_index);
gimbal_variant_dir = fullfile('/tmp', gimbal_variant_name);
if ~isfolder(gimbal_variant_dir)
    mkdir(gimbal_variant_dir);
end

if isfile(gimbal_template_sdf)
    gimbal_sdf_text = fileread(gimbal_template_sdf);
    gimbal_sdf_text = strrep(gimbal_sdf_text, '<model name="gimbal_small_3d">', sprintf('<model name="%s">', gimbal_variant_name));
    gimbal_udp_port = 5600 + 10 * (drone_index - 1);
    gimbal_sdf_text = regexprep(gimbal_sdf_text, '<udp_port>\s*5600\s*</udp_port>', sprintf('<udp_port>%d</udp_port>', gimbal_udp_port), 'once');

    fid = fopen(fullfile(gimbal_variant_dir, 'model.sdf'), 'w');
    if fid < 0
        error('Failed to write gimbal model variant sdf: %s', fullfile(gimbal_variant_dir, 'model.sdf'));
    end
    fprintf(fid, '%s', gimbal_sdf_text);
    fclose(fid);

    if isfile(gimbal_template_cfg)
        gimbal_cfg_text = fileread(gimbal_template_cfg);
        gimbal_cfg_text = regexprep(gimbal_cfg_text, '<name>\s*[^<]+\s*</name>', sprintf('<name>%s</name>', gimbal_variant_name), 'once');
        fid = fopen(fullfile(gimbal_variant_dir, 'model.config'), 'w');
        if fid < 0
            error('Failed to write gimbal model variant config: %s', fullfile(gimbal_variant_dir, 'model.config'));
        end
        fprintf(fid, '%s', gimbal_cfg_text);
        fclose(fid);
    end
else
    error('Gimbal template not found: %s', gimbal_template_sdf);
end

sdf_text = fileread(template_sdf);
sdf_text = strrep(sdf_text, '<model name="iris_with_gimbal">', sprintf('<model name="%s">', variant_name));
sdf_text = strrep(sdf_text, 'model://gimbal_small_3d', sprintf('model://%s', gimbal_variant_name));

% Update only FDM ports. Keep nested model/link/joint scoped names intact to avoid
% breaking SDF frame graph resolution for included models.
sdf_text = regexprep(sdf_text, '<fdm_addr>\s*[^<]+\s*</fdm_addr>', sprintf('<fdm_addr>%s</fdm_addr>', fdm_addr), 'once');
sdf_text = regexprep(sdf_text, '<fdm_port_in>\s*\d+\s*</fdm_port_in>', sprintf('<fdm_port_in>%d</fdm_port_in>', fdm_port_in), 'once');
% fdm_port_out is deprecated in recent ArduPilotPlugin builds and may add
% noisy reconnect behavior; remove it if present and rely on auto-detection.
sdf_text = regexprep(sdf_text, '\s*<fdm_port_out>\s*\d+\s*</fdm_port_out>\s*', '\n', 'once');

% Multi-instance startup can be staggered; allow longer controller handshake
% before plugin-side timeout/reset to avoid intermittent no-JSON loops.
sdf_text = regexprep(sdf_text, '<connectionTimeoutMaxCount>\s*\d+\s*</connectionTimeoutMaxCount>', ...
    '<connectionTimeoutMaxCount>500</connectionTimeoutMaxCount>', 'once');

% In multi-drone GUI runs, strict lock-step can starve some instances and
% trigger controller reset/drained-packets loops; keep plugin asynchronous.
sdf_text = regexprep(sdf_text, '<lock_step>\s*1\s*</lock_step>', '<lock_step>0</lock_step>', 'once');

fid = fopen(fullfile(variant_dir, 'model.sdf'), 'w');
if fid < 0
    error('Failed to write model variant sdf: %s', fullfile(variant_dir, 'model.sdf'));
end
fprintf(fid, '%s', sdf_text);
fclose(fid);

if isfile(template_cfg)
    cfg_text = fileread(template_cfg);
    cfg_text = regexprep(cfg_text, '<name>\s*iris_with_gimbal\s*</name>', sprintf('<name>%s</name>', variant_name), 'once');
    fid = fopen(fullfile(variant_dir, 'model.config'), 'w');
    if fid < 0
        error('Failed to write model variant config: %s', fullfile(variant_dir, 'model.config'));
    end
    fprintf(fid, '%s', cfg_text);
    fclose(fid);
end
end

function profiles = autlBuildWorkerProfiles(num_workers, drone_spawn_xy, pad_spawn_xy, base_takeoff_height_m, landing_pad_center, landing_pad_size)
% Build worker-specific profiles so each worker follows the shared formation layout.

if nargin < 2 || isempty(drone_spawn_xy)
    drone_spawn_xy = zeros(max(1, num_workers), 2);
end
if nargin < 3 || isempty(pad_spawn_xy)
    pad_spawn_xy = drone_spawn_xy;
end
if nargin < 4 || isempty(base_takeoff_height_m)
    base_takeoff_height_m = 3.0;
end
if nargin < 5 || isempty(landing_pad_center)
    landing_pad_center = [0.0, 0.0, 0.0];
end
if nargin < 6 || isempty(landing_pad_size)
    landing_pad_size = [0.4, 0.4];
end

profiles = repmat(struct(), num_workers, 1);
state_cycle = {'aggressive', 'conservative', 'orbit-heavy', 'balanced'};

for i = 1:num_workers
    spawn_xy_i = double(drone_spawn_xy(i, 1:2));
    pad_xy_i = double(pad_spawn_xy(i, 1:2));
    yaw_i = rad2deg(atan2(pad_xy_i(2) - spawn_xy_i(2), pad_xy_i(1) - spawn_xy_i(1)));

    profiles(i).worker_state_tag = sprintf('drone_%d_%s', i, state_cycle{mod(i-1, numel(state_cycle)) + 1});
    profiles(i).mavlink_master = sprintf('tcp:127.0.0.1:%d', 5760 + 10 * (i - 1));
    profiles(i).mavlink_master_fallback = sprintf('tcp:127.0.0.1:%d', 5762 + 10 * (i - 1));
    profiles(i).mavros_namespace = sprintf('/mavros_w%d', i);
    if i == 1
        profiles(i).reset_model_name = 'iris_with_gimbal';
    else
        profiles(i).reset_model_name = sprintf('iris_with_gimbal_%d', i - 1);
    end
    profiles(i).reset_spawn_xy = spawn_xy_i;
    profiles(i).reset_spawn_yaw_deg = yaw_i;
    profiles(i).takeoff_height_m = base_takeoff_height_m + 0.3 * (i - 1);
    profiles(i).landing_pad_center = [pad_xy_i(1), pad_xy_i(2), landing_pad_center(3)];
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

function [drone_spawn_xy, pad_spawn_xy, grid_dims] = autlBuildFormationLayout(num_items, center_xy, spacing_m, layout_mode)
% Build a centered formation layout for drones and landing pads.

if nargin < 1 || isempty(num_items)
    num_items = 1;
end
if nargin < 2 || isempty(center_xy)
    center_xy = [0.0, 0.0];
end
if nargin < 3 || isempty(spacing_m)
    spacing_m = 3.0;
end
if nargin < 4 || isempty(layout_mode)
    layout_mode = 'auto';
end

[grid_cols, grid_rows] = autlComputeFormationGridShape(num_items, layout_mode);
grid_dims = [grid_cols, grid_rows];

x_offsets = ((0:grid_cols-1) - (grid_cols - 1) * 0.5) * spacing_m;
y_offsets = ((0:grid_rows-1) - (grid_rows - 1) * 0.5) * spacing_m;

drone_spawn_xy = zeros(num_items, 2);
idx = 0;
for row = 1:grid_rows
    for col = 1:grid_cols
        idx = idx + 1;
        if idx > num_items
            break;
        end
        drone_spawn_xy(idx, :) = [center_xy(1) + x_offsets(col), center_xy(2) + y_offsets(row)];
    end
end

pad_spawn_xy = zeros(num_items, 2);
pad_offset_m = max(0.6, min(1.2, 0.35 * spacing_m));
for idx = 1:num_items
    direction_xy = center_xy - drone_spawn_xy(idx, :);
    direction_norm = norm(direction_xy);
    if direction_norm < 1.0e-6
        direction_unit = [1.0, 0.0];
    else
        direction_unit = direction_xy / direction_norm;
    end
    pad_spawn_xy(idx, :) = drone_spawn_xy(idx, :) + pad_offset_m * direction_unit;
end
end

function [grid_cols, grid_rows] = autlComputeFormationGridShape(num_items, layout_mode)
% Choose a centered grid that is square-ish by default and rectangular when requested.

if nargin < 1 || isempty(num_items)
    num_items = 1;
end
if nargin < 2 || isempty(layout_mode)
    layout_mode = 'auto';
end

layout_mode = lower(string(layout_mode));
switch layout_mode
    case 'rectangle'
        grid_cols = max(1, ceil(sqrt(num_items * 1.5)));
        grid_rows = ceil(num_items / grid_cols);
    otherwise
        grid_cols = max(1, ceil(sqrt(num_items)));
        grid_rows = ceil(num_items / grid_cols);
end
end

function rotated_xy = autlRotatePointsAroundCenter(points_xy, center_xy, rotation_deg)
% Rotate a set of XY points around a shared center.

if isempty(points_xy)
    rotated_xy = points_xy;
    return;
end

rotation_rad = deg2rad(rotation_deg);
rotation_mat = [cos(rotation_rad), -sin(rotation_rad); sin(rotation_rad), cos(rotation_rad)];
center_xy_row = double(center_xy(:).');
shifted_xy = double(points_xy) - center_xy_row;
rotated_xy = shifted_xy * rotation_mat.' + center_xy_row;
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

stop_cmd = sprintf('bash -lc ''bash "%s" --stop >/tmp/autolanding_rviz_stop.log 2>&1 || true''', launch_script);
system(stop_cmd);

cmd = sprintf('bash -lc ''nohup bash "%s" --domain "%s" --workers %d > /tmp/autolanding_rviz.log 2>&1 &''', ...
    launch_script, char(string(ros_domain_id)), num_workers);
[rc, ~] = system(cmd);
if rc == 0
    fprintf('[Pipeline] RViz monitor launch requested (log: /tmp/autolanding_rviz.log)\n');
else
    fprintf('[Pipeline] WARNING: Failed to launch RViz monitor (rc=%d)\n', rc);
end
end

function autlLaunchMavrosBridge(rootDir, ros_domain_id, num_workers, namespace_prefix)
% Launch worker-specific MAVROS nodes in the background.

launch_script = fullfile(rootDir, 'scripts', 'launch_multi_mavros.sh');
if ~isfile(launch_script)
    fprintf('[Pipeline] WARNING: MAVROS launcher not found: %s\n', launch_script);
    return;
end

if nargin < 2 || strlength(string(ros_domain_id)) == 0
    ros_domain_id = 'auto';
end
if nargin < 3 || isempty(num_workers)
    num_workers = 1;
end
if nargin < 4 || strlength(string(namespace_prefix)) == 0
    namespace_prefix = '/mavros_w';
end

cmd = sprintf('bash -lc ''nohup bash "%s" --domain "%s" --workers %d --namespace-prefix "%s" > /tmp/autolanding_mavros_launcher.log 2>&1 &''', ...
    launch_script, char(string(ros_domain_id)), num_workers, char(string(namespace_prefix)));
[rc, ~] = system(cmd);
if rc == 0
    fprintf('[Pipeline] MAVROS bridge launch requested (log: /tmp/autolanding_mavros_launcher.log)\n');
else
    fprintf('[Pipeline] WARNING: Failed to launch MAVROS bridge (rc=%d)\n', rc);
end
end

function domain_id = autlResolveRosDomainId(requested_domain_id)
% Resolve ROS_DOMAIN_ID so MATLAB workers and RViz/bridge use the same value.

if nargin < 1 || strlength(string(requested_domain_id)) == 0
    requested_domain_id = 'auto';
end

requested = char(string(requested_domain_id));
if ~strcmpi(requested, 'auto')
    domain_id = requested;
    return;
end

existing = getenv('ROS_DOMAIN_ID');
if strlength(string(existing)) > 0
    domain_id = char(string(existing));
else
    domain_id = '0';
end
end

function autlAssertDroneIncludeCount(world_path, expected_workers)
% Fail fast if generated world does not contain expected number of drone includes.

if nargin < 2
    return;
end
if ~isfile(world_path)
    return;
end

world_text = fileread(world_path);
match_tokens = regexp(world_text, 'model://iris_with_gimbal_w\d+', 'match');
actual_count = numel(unique(match_tokens));
if actual_count ~= expected_workers
    error('Generated world mismatch: expected %d drone includes, found %d in %s', ...
        expected_workers, actual_count, world_path);
end
end

function [all_ready, report_text] = autlVerifyMavlinkHeartbeatReady(num_workers, multi_drone_enabled, deadline_s, poll_s)
% Verify heartbeat availability on expected MAVLink endpoints before data collection.

if nargin < 1 || isempty(num_workers)
    num_workers = 1;
end
if nargin < 2
    multi_drone_enabled = false;
end
if nargin < 3 || isempty(deadline_s)
    deadline_s = 35.0;
end
if nargin < 4 || isempty(poll_s)
    poll_s = 1.5;
end

if multi_drone_enabled
    target_workers = 1:max(1, round(num_workers));
else
    target_workers = 1;
end

all_ready = true;
line_buf = strings(0, 1);
line_buf(end+1) = "[Pipeline] MAVLink readiness check (heartbeat)...";
ready_flags = false(size(target_workers));
tcp_open_flags = false(size(target_workers));
last_errors = strings(size(target_workers));
progress_last_t = -inf;
progress_interval_s = 5.0;
probe_timeout_s = 0.8;
tcp_fallback_grace_s = 12.0;

start_t = tic;
while toc(start_t) < deadline_s && ~all(ready_flags)
    drawnow limitrate;
    for idx = 1:numel(target_workers)
        if ready_flags(idx)
            continue;
        end

        worker_id = target_workers(idx);
        serial0_conn = sprintf('tcp:127.0.0.1:%d', 5760 + 10 * (worker_id - 1));
        serial1_conn = sprintf('tcp:127.0.0.1:%d', 5762 + 10 * (worker_id - 1));

        cfg = struct('mav', struct('master_connection', serial0_conn, 'allow_port_fallback', false, 'timeout_s', probe_timeout_s));
        res = autlMavproxyControl('status', struct(), cfg);
        if res.is_success
            ready_flags(idx) = true;
            line_buf(end+1) = sprintf('[Pipeline]   Worker %d endpoint READY (serial0=%s, serial1=%s)', ...
                worker_id, serial0_conn, serial1_conn);
            continue;
        end

        fb_cfg = struct('mav', struct('master_connection', serial1_conn, 'allow_port_fallback', false, 'timeout_s', probe_timeout_s));
        fb_res = autlMavproxyControl('status', struct(), fb_cfg);
        if fb_res.is_success
            ready_flags(idx) = true;
            line_buf(end+1) = sprintf('[Pipeline]   Worker %d endpoint READY (serial0=%s, serial1=%s)', ...
                worker_id, serial0_conn, serial1_conn);
        else
            last_err = string(res.error_message);
            if strlength(string(fb_res.error_message)) > 0
                last_err = last_err + " | fallback:" + string(fb_res.error_message);
            end
            last_errors(idx) = last_err;
        end

        drawnow limitrate;
    end

    elapsed_s = toc(start_t);

    % Early fallback: if heartbeat is still delayed but all worker MAVLink TCP ports are open,
    % proceed after a short grace period instead of waiting for full timeout budget.
    if elapsed_s >= tcp_fallback_grace_s && ~all(ready_flags)
        open_now = true;
        for k = 1:numel(target_workers)
            if ready_flags(k)
                continue;
            end
            worker_id = target_workers(k);
            serial0_port = 5760 + 10 * (worker_id - 1);
            serial1_port = 5762 + 10 * (worker_id - 1);
            if ~(autlCanOpenLocalTcp(serial0_port, 0.25) || autlCanOpenLocalTcp(serial1_port, 0.25))
                open_now = false;
                break;
            end
        end

        if open_now
            line_buf(end+1) = sprintf('[Pipeline]   Heartbeat not yet observed by %.1fs, but all MAVLink TCP endpoints are reachable.', elapsed_s);
            line_buf(end+1) = "[Pipeline] WARNING: Using early TCP fallback to continue startup.";
            line_buf(end+1) = "[Pipeline] MAVLink readiness check: PASS (TCP fallback)";
            report_text = sprintf('%s\n', line_buf{:});
            all_ready = true;
            return;
        end
    end

    if (elapsed_s - progress_last_t) >= progress_interval_s
        progress_last_t = elapsed_s;
        status_parts = strings(1, numel(target_workers));
        for k = 1:numel(target_workers)
            worker_id = target_workers(k);
            if ready_flags(k)
                status_parts(k) = sprintf('W%d=READY', worker_id);
            else
                status_parts(k) = sprintf('W%d=WAIT', worker_id);
            end
        end
        fprintf('[Pipeline] MAVLink wait %.1fs/%.1fs [%s]\n', elapsed_s, deadline_s, strjoin(status_parts, ', '));
    end

    if ~all(ready_flags)
        pause(poll_s);
    end
end

for idx = 1:numel(target_workers)
    if ~ready_flags(idx)
        all_ready = false;
        worker_id = target_workers(idx);
        serial0_port = 5760 + 10 * (worker_id - 1);
        serial1_port = 5762 + 10 * (worker_id - 1);
        serial0_conn = sprintf('tcp:127.0.0.1:%d', serial0_port);
        serial1_conn = sprintf('tcp:127.0.0.1:%d', serial1_port);

        serial0_open = autlCanOpenLocalTcp(serial0_port, 0.35);
        serial1_open = autlCanOpenLocalTcp(serial1_port, 0.35);
        tcp_open_flags(idx) = serial0_open || serial1_open;

        line_buf(end+1) = sprintf('[Pipeline]   Worker %d endpoint NOT READY after %.1fs', worker_id, deadline_s);
        if strlength(last_errors(idx)) > 0
            line_buf(end+1) = sprintf('[Pipeline]     Last error: %s', char(last_errors(idx)));
        end
        if tcp_open_flags(idx)
            line_buf(end+1) = sprintf('[Pipeline]     TCP probe: OPEN (serial0=%s, serial1=%s)', serial0_conn, serial1_conn);
        else
            line_buf(end+1) = sprintf('[Pipeline]     TCP probe: CLOSED (serial0=%s, serial1=%s)', serial0_conn, serial1_conn);
        end
    end
end

if all_ready
    line_buf(end+1) = "[Pipeline] MAVLink readiness check: PASS";
elseif all(tcp_open_flags)
    % Some environments delay/omit heartbeat at startup while endpoints are already reachable.
    % Allow pipeline to continue; worker-level control still performs per-call readiness checks.
    all_ready = true;
    line_buf(end+1) = "[Pipeline] WARNING: Heartbeat not observed, but MAVLink TCP endpoints are open for all workers.";
    line_buf(end+1) = "[Pipeline] MAVLink readiness check: PASS (TCP fallback)";
else
    line_buf(end+1) = "[Pipeline] MAVLink readiness check: FAIL";
end

report_text = sprintf('%s\n', line_buf{:});
end

function is_open = autlCanOpenLocalTcp(port, timeout_s)
% Return true when a local TCP endpoint accepts a connection.

if nargin < 2 || isempty(timeout_s)
    timeout_s = 0.35;
end

cmd = sprintf([ ...
    'bash -lc ''timeout %.2f bash -lc "exec 3<>/dev/tcp/127.0.0.1/%d; ' ...
    'exec 3<&-; exec 3>&-" >/dev/null 2>&1'''], max(0.1, double(timeout_s)), round(double(port)));
is_open = (system(cmd) == 0);
end

function [all_ready, report_text] = autlWaitForSitlJsonReady(num_workers, multi_drone_enabled, timeout_s)
% Wait until each required SITL instance reports JSON sensor data reception.

if nargin < 1 || isempty(num_workers)
    num_workers = 1;
end
if nargin < 2
    multi_drone_enabled = false;
end
if nargin < 3 || isempty(timeout_s)
    timeout_s = 30.0;
end

if multi_drone_enabled
    target_instances = 0:(max(1, round(num_workers)) - 1);
else
    target_instances = 0;
end

poll_s = 1.0;
start_t = tic;
line_buf = strings(0, 1);
line_buf(end+1) = "[Pipeline] SITL JSON bridge readiness check...";
ready_flags = false(size(target_instances));
bootstrap_last_t = -inf(size(target_instances));
bootstrap_retry_s = 2.0;
progress_last_t = -inf;
progress_interval_s = 5.0;

while toc(start_t) < timeout_s
    drawnow limitrate;
    for idx = 1:numel(target_instances)
        if ready_flags(idx)
            continue;
        end
        instance_id = target_instances(idx);
        log_file = sprintf('/tmp/autolanding_sitl/arducopter_I%d.log', instance_id);

        elapsed_s = toc(start_t);
        if (elapsed_s - bootstrap_last_t(idx)) >= bootstrap_retry_s
            autlNudgeSitlSerial0(instance_id);
            bootstrap_last_t(idx) = elapsed_s;
        end

        if ~isfile(log_file)
            continue;
        end
        try
            txt = fileread(log_file);
            ready_flags(idx) = contains(txt, 'JSON received:');
        catch
            % Ignore transient file access errors and retry.
        end

        drawnow limitrate;
    end

    elapsed_s = toc(start_t);
    if (elapsed_s - progress_last_t) >= progress_interval_s
        progress_last_t = elapsed_s;
        status_parts = strings(1, numel(target_instances));
        for k = 1:numel(target_instances)
            instance_id = target_instances(k);
            if ready_flags(k)
                status_parts(k) = sprintf('I%d=READY', instance_id);
            else
                status_parts(k) = sprintf('I%d=WAIT', instance_id);
            end
        end
        fprintf('[Pipeline] SITL JSON wait %.1fs/%.1fs [%s]\n', elapsed_s, timeout_s, strjoin(status_parts, ', '));
    end

    if all(ready_flags)
        break;
    end
    pause(poll_s);
end

all_ready = all(ready_flags);
for idx = 1:numel(target_instances)
    instance_id = target_instances(idx);
    log_file = sprintf('/tmp/autolanding_sitl/arducopter_I%d.log', instance_id);
    if ready_flags(idx)
        line_buf(end+1) = sprintf('[Pipeline]   I%d JSON READY (%s)', instance_id, log_file);
    else
        line_buf(end+1) = sprintf('[Pipeline]   I%d JSON NOT READY after %.1fs (%s)', ...
            instance_id, timeout_s, log_file);
    end
end

if all_ready
    line_buf(end+1) = "[Pipeline] SITL JSON bridge readiness check: PASS";
else
    line_buf(end+1) = "[Pipeline] SITL JSON bridge readiness check: FAIL";
end

report_text = sprintf('%s\n', line_buf{:});
end

function autlNudgeSitlSerial0(instance_id)
% Open and close SERIAL0 TCP quickly so SITL exits "Waiting for connection ....".

if nargin < 1 || isempty(instance_id)
    return;
end

serial0_port = 5760 + 10 * instance_id;
cmd = sprintf([ ...
    'bash -lc ''timeout 0.4 bash -lc "exec 3<>/dev/tcp/127.0.0.1/%d; ' ...
    'sleep 0.05; exec 3<&-; exec 3>&-" >/dev/null 2>&1 || true'''], serial0_port);
system(cmd);
end

function autlMainCleanup()
% Cleanup function called on any exit (Ctrl+C or normal termination)

try
    fprintf('\n[Pipeline.cleanup] Cleaning up resources...\n');
    
    % Kill background processes to prevent port conflicts
    system('pkill -f "gz sim" 2>/dev/null');
    system('pkill -f "arducopter" 2>/dev/null');
    system('pkill -f "mavproxy.py" 2>/dev/null');
    system('pkill -f "ros_gz_bridge parameter_bridge" 2>/dev/null');
    system('pkill -f "ros2_aruco" 2>/dev/null');
    system('pkill -f "publish_multi_drone_odom.py" 2>/dev/null');
    system('pkill -f "mavros_node" 2>/dev/null');
    
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
