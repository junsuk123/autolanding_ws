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
    num_workers = 1;                    % Number of parallel workers (use 1 to avoid MAVProxy contention)
    scenarios_per_worker = 2;           % Scenarios per worker (total = num_workers * scenarios_per_worker)
    enable_auto_motion = true;          % Send MAVLink velocity setpoints during collection
    takeoff_height_m = 3.0;             % Takeoff altitude used before motion commands

    % ArUco landing pad configuration
    use_aruco_landing_pad = true;
    aruco_marker_id = 23;
    drone_body_size_m = 0.1;            % Reference body size for marker scaling (0.1 * 2^2 = 0.4 m)
    marker_size_multiplier = 2^2;       % Requested: 2 squared times drone size
    marker_size_m = drone_body_size_m * marker_size_multiplier;
    landing_pad_center = [0.0, 0.0, 0.0];   % Fixed pad position [x, y, z]
    landing_pad_size = [marker_size_m, marker_size_m];
    landing_pad_topic = '/autolanding/landing_pad';
    spawn_radius_m = 0.8;               % Must remain within 1 m of marker center
    spawn_angle_deg = 35.0;             % Fixed angle to keep start deterministic

    % Train/Validation Split
    total_training_samples = 4;         % Total samples for training (including both train+val)
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
    fprintf('    - Landing Pad Topic: %s\n', landing_pad_topic);
    fprintf('    - ArUco Enabled: %s (id=%d)\n', char(string(use_aruco_landing_pad)), aruco_marker_id);
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
        if norm(spawn_xy - landing_pad_center(1:2)) > 1.0
            error('Spawn position exceeds 1 m radius from marker center.');
        end

        world_path_to_launch = base_world_path;
        if use_aruco_landing_pad
            generated_world_path = fullfile('/tmp', 'iris_runway_aruco_landing.sdf');
            autlCreateArucoLandingWorld(base_world_path, generated_world_path, aruco_pkg_dir, ...
                landing_pad_center, marker_size_m, spawn_xy, aruco_marker_id);
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
        
        % Start ArduPilot SITL
        fprintf('[Pipeline] Starting ArduPilot SITL...\n');
        ap_cmd = sprintf('nohup $HOME/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 --defaults $HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I0 > /tmp/ardupilot_sitl.log 2>&1 &');
        system(ap_cmd);
        pause(5);
        
        % Extra wait time to ensure processes started
        fprintf('[Pipeline] Waiting for ArduPilot initialization...\n');
        pause(10);
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

        %% STEP 7: Generate Hover-to-Land Trajectory (ArUco pad target)
        fprintf('\n[Pipeline] STEP 6: Generate Hover-to-Land Trajectory\n');
        fprintf('════════════════════════════════════════════\n');
        cfg_traj = autlDefaultConfig();
        hover_initial = struct('x', spawn_xy(1), 'y', spawn_xy(2), 'z', takeoff_height_m);
        pad_target = struct('x', landing_pad_center(1), 'y', landing_pad_center(2), 'z', landing_pad_center(3));
        fused_stub = struct('fused_confidence', 0.90);
        aruco_traj = autlGenerateTrajectory(hover_initial, pad_target, fused_stub, cfg_traj);
        aruco_traj_csv = fullfile(rootDir, 'data', 'processed', 'landing_trajectory_aruco_hover.csv');
        aruco_traj_json = fullfile(rootDir, 'data', 'processed', 'landing_trajectory_aruco_hover.json');
        autlWriteTrajectoryCsv(aruco_traj_csv, aruco_traj);
        autlSaveJson(aruco_traj_json, struct('marker_id', aruco_marker_id, 'spawn_xy', spawn_xy, ...
            'hover_alt_m', takeoff_height_m, 'pad_center', landing_pad_center, 'pad_size', landing_pad_size, ...
            'trajectory', table2struct(aruco_traj)));
        fprintf('[Pipeline] ArUco landing trajectory saved: %s\n', aruco_traj_csv);
        
        %% FINAL SUMMARY
        fprintf('\n');
        fprintf('═════════════════════════════════════════════════════════════\n');
        fprintf('  Pipeline Complete!\n');
        fprintf('═════════════════════════════════════════════════════════════\n');
        fprintf('\nOUTPUT FILES:\n');
        fprintf('  - Raw Data: %s\n', collection_dir);
        fprintf('  - ArUco Trajectory: %s\n', aruco_traj_csv);
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

function autlCreateArucoLandingWorld(base_world_path, output_world_path, aruco_pkg_dir, marker_center_xyz, marker_size_m, spawn_xy, marker_id)
% Generate a temporary world using ros2-gazebo-aruco marker model and fixed drone spawn.

if ~isfile(base_world_path)
    error('Base world not found: %s', base_world_path);
end
if ~isfolder(fullfile(aruco_pkg_dir, 'gz-world', 'aruco_box'))
    error('ArUco model directory not found: %s', fullfile(aruco_pkg_dir, 'gz-world', 'aruco_box'));
end

world_text = fileread(base_world_path);
drone_pose = sprintf('%.3f %.3f 0.195 0 0 90', spawn_xy(1), spawn_xy(2));
world_text = regexprep(world_text, ...
    '<uri>model://iris_with_gimbal</uri>\s*<pose degrees="true">[^<]*</pose>', ...
    sprintf('<uri>model://iris_with_gimbal</uri>\n      <pose degrees="true">%s</pose>', drone_pose), ...
    'once');

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

camera_model = sprintf([ ...
    '\n    <model name="aruco_observer_camera">\n' ...
    '      <static>true</static>\n' ...
    '      <pose>%.3f %.3f %.3f 0 0 %.3f</pose>\n' ...
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
    marker_center_xyz(1) + 4.0, marker_center_xyz(2), marker_center_xyz(3) + 1.6, pi);

world_text = strrep(world_text, '</world>', [aruco_model camera_model '  </world>']);

fid = fopen(output_world_path, 'w');
if fid < 0
    error('Failed to write generated world: %s', output_world_path);
end
fprintf(fid, '%s', world_text);
fclose(fid);
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
