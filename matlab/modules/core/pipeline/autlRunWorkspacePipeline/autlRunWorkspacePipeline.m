function summary = autlRunWorkspacePipeline(rootDir, varargin)
% autlRunWorkspacePipeline
% MATLAB orchestration for pipeline, collection, training, validation, and plotting.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end

ros_domain_id_value = strtrim(getenv('ROS_DOMAIN_ID'));
if isempty(ros_domain_id_value) || isempty(regexp(ros_domain_id_value, '^[0-9]+$', 'once'))
    setenv('ROS_DOMAIN_ID', '0');
end
localAddWorkspacePaths(rootDir);

cfg = autlLoadOrchestrationConfig(fullfile(rootDir, 'ai', 'configs', 'orchestration_config.yaml'));
runtime = localBuildRuntime(rootDir, cfg, varargin{:});
cfg = localApplyCfgOverrides(cfg, runtime);

summary = struct();
summary.mode = runtime.mode;
summary.root_dir = rootDir;
summary.run_id = runtime.run_id;
summary.config = cfg;
summary.stages = struct();

semanticInputPath = localResolvePath(rootDir, localGetValue(cfg, 'semantic_input', fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json')));
collectionRoot = localResolvePath(rootDir, localGetValue(cfg, 'collection_root', fullfile(rootDir, 'data', 'collected')));
plotsDir = localResolvePath(rootDir, localGetValue(cfg, 'plots_dir', fullfile(rootDir, 'data', 'plots', 'paper')));
modelRoot = localResolvePath(rootDir, localGetValue(cfg, 'model_root', fullfile(rootDir, 'data', 'models')));
trainRatio = localGetDouble(cfg, 'train_ratio', 0.8);
if ~(isfinite(trainRatio) && trainRatio > 0 && trainRatio < 1)
    trainRatio = 0.8;
end
valRatio = localGetDouble(cfg, 'val_ratio', 1.0 - trainRatio);
if ~(isfinite(valRatio) && valRatio >= 0)
    valRatio = 1.0 - trainRatio;
end

workers = max(1, localGetInt(cfg, 'workers', 1));
scenariosPerWorker = max(1, localGetInt(cfg, 'scenarios_per_worker', 10));
if isfield(runtime, 'workers_override') && isfinite(runtime.workers_override)
    workers = max(1, round(runtime.workers_override));
end
if isfield(runtime, 'scenarios_override') && isfinite(runtime.scenarios_override)
    scenariosPerWorker = max(1, round(runtime.scenarios_override));
end
totalTargetScenarios = max(2, workers * scenariosPerWorker);
trainScenarios = max(1, round(totalTargetScenarios * trainRatio));
valScenarios = max(1, totalTargetScenarios - trainScenarios);

summary.paths = struct( ...
    'semantic_input', semanticInputPath, ...
    'collection_root', collectionRoot, ...
    'plots_dir', plotsDir, ...
    'model_root', modelRoot);
summary.split = struct('train_ratio', trainRatio, 'val_ratio', valRatio, 'train_scenarios', trainScenarios, 'val_scenarios', valScenarios);

if any(strcmp(runtime.mode, {'collect', 'collect_parallel', 'full'}))
    localPreflightRosComms(cfg);
end

switch runtime.mode
    case 'pipeline'
        pipelineSummary = autlRunPipeline(rootDir, semanticInputPath);
        summary.stages.pipeline = localCompactPipelineSummary(pipelineSummary);
        summary.stages.validation = autlRunValidation(rootDir, semanticInputPath);

    case 'validation'
        summary.stages.validation = localRunModelValidation(rootDir, collectionRoot, trainScenarios, valScenarios, cfg, runtime, modelRoot, plotsDir);

    case 'collect'
        collectionConfig = localBuildCollectionConfig(cfg, rootDir, collectionRoot, runtime, 1, 1);
        summary.stages.collection = autlRunDataCollection(rootDir, collectionConfig);

    case 'collect_parallel'
        summary.stages.collection = localRunCollectionBatch(rootDir, collectionRoot, cfg, runtime, workers, scenariosPerWorker);

    case 'mission'
        pipelineSummary = autlRunPipeline(rootDir, semanticInputPath);
        summary.stages.pipeline = localCompactPipelineSummary(pipelineSummary);
        summary.stages.validation = autlRunValidation(rootDir, semanticInputPath);
        summary.stages.mission = localRunMission(rootDir, cfg, runtime, pipelineSummary, false);

    case 'sim'
        pipelineSummary = autlRunPipeline(rootDir, semanticInputPath);
        summary.stages.pipeline = localCompactPipelineSummary(pipelineSummary);
        summary.stages.validation = autlRunValidation(rootDir, semanticInputPath);
        summary.stages.mission = localRunMission(rootDir, cfg, runtime, pipelineSummary, true);

    case 'full'
        pipelineSummary = autlRunPipeline(rootDir, semanticInputPath);
        summary.stages.pipeline = localCompactPipelineSummary(pipelineSummary);
        summary.stages.validation = autlRunValidation(rootDir, semanticInputPath);
        summary.stages.collection = localRunCollectionBatch(rootDir, collectionRoot, cfg, runtime, workers, scenariosPerWorker);
        localAssertCollectionHealthy(summary.stages.collection);
        summary.stages.training = localRunTrainingAndComparison(rootDir, collectionRoot, modelRoot, plotsDir, cfg, runtime, trainScenarios, valScenarios);

    otherwise
        error('[autlRunWorkspacePipeline] Unsupported mode: %s', runtime.mode);
end

summary.output_dir = modelRoot;
summary.plots_dir = plotsDir;

summary_file = fullfile(rootDir, 'data', 'processed', sprintf('workspace_summary_%s.json', runtime.run_id));
autlSaveJson(summary_file, summary);
summary.summary_file = summary_file;

fprintf('[autlRunWorkspacePipeline] mode=%s summary=%s\n', runtime.mode, summary_file);
end

function localAddWorkspacePaths(rootDir)
modDir = fullfile(rootDir, 'matlab', 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end
end

function runtime = localBuildRuntime(rootDir, cfg, varargin)
runtime = struct();
runtime.run_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
runtime.mode = localNormalizeMode(localGetValue(cfg, 'mode', 'full'));
runtime.workers_override = NaN;
runtime.scenarios_override = NaN;
runtime.cfg_overrides = struct();

if nargin >= 3 && ~isempty(varargin)
    if isstruct(varargin{1})
        runtime = localApplyRuntimeOptions(runtime, varargin{1});
    else
        first_arg = string(varargin{1});
        if strlength(first_arg) > 0
            runtime.mode = localNormalizeMode(first_arg);
        end
        if numel(varargin) >= 2
            runtime.workers_override = double(varargin{2});
        end
        if numel(varargin) >= 3
            runtime.scenarios_override = double(varargin{3});
        end
    end
end

if any(strcmp(runtime.mode, {'gui', 'server', 'headless'}))
    requested_mode = runtime.mode;
    runtime.mode = 'sim';
    if ~isfield(runtime, 'gazebo_server_mode')
        runtime.gazebo_server_mode = any(strcmp(requested_mode, {'server', 'headless'}));
    end
else
    if ~isfield(runtime, 'gazebo_server_mode')
        runtime.gazebo_server_mode = logical(localGetValue(cfg, 'headless', false));
        if localGetValue(cfg, 'gui', true)
            runtime.gazebo_server_mode = false;
        end
    end
end

runtime.keep_sim = logical(localGetValue(cfg, 'keep_sim', false));
runtime.verbose = logical(localGetValue(cfg, 'verbose', true));
runtime.no_plots = logical(localGetValue(cfg, 'no_plots', false));
runtime.mode = localNormalizeMode(runtime.mode);
runtime.rootDir = rootDir;
end

function runtime = localApplyRuntimeOptions(runtime, opts)
if ~isstruct(opts)
    return;
end

if localHasRuntimeOption(opts, 'mode') && strlength(string(opts.mode)) > 0
    runtime.mode = localNormalizeMode(string(opts.mode));
end

if localHasRuntimeOption(opts, 'num_drones') && isfinite(double(opts.num_drones))
    runtime.workers_override = max(1, round(double(opts.num_drones)));
    runtime.cfg_overrides.workers = runtime.workers_override;
elseif localHasRuntimeOption(opts, 'workers') && isfinite(double(opts.workers))
    runtime.workers_override = max(1, round(double(opts.workers)));
    runtime.cfg_overrides.workers = runtime.workers_override;
end

if localHasRuntimeOption(opts, 'scenarios_per_worker') && isfinite(double(opts.scenarios_per_worker))
    runtime.scenarios_override = max(1, round(double(opts.scenarios_per_worker)));
    runtime.cfg_overrides.scenarios_per_worker = runtime.scenarios_override;
end

if localHasRuntimeOption(opts, 'num_landing_pads') && isfinite(double(opts.num_landing_pads))
    runtime.cfg_overrides.landing_pad_count = max(1, round(double(opts.num_landing_pads)));
end
if localHasRuntimeOption(opts, 'train_ratio') && isfinite(double(opts.train_ratio))
    runtime.cfg_overrides.train_ratio = double(opts.train_ratio);
end
if localHasRuntimeOption(opts, 'val_ratio') && isfinite(double(opts.val_ratio))
    runtime.cfg_overrides.val_ratio = double(opts.val_ratio);
end
if localHasRuntimeOption(opts, 'headless')
    runtime.cfg_overrides.headless = logical(opts.headless);
    runtime.cfg_overrides.gui = ~logical(opts.headless);
    runtime.gazebo_server_mode = logical(opts.headless);
end
if localHasRuntimeOption(opts, 'enable_visualization')
    runtime.cfg_overrides.enable_visualization = logical(opts.enable_visualization);
end
if localHasRuntimeOption(opts, 'auto_flight_demo')
    runtime.cfg_overrides.auto_flight_demo = logical(opts.auto_flight_demo);
end
if localHasRuntimeOption(opts, 'drone_spawn_above_pad_m') && isfinite(double(opts.drone_spawn_above_pad_m))
    runtime.cfg_overrides.drone_spawn_above_pad_m = double(opts.drone_spawn_above_pad_m);
end
if localHasRuntimeOption(opts, 'landing_pad_size_xy') && isfinite(double(opts.landing_pad_size_xy))
    runtime.cfg_overrides.landing_pad_size_xy = double(opts.landing_pad_size_xy);
end
if localHasRuntimeOption(opts, 'demo_takeoff_alt_m') && isfinite(double(opts.demo_takeoff_alt_m))
    runtime.cfg_overrides.demo_takeoff_alt_m = double(opts.demo_takeoff_alt_m);
end
if localHasRuntimeOption(opts, 'reset_spawn_z_m') && isfinite(double(opts.reset_spawn_z_m))
    runtime.cfg_overrides.reset_spawn_z_m = double(opts.reset_spawn_z_m);
end
if localHasRuntimeOption(opts, 'control_backend') && strlength(string(opts.control_backend)) > 0
    runtime.cfg_overrides.control_backend = lower(char(string(opts.control_backend)));
end
if localHasRuntimeOption(opts, 'mavlink_precheck_timeout_s') && isfinite(double(opts.mavlink_precheck_timeout_s))
    runtime.cfg_overrides.mavlink_precheck_timeout_s = double(opts.mavlink_precheck_timeout_s);
end
if localHasRuntimeOption(opts, 'mavlink_control_timeout_s') && isfinite(double(opts.mavlink_control_timeout_s))
    runtime.cfg_overrides.mavlink_control_timeout_s = double(opts.mavlink_control_timeout_s);
end
end

function tf = localHasRuntimeOption(opts, field_name)
if ~isstruct(opts)
    tf = false;
    return;
end
if ~isfield(opts, field_name)
    tf = false;
    return;
end
if isfield(opts, 'explicit_fields__') && iscell(opts.explicit_fields__)
    tf = any(strcmp(opts.explicit_fields__, field_name));
else
    tf = true;
end
end

function cfg = localApplyCfgOverrides(cfg, runtime)
if ~isstruct(runtime) || ~isfield(runtime, 'cfg_overrides') || ~isstruct(runtime.cfg_overrides)
    return;
end
names = fieldnames(runtime.cfg_overrides);
for i = 1:numel(names)
    cfg.(names{i}) = runtime.cfg_overrides.(names{i});
end
end

function mode = localNormalizeMode(value)
mode = lower(strtrim(char(string(value))));
switch mode
    case {'', 'default'}
        mode = 'full';
    case {'gui', 'server', 'headless'}
        % keep aliases for callers that pass simulation style flags
    case {'pipeline', 'validation', 'collect', 'collect_parallel', 'mission', 'sim', 'full'}
        % supported values
    otherwise
        error('[autlRunWorkspacePipeline] Unsupported mode: %s', mode);
end
end

function value = localGetValue(cfg, field_name, default_value)
if isstruct(cfg) && isfield(cfg, field_name)
    value = cfg.(field_name);
    if isempty(value)
        value = default_value;
    end
else
    value = default_value;
end
end

function value = localGetInt(cfg, field_name, default_value)
raw = localGetValue(cfg, field_name, default_value);
try
    value = round(double(raw));
catch
    value = default_value;
end
if ~isfinite(value)
    value = default_value;
end
end

function value = localGetDouble(cfg, field_name, default_value)
raw = localGetValue(cfg, field_name, default_value);
try
    value = double(raw);
catch
    value = default_value;
end
if ~isfinite(value)
    value = default_value;
end
end

function path_value = localResolvePath(rootDir, path_value)
path_value = char(string(path_value));
if isempty(path_value)
    return;
end
if ~isfolder(fileparts(path_value)) && ~isfile(path_value) && ~startsWith(path_value, filesep)
    path_value = fullfile(rootDir, path_value);
end
end

function collectionConfig = localBuildCollectionConfig(cfg, rootDir, collectionRoot, runtime, workerIndex, workerCount)
collectionConfig = struct();
padCenterX = localGetDouble(cfg, 'landing_pad_center_x', 0.0);
padCenterY = localGetDouble(cfg, 'landing_pad_center_y', 0.0);
padCenterZ = localGetDouble(cfg, 'landing_pad_center_z', 0.375);  % Match world generation (0.5m base marker, scaled 0.75m effective, center at 0.375)
arucoBoxHeightM = max(0.05, localGetDouble(cfg, 'aruco_box_height_m', 0.75));  % Match 1.5x scaled model
spawnAbovePadM = max(0.35, localGetDouble(cfg, 'drone_spawn_above_pad_m', 0.5));  % Match world generation
padSizeXY = max(0.3, localGetDouble(cfg, 'landing_pad_size_xy', 1.2));
collectionConfig.max_duration = localGetDouble(cfg, 'duration', 120);
collectionConfig.sample_rate = localGetDouble(cfg, 'sample_rate', 50);
collectionConfig.output_dir = fullfile(collectionRoot, runtime.run_id, sprintf('worker_%d', workerIndex));
collectionConfig.session_id = sprintf('session_%s', runtime.run_id);
collectionConfig.gazebo_server_mode = runtime.gazebo_server_mode;
collectionConfig.enable_visualization = logical(localGetValue(cfg, 'enable_visualization', true));
collectionConfig.close_visualization_on_finish = true;
collectionConfig.force_close_stale_visualizations = true;
collectionConfig.enable_auto_motion = logical(localGetValue(cfg, 'auto_flight_demo', true));
collectionConfig.takeoff_height_m = localGetDouble(cfg, 'demo_takeoff_alt_m', 3.0);
collectionConfig.mavlink_master = localGetValue(cfg, 'mavlink_master', 'udpin:127.0.0.1:14550');
collectionConfig.mavlink_master_fallback = localGetValue(cfg, 'mavlink_master_fallback', '');
collectionConfig.mavlink_precheck_timeout_s = localGetDouble(cfg, 'mavlink_precheck_timeout_s', 90.0);
collectionConfig.mavlink_control_timeout_s = localGetDouble(cfg, 'mavlink_control_timeout_s', 20.0);
collectionConfig.control_backend = lower(char(string(localGetValue(cfg, 'control_backend', 'mavproxy'))));
collectionConfig.control_backend_fallback = logical(localGetValue(cfg, 'control_backend_fallback', false));
collectionConfig.mavros_namespace = localGetValue(cfg, 'mavros_namespace', '/mavros');
collectionConfig.aruco_markers_topic = localGetValue(cfg, 'aruco_markers_topic', '/aruco_markers');
collectionConfig.landing_pad_center = [padCenterX, padCenterY, padCenterZ];
collectionConfig.landing_pad_size = [padSizeXY, padSizeXY];
collectionConfig.reset_spawn_xy = [padCenterX, padCenterY];
collectionConfig.aruco_box_height_m = arucoBoxHeightM;
collectionConfig.drone_spawn_above_pad_m = spawnAbovePadM;
collectionConfig.enforce_spawn_above_pad = true;
collectionConfig.reset_spawn_z_m = localGetDouble(cfg, 'reset_spawn_z_m', padCenterZ + 0.5 * arucoBoxHeightM + spawnAbovePadM);
collectionConfig.log_prefix = sprintf('[Worker %d/%d] ', workerIndex, workerCount);
collectionConfig.worker_state_tag = sprintf('worker_%d', workerIndex);
collectionConfig.mission_overrides = struct( ...
    'mavlink_precheck_timeout_s', collectionConfig.mavlink_precheck_timeout_s, ...
    'mavlink_control_timeout_s', localGetDouble(cfg, 'mavlink_control_timeout_s', 20.0));
collectionConfig.flow_log_dir = fullfile(rootDir, 'data', 'logs', 'parallel', runtime.run_id);
collectionConfig.reset_pose_each_scenario = true;
collectionConfig.reset_ardupilot_each_scenario = true;
collectionConfig.require_mavlink_for_auto_motion = true;
end

function collection_summary = localRunCollectionBatch(rootDir, collectionRoot, cfg, runtime, workerCount, scenariosPerWorker)
collection_summary = struct();
collection_summary.worker_results = cell(workerCount, 1);
collection_summary.worker_count = workerCount;
collection_summary.scenarios_per_worker = scenariosPerWorker;
for workerIndex = 1:workerCount
    workerConfig = localBuildCollectionConfig(cfg, rootDir, collectionRoot, runtime, workerIndex, workerCount);
    workerConfig.scenario_duration = localGetDouble(cfg, 'duration', 120);
    workerConfig.scenarios_per_worker = scenariosPerWorker;
    collection_summary.worker_results{workerIndex} = autlRunDataCollection(rootDir, workerConfig);
end
collection_summary.total_requested = workerCount * scenariosPerWorker;
collection_summary.collection_root = fullfile(collectionRoot, runtime.run_id);
end

function localPreflightRosComms(cfg)
control_backend = lower(char(string(localGetValue(cfg, 'control_backend', 'mavproxy'))));
if ~strcmp(control_backend, 'mavros')
    return;
end

iicc_setup = fullfile(fileparts(fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))))), '..', 'IICC26_ws', 'install', 'setup.bash');
source_overlay = '';
if isfile(iicc_setup)
    source_overlay = sprintf('[ -f "%s" ] && source "%s" >/dev/null 2>&1; ', iicc_setup, iicc_setup);
end

ros2_cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; command -v ros2 >/dev/null 2>&1''', source_overlay);
[ros2_rc, ~] = system(ros2_cmd);
if ros2_rc ~= 0
    error('[autlRunWorkspacePipeline] ROS2 command is unavailable. Source ROS2 and IICC26_ws before MATLAB run.');
end

pkg_cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; ros2 pkg prefix mavros >/dev/null 2>&1 && ros2 pkg prefix mavros_msgs >/dev/null 2>&1''', source_overlay);
[pkg_rc, ~] = system(pkg_cmd);
if pkg_rc ~= 0
    error('[autlRunWorkspacePipeline] MAVROS packages are missing. Run scripts/install_mavros2.sh and build overlays.');
end

if logical(localGetValue(cfg, 'require_custom_ros_interface', false))
    required_interfaces = localGetValue(cfg, 'required_ros_interfaces', {});
    if ischar(required_interfaces) || isstring(required_interfaces)
        required_interfaces = {char(string(required_interfaces))};
    end
    if iscell(required_interfaces)
        for i = 1:numel(required_interfaces)
            iface = strtrim(char(string(required_interfaces{i})));
            if strlength(string(iface)) == 0
                continue;
            end
            iface_cmd = sprintf('bash -lc ''set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; %sset -u; ros2 interface show %s >/dev/null 2>&1''', source_overlay, iface);
            [iface_rc, ~] = system(iface_cmd);
            if iface_rc ~= 0
                error('[autlRunWorkspacePipeline] Missing ROS2 interface: %s (build IICC26_ws and source install/setup.bash).', iface);
            end
        end
    end
end
end

function localAssertCollectionHealthy(collection_summary)
if ~isstruct(collection_summary) || ~isfield(collection_summary, 'worker_results')
    error('[autlRunWorkspacePipeline] Collection summary is invalid.');
end

for i = 1:numel(collection_summary.worker_results)
    wr = collection_summary.worker_results{i};
    if ~isstruct(wr)
        error('[autlRunWorkspacePipeline] Worker %d returned invalid collection result.', i);
    end
    st = lower(strtrim(char(string(localGetField(wr, 'status', 'unknown')))));
    if ~strcmp(st, 'completed')
        msg = char(string(localGetField(wr, 'error_message', 'no error message')));
        error('[autlRunWorkspacePipeline] Collection failed on worker %d (status=%s): %s', i, st, msg);
    end
    sample_count = double(localGetField(wr, 'sample_count', 0));
    if ~(isfinite(sample_count) && sample_count > 0)
        error('[autlRunWorkspacePipeline] Collection produced no samples on worker %d.', i);
    end
end
end

function value = localGetField(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
else
    value = default_value;
end
end

function training_summary = localRunTrainingAndComparison(rootDir, collectionRoot, modelRoot, plotsDir, cfg, runtime, trainScenarios, valScenarios)
collectionRunDir = fullfile(collectionRoot, runtime.run_id);
[X_train, y_train, X_val, y_val, data_summary] = autlLoadAndPrepareTrainingData(rootDir, collectionRunDir, trainScenarios, valScenarios);

modelHybrid = autlTrainHybridModel(X_train, y_train, X_val, y_val, rootDir);
modelPure = autlTrainPureAiModel(X_train, y_train, X_val, y_val, rootDir);
comparison = autlCompareModels(modelHybrid, modelPure, X_val, y_val, true, true);

if ~logical(localGetValue(cfg, 'no_plots', false))
    autlGenerateComparisonPlots(modelHybrid, modelPure, X_val, y_val, comparison, rootDir, true, true);
    localCopyComparisonPlots(rootDir, plotsDir);
end

runModelDir = fullfile(modelRoot, sprintf('%s_scenarios_%03d', runtime.run_id, data_summary.num_train + data_summary.num_val));
if ~exist(runModelDir, 'dir')
    mkdir(runModelDir);
end

autlCopyIfExists(fullfile(rootDir, 'data', 'models', 'model_hybrid_ontology_ai.mat'), fullfile(runModelDir, 'model_hybrid_ontology_ai.mat'));
autlCopyIfExists(fullfile(rootDir, 'data', 'models', 'model_pure_ai.mat'), fullfile(runModelDir, 'model_pure_ai.mat'));

training_summary = struct();
training_summary.data_summary = data_summary;
training_summary.comparison = comparison;
training_summary.model_run_dir = runModelDir;
training_summary.validation_split = struct('train_scenarios', trainScenarios, 'val_scenarios', valScenarios);
training_summary.model_paths = struct( ...
    'hybrid', fullfile(rootDir, 'data', 'models', 'model_hybrid_ontology_ai.mat'), ...
    'pure', fullfile(rootDir, 'data', 'models', 'model_pure_ai.mat'));
training_summary.model_metadata = struct( ...
    'hybrid_type', string(modelHybrid.type), ...
    'pure_type', string(modelPure.type), ...
    'hybrid_acc_val', modelHybrid.acc_val, ...
    'pure_acc_val', modelPure.acc_val);

autlSaveJson(fullfile(runModelDir, 'training_summary.json'), training_summary);
autlSaveJson(fullfile(runModelDir, 'comparison_summary.json'), comparison);
end

function validation_summary = localRunModelValidation(rootDir, collectionRoot, trainScenarios, valScenarios, cfg, runtime, modelRoot, plotsDir)
collectionRunDir = fullfile(collectionRoot, runtime.run_id);
if ~exist(collectionRunDir, 'dir')
    collectionRunDir = collectionRoot;
end

[X_train, y_train, X_val, y_val, data_summary] = autlLoadAndPrepareTrainingData(rootDir, collectionRunDir, trainScenarios, valScenarios);

defaultHybridPath = fullfile(rootDir, 'data', 'models', 'model_hybrid_ontology_ai.mat');
defaultPurePath = fullfile(rootDir, 'data', 'models', 'model_pure_ai.mat');
hybridPath = localResolveModelPath(rootDir, modelRoot, localGetValue(cfg, 'validation_hybrid_model_path', defaultHybridPath));
purePath = localResolveModelPath(rootDir, modelRoot, localGetValue(cfg, 'validation_pure_model_path', defaultPurePath));

if isempty(hybridPath) || ~isfile(hybridPath) || isempty(purePath) || ~isfile(purePath)
    validation_summary = localRunTrainingAndComparison(rootDir, collectionRoot, modelRoot, plotsDir, cfg, runtime, trainScenarios, valScenarios);
    return;
end

hybridModel = load(hybridPath, 'model');
pureModel = load(purePath, 'model');
hybridModel = hybridModel.model;
pureModel = pureModel.model;

comparison = autlCompareModels(hybridModel, pureModel, X_val, y_val, true, true);
if ~logical(localGetValue(cfg, 'no_plots', false))
    autlGenerateComparisonPlots(hybridModel, pureModel, X_val, y_val, comparison, rootDir, true, true);
    localCopyComparisonPlots(rootDir, plotsDir);
end

validation_summary = struct();
validation_summary.data_summary = data_summary;
validation_summary.comparison = comparison;
validation_summary.hybrid_model_path = hybridPath;
validation_summary.pure_model_path = purePath;
validation_summary.selected_model_policy = 'latest-or-configured';
autlSaveJson(fullfile(modelRoot, sprintf('%s_validation_summary.json', runtime.run_id)), validation_summary);
end

function mission_summary = localRunMission(rootDir, cfg, runtime, pipelineSummary, enableGui)
mission_summary = struct();
mission_summary.enable_gui = enableGui;
mission_summary.pipeline = pipelineSummary;

if ~isfield(pipelineSummary, 'landing_trajectory') || isempty(pipelineSummary.landing_trajectory)
    mission_summary.status = 'no_trajectory';
    return;
end

initial_state = struct('x0', pipelineSummary.initial_state(1), 'y0', pipelineSummary.initial_state(2), 'z0', pipelineSummary.initial_state(3), 'yaw0', 0);
target_state = struct('x_target', pipelineSummary.target_state(1), 'y_target', pipelineSummary.target_state(2), 'z_target', pipelineSummary.target_state(3));

missionCfg = autlDefaultConfig();
missionCfg.mission = struct();
missionCfg.mission.takeoff_height = localGetDouble(cfg, 'demo_takeoff_alt_m', 3.0);
missionCfg.mission.trajectory_tracking_enabled = true;
missionCfg.mission.use_ros_control = false;
missionCfg.mav = struct('allow_port_fallback', true);

try
    mission_result = autlAutonomousMission(pipelineSummary.landing_trajectory, initial_state, target_state, missionCfg, rootDir);
    mission_summary.status = mission_result.mission_status;
    mission_summary.result = mission_result;
    autlSaveJson(fullfile(rootDir, 'data', 'processed', sprintf('mission_result_%s.json', runtime.run_id)), mission_result);
catch ME
    mission_summary.status = 'failed';
    mission_summary.error = ME.message;
end
end

function path_value = localResolveModelPath(rootDir, modelRoot, path_value)
path_value = char(string(path_value));
if isempty(path_value)
    path_value = '';
    return;
end
if ~isfile(path_value)
    candidate = fullfile(rootDir, path_value);
    if isfile(candidate)
        path_value = candidate;
        return;
    end
    candidate = fullfile(modelRoot, path_value);
    if isfile(candidate)
        path_value = candidate;
    end
end
end

function autlCopyIfExists(srcPath, dstPath)
if isfile(srcPath)
    copyfile(srcPath, dstPath);
end
end

function compact = localCompactPipelineSummary(pipelineSummary)
compact = struct();
compact.out_json = pipelineSummary.out_json;
compact.out_csv = pipelineSummary.out_csv;
compact.out_validation_json = pipelineSummary.out_validation_json;
compact.trajectory_points = pipelineSummary.trajectory_points;
compact.semantic_risk = pipelineSummary.semantic_risk;
compact.fused_confidence = pipelineSummary.fused_confidence;
compact.decision = pipelineSummary.decision;
compact.touchdown_error_xy = pipelineSummary.touchdown_error_xy;
end

function localCopyComparisonPlots(rootDir, plotsDir)
sourcePlotDir = fullfile(rootDir, 'data', 'plots');
if ~exist(sourcePlotDir, 'dir')
    return;
end
if ~exist(plotsDir, 'dir')
    mkdir(plotsDir);
end

copyPairs = {
    'model_comparison.fig', 'paper_model_comparison.fig';
    'model_comparison.png', 'paper_model_comparison.png';
};

for i = 1:size(copyPairs, 1)
    srcPath = fullfile(sourcePlotDir, copyPairs{i, 1});
    dstPath = fullfile(plotsDir, copyPairs{i, 2});
    if isfile(srcPath)
        copyfile(srcPath, dstPath);
    end
end
end