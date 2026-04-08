function result = AutoLandingDataParallel(num_workers, scenarios_per_worker)
% AUTOLANDINGDATAPARALLEL
% MATLAB data collection entrypoint.

if nargin < 1 || isempty(num_workers)
    num_workers = 3;
end
if nargin < 2 || isempty(scenarios_per_worker)
    scenarios_per_worker = 2;
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

cfg = autlLoadOrchestrationConfig(fullfile(rootDir, 'ai', 'configs', 'orchestration_config.yaml'));
cfg.workers = num_workers;
cfg.scenarios_per_worker = scenarios_per_worker;
cfg.mode = 'collect_parallel';

result = autlRunWorkspacePipeline(rootDir, 'collect_parallel', num_workers, scenarios_per_worker);
end
