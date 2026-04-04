function result = AutoLandingDataParallel(num_workers, scenarios_per_worker)
% AUTOLANDINGDATAPARALLEL
% MATLAB compatibility entrypoint.
% Parallel collection orchestration is handled by scripts/autolanding_launcher.py.

if nargin < 1 || isempty(num_workers)
    num_workers = 3;
end
if nargin < 2 || isempty(scenarios_per_worker)
    scenarios_per_worker = 2;
end

if autlRunPythonLauncher('collect_parallel', num_workers, scenarios_per_worker)
    result = {'python_launcher', num_workers * scenarios_per_worker};
    return;
end

error('[AutoLandingDataParallel] Python launcher not found. Expected scripts/autolanding_launcher.py');
end
