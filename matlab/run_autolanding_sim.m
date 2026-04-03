% run_autolanding_sim.m
% Full simulation mode: run pipeline and execute autonomous mission with telemetry
%
% This script:
%   1) Generates trajectory from MATLAB pipeline (semantic features → AI fusion → trajectory)
%   2) Executes autonomous mission (arm → takeoff → trajectory follow → land → disarm)
%   3) Logs all telemetry and mission results
%
% Prerequisites:
%   - Gazebo simulator with iris_runway.sdf
%   - ArduPilot SITL with gazebo-iris model
%   - MAVLink connectivity at tcp:127.0.0.1:5760

clear all; close all; clc;

fprintf('======================================\n');
fprintf('   AUTOLANDING FULL SIMULATION MODE\n');
fprintf('======================================\n\n');

% Get workspace root
ws_root = fileparts(fileparts(mfilename('fullpath')));
fprintf('[SETUP] Workspace root: %s\n', ws_root);

% Add modules to path
mod_dir = fullfile(ws_root, 'matlab', 'modules');
if exist(mod_dir, 'dir')
    addpath(mod_dir);
else
    error('[ERROR] Modules directory not found');
end

core_dir = fullfile(mod_dir, 'core');
if exist(core_dir, 'dir')
    addpath(genpath(core_dir));
else
    error('[ERROR] Core modules directory not found');
end

% Show current simulation status
fprintf('\n[DIAGNOSTICS] Checking simulation connectivity...\n');
try
    % Try to get vehicle status via MAVProxy
    [status, output] = system('timeout 5 mavproxy.py --master tcp:127.0.0.1:5760 --cmd="status" 2>&1');
    if status == 0
        if contains(output, 'Detected vehicle', 'IgnoreCase', true)
            fprintf('[OK] Vehicle heartbeat detected\n');
            % Extract vehicle info
            if contains(output, 'ArduCopter')
                fprintf('[OK] ArduCopter SITL connected\n');
            end
        else
            fprintf('[WARNING] MAVProxy connected but no vehicle detected\n');
            fprintf('[WARNING] Output: %s\n', output(1:min(200, length(output))));
        end
    else
        fprintf('[WARNING] Could not query vehicle status\n');
        fprintf('[WARNING] Mission may proceed but vehicle control uncertain\n');
    end
catch
    fprintf('[WARNING] Diagnostic check failed\n');
end

% Run full end-to-end pipeline from single entrypoint.
fprintf('\n[SIMULATION] Starting full simulation via AutoLandingMainFull...\n');
try
    mode_env = strtrim(getenv('AUTOLANDING_GAZEBO_MODE'));
    if strcmpi(mode_env, 'server') || strcmpi(mode_env, 'headless')
        AutoLandingMainFull('server');
    else
        AutoLandingMainFull('gui');
    end
    fprintf('\n[SUCCESS] Simulation completed!\n');
catch ME
    fprintf('\n[ERROR] Simulation failed: %s\n', ME.message);
    disp(ME.stack);
end

% Check output files
fprintf('\n[RESULTS] Checking generated files...\n');
data_dir = fullfile(ws_root, 'data', 'processed');
if exist(data_dir, 'dir')
    files = dir(fullfile(data_dir, '*_matlab.*'));
    if ~isempty(files)
        fprintf('[OK] Generated %d output files:\n', length(files));
        for i = 1:length(files)
            fprintf('   - %s\n', files(i).name);
        end
    else
        fprintf('[WARNING] No output files found in %s\n', data_dir);
    end
else
    fprintf('[WARNING] Data directory not found\n');
end

fprintf('\n======================================\n');
fprintf('   SIMULATION COMPLETE\n');
fprintf('======================================\n');
fprintf('Mission results saved to: %s/data/processed/\n', ws_root);
fprintf('Check AutoLandingMain output above for details.\n');
