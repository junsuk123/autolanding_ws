% run_autolanding_mission.m
% Execute autonomous landing mission with MATLAB trajectory and MAVProxy control
% 
% Prerequisites:
%   - Gazebo simulator running: gz sim iris_runway.sdf
%   - ArduPilot SITL running: ardupilot with gazebo-iris model
%   - MAVProxy connection available at tcp:127.0.0.1:5760

clear all; close all; clc;

fprintf('======================================\n');
fprintf('   AUTOLANDING MISSION EXECUTION\n');
fprintf('======================================\n\n');

% Get workspace root
ws_root = fileparts(fileparts(mfilename('fullpath')));
fprintf('[SETUP] Workspace root: %s\n', ws_root);

% Add modules to path
mod_dir = fullfile(ws_root, 'matlab', 'modules');
if exist(mod_dir, 'dir')
    addpath(mod_dir);
    fprintf('[SETUP] Added modules to path\n');
else
    error('[ERROR] Modules directory not found at %s', mod_dir);
end

core_dir = fullfile(mod_dir, 'core');
if exist(core_dir, 'dir')
    addpath(genpath(core_dir));
    fprintf('[SETUP] Added core modules to path\n');
else
    error('[ERROR] Core modules directory not found at %s', core_dir);
end

% Run AutoLandingMain in mission mode
fprintf('\n[MISSION] Starting autonomous mission in MATLAB...\n');
try
    AutoLandingMain('mission');
    fprintf('\n[SUCCESS] Mission completed successfully!\n');
catch ME
    fprintf('\n[ERROR] Mission failed: %s\n', ME.message);
    disp(ME.stack);
end

fprintf('\n======================================\n');
fprintf('   MISSION EXECUTION COMPLETE\n');
fprintf('======================================\n');
