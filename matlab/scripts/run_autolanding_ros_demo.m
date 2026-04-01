function run_autolanding_ros_demo()
% run_autolanding_ros_demo
% Demo: create ROS context, run pipeline, publish trajectory.

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
modDir = fullfile(rootDir, 'matlab', 'modules');
if exist(modDir, 'dir'), addpath(modDir); end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir'), addpath(genpath(coreDir)); end

semanticInputPath = fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json');
summary = autlRunPipeline(rootDir, semanticInputPath);

trajPath = fullfile(rootDir, 'data', 'processed', 'landing_trajectory_matlab.csv');
trajTbl = readtable(trajPath);

rosCtx = autlCreateRosContext("autolanding_matlab_demo");
pubStatus = autlPublishTrajectory(rosCtx, trajTbl, "/autolanding/trajectory");
autlReleaseRosContext(rosCtx);

disp('[run_autolanding_ros_demo] pipeline summary');
disp(summary);
disp('[run_autolanding_ros_demo] publish status');
disp(pubStatus);
end
