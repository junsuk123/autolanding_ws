function run_autolanding_pipeline()
% run_autolanding_pipeline
% Script helper for -batch execution.

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cd(rootDir);
AutoLandingMainFull();
end
