function run_autolanding_validation()
% run_autolanding_validation
% Validation-only script helper.

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cd(rootDir);
addpath(fullfile(rootDir, 'matlab'));
summary = AutoLandingMainFull('validation');
disp('[run_autolanding_validation] done');
disp(summary);
end
