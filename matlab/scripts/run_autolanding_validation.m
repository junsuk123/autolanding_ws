function run_autolanding_validation()
% run_autolanding_validation
% Validation-only script helper.

rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
modDir = fullfile(rootDir, 'matlab', 'modules');
if exist(modDir, 'dir'), addpath(modDir); end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir'), addpath(genpath(coreDir)); end

semanticInputPath = fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json');
summary = autlRunValidation(rootDir, semanticInputPath);
disp('[run_autolanding_validation] done');
disp(summary);
end
