function model = autlTrainAiModel(rootDir, cfg)
% autlTrainAiModel
% Create a lightweight logistic linear model and save it as MAT file.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = pwd;
end

featureNames = string(cfg.ai.feature_names(:));

% Hand-tuned seed weights representing safety behavior priors.
% Positive weight increases safe confidence, negative decreases it.
weightsMap = containers.Map( ...
    {'semantic_risk','semantic_safety','wind_risk','vision_risk','alignment_risk','relation_density'}, ...
    {-2.4, 2.8, -1.6, -1.3, -1.1, 0.4});

w = zeros(1, numel(featureNames));
for i = 1:numel(featureNames)
    key = char(featureNames(i));
    if isKey(weightsMap, key)
        w(i) = weightsMap(key);
    else
        w(i) = 0.0;
    end
end

model = struct();
model.name = "autl_ai_model_v1";
model.type = "logistic_linear";
model.feature_names = featureNames;
model.weights = w;
model.bias = 0.25;
model.created_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));

modelRelPath = cfg.ai.model_path;
modelPath = fullfile(rootDir, modelRelPath);
outDir = fileparts(modelPath);
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
save(modelPath, 'model');
end
