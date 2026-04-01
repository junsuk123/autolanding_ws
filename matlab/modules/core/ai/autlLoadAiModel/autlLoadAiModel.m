function model = autlLoadAiModel(rootDir, cfg)
% autlLoadAiModel
% Load AI model from MAT file, optionally train if missing.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = pwd;
end

modelPath = fullfile(rootDir, cfg.ai.model_path);
if ~isfile(modelPath)
    if isfield(cfg.ai, 'auto_train_if_missing') && logical(cfg.ai.auto_train_if_missing)
        model = autlTrainAiModel(rootDir, cfg);
        return;
    end
    error('autlLoadAiModel:ModelNotFound', 'Model not found: %s', modelPath);
end

S = load(modelPath, 'model');
if ~isfield(S, 'model')
    error('autlLoadAiModel:InvalidModelFile', 'MAT file does not contain "model": %s', modelPath);
end
model = S.model;
end
