function model = autlTrainPureAiModel(X_train, y_train, X_val, y_val, rootDir)
% autlTrainPureAiModel
% Train pure AI model (baseline) without ontology features
% Uses decision tree classifier as baseline approach
%
% Inputs:
%   X_train   - Training features (N x M)
%   y_train   - Training labels (N x 1)
%   X_val     - Validation features (M x M)
%   y_val     - Validation labels (M x 1)
%   rootDir   - Root directory for saving models
%
% Outputs:
%   model     - Struct with trained classifier and metrics
%
% DESCRIPTION:
%   Trains a simple decision tree classifier as a baseline for
%   comparison against the ontology+AI hybrid model.

fprintf('[PureAI] Training Pure AI Baseline Model...\n');

model = struct();
model.type = 'pure_ai';
model.train_time = datetime('now');

try
    % Use simpler model (decision tree) as baseline
    fprintf('[PureAI] Training decision tree (no semantic features)...\n');
    
    % Create classification tree
    tree = fitctree(X_train, y_train, 'MaxNumSplits', 10, 'MinLeafSize', 5);
    model.classifier = tree;
    
    % Evaluate on training set
    y_pred_train = predict(model.classifier, X_train);
    acc_train = mean(y_pred_train == y_train);
    
    % Evaluate on validation set
    y_pred_val = predict(model.classifier, X_val);
    acc_val = mean(y_pred_val == y_val);
    
    fprintf('[PureAI] Training Accuracy: %.4f\n', acc_train);
    fprintf('[PureAI] Validation Accuracy: %.4f\n', acc_val);
    
    model.acc_train = acc_train;
    model.acc_val = acc_val;
    model.y_pred_val = y_pred_val;
    
    % Save model
    model_dir = fullfile(rootDir, 'data', 'models');
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end
    
    model_file = fullfile(model_dir, 'model_pure_ai.mat');
    save(model_file, 'model');
    fprintf('[PureAI] Model saved: %s\n', model_file);
    
catch ME
    fprintf('[PureAI] Training failed: %s\n', ME.message);
    model.error = ME.message;
    model.classifier = [];
end

end
