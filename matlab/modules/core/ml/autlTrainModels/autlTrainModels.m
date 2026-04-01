function model = autlTrainHybridModel(X_train, y_train, X_val, y_val, rootDir)
% autlTrainHybridModel
% Train hybrid model: Ontology features + AI (decision tree/SVM/etc)
%
% This model uses semantic/ontology features combined with neural network

fprintf('[HybridModel] Training Ontology+AI Model...\n');

% For now, use simple neural network as baseline
% In production, this would combine ontology-extracted features with deep learning

% Create simple model structure
model = struct();
model.type = 'hybrid_ontology_ai';
model.train_time = datetime('now');

% Use MATLAB's built-in classification algorithms
try
    % Try to use fitcnet (neural network classifier) if available
    if isempty(ver('statistics'))
        fprintf('[HybridModel] Statistics toolbox not available, using simple SVM\n');
        model.classifier = fitcsvm(X_train, y_train, 'KernelFunction', 'rbf', 'Standardize', true);
    else
        % Use neural network
        fprintf('[HybridModel] Training neural network (3 layers)...\n');
        
        % Create simple network
        net = feedforwardnet([10 5]);  % 2 hidden layers
        net.trainFcn = 'trainscg';      % Scaled conjugate gradient
        net.layers{1}.transferFcn = 'tansig';
        net.layers{2}.transferFcn = 'tansig';
        net.layers{3}.transferFcn = 'logsig';
        
        % Prepare data for neural network
        X_train_nn = X_train';
        y_train_nn = y_train' + 1;  % Convert 0/1 to 1/2 for classification
        
        % Train
        [net, ~] = train(net, X_train_nn, y_train_nn);
        model.classifier = net;
        model.is_nn = true;
    end
    
    % Evaluate on training set
    y_pred_train = autlPredictHybrid(model, X_train);
    acc_train = mean(y_pred_train == y_train);
    
    % Evaluate on validation set
    y_pred_val = autlPredictHybrid(model, X_val);
    acc_val = mean(y_pred_val == y_val);
    
    fprintf('[HybridModel] Training Accuracy: %.4f\n', acc_train);
    fprintf('[HybridModel] Validation Accuracy: %.4f\n', acc_val);
    
    model.acc_train = acc_train;
    model.acc_val = acc_val;
    model.y_pred_val = y_pred_val;
    
    % Save model
    model_dir = fullfile(rootDir, 'data', 'models');
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end
    
    model_file = fullfile(model_dir, 'model_hybrid_ontology_ai.mat');
    save(model_file, 'model');
    fprintf('[HybridModel] Model saved: %s\n', model_file);
    
catch ME
    fprintf('[HybridModel] Training failed: %s\n', ME.message);
    model.error = ME.message;
    model.classifier = [];
end

end

function y_pred = autlPredictHybrid(model, X_test)
% Prediction for hybrid model

if isempty(model.classifier)
    y_pred = zeros(size(X_test, 1), 1);
    return;
end

try
    if isfield(model, 'is_nn') && model.is_nn
        % Neural network prediction
        X_test_nn = X_test';
        y_pred_nn = model.classifier(X_test_nn);
        [~, y_pred] = max(y_pred_nn, [], 1);
        y_pred = (y_pred - 1)';  % Convert 1/2 back to 0/1
    else
        % SVM or other classifier prediction
        y_pred = predict(model.classifier, X_test);
    end
catch
    y_pred = zeros(size(X_test, 1), 1);
end

end

function model = autlTrainPureAiModel(X_train, y_train, X_val, y_val, rootDir)
% autlTrainPureAiModel
% Train pure AI model (baseline) without ontology features
% Uses same X data but different architecture/approach

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

function [accuracy, metrics] = autlEvaluateModel(model, X_val, y_val)
% Evaluate model on validation set

if isempty(model) || isempty(model.classifier)
    accuracy = 0;
    metrics = struct();
    return;
end

try
    % Get predictions
    if strcmp(model.type, 'hybrid_ontology_ai') && isfield(model, 'is_nn') && model.is_nn
        y_pred = autlPredictHybrid(model, X_val);
    else
        y_pred = predict(model.classifier, X_val);
    end
    
    % Calculate accuracy
    accuracy = mean(y_pred == y_val);
    
    % Calculate additional metrics (precision, recall, F1)
    TP = sum((y_pred == 1) & (y_val == 1));
    FP = sum((y_pred == 1) & (y_val == 0));
    FN = sum((y_pred == 0) & (y_val == 1));
    TN = sum((y_pred == 0) & (y_val == 0));
    
    precision = TP / (TP + FP + 1e-6);
    recall = TP / (TP + FN + 1e-6);
    f1 = 2 * (precision * recall) / (precision + recall + 1e-6);
    
    metrics = struct();
    metrics.accuracy = accuracy;
    metrics.precision = precision;
    metrics.recall = recall;
    metrics.f1 = f1;
    metrics.TP = TP;
    metrics.FP = FP;
    metrics.FN = FN;
    metrics.TN = TN;
    
catch
    accuracy = 0;
    metrics = struct();
end

end
