function [accuracy, metrics] = autlEvaluateModel(model, X_val, y_val)
% autlEvaluateModel
% Evaluate model on validation set
%
% Inputs:
%   model     - Trained model struct (hybrid_ontology_ai or pure_ai)
%   X_val     - Validation feature matrix (N x M)
%   y_val     - Validation labels (N x 1)
%
% Outputs:
%   accuracy  - Overall accuracy
%   metrics   - Struct with precision, recall, F1, and confusion matrix
%
% DESCRIPTION:
%   Computes multiple evaluation metrics for classification models
%   including accuracy, precision, recall, F1-score, and confusion matrix elements.

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
    
catch ME
    fprintf('[autlEvaluateModel] Error: %s\n', ME.message);
    accuracy = 0;
    metrics = struct();
end

end
