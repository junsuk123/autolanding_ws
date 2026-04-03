function comparison_result = autlCompareModels(model_onto_ai, model_pure_ai, X_val, y_val, use_onto, use_pure)
% autlCompareModels
% Compare two configured models (hybrid ontology+AI vs pure AI)
%
% Inputs:
%   model_onto_ai     - Trained ontology+AI hybrid model struct
%   model_pure_ai     - Trained pure AI baseline model struct
%   X_val             - Validation feature matrix (N x M)
%   y_val             - Validation labels (N x 1)
%   use_onto          - Boolean: whether hybrid model was trained
%   use_pure          - Boolean: whether pure AI model was trained
%
% Outputs:
%   comparison_result - Struct with models, metrics, and summary text
%
% DESCRIPTION:
%   Compares model performance metrics and generates comparison summary.
%   Computes accuracy, precision, recall, and F1-score for each model.

comparison_result = struct();

% Get predictions for both models
y_pred_onto = [];
y_pred_pure = [];

if use_onto && ~isempty(model_onto_ai)
    if isfield(model_onto_ai, 'y_pred_val')
        y_pred_onto = model_onto_ai.y_pred_val;
    else
        if isfield(model_onto_ai, 'is_nn') && model_onto_ai.is_nn
            y_pred_onto = autlPredictHybrid(model_onto_ai, X_val);
        else
            y_pred_onto = autlSafePredictLabels(model_onto_ai.classifier, X_val, size(y_val, 1));
        end
    end
end

if use_pure && ~isempty(model_pure_ai)
    if isfield(model_pure_ai, 'y_pred_val')
        y_pred_pure = model_pure_ai.y_pred_val;
    else
        y_pred_pure = autlSafePredictLabels(model_pure_ai.classifier, X_val, size(y_val, 1));
    end
end

% Compute metrics for each model
comparison_result.models = {};
comparison_result.metrics = {};

if use_onto && ~isempty(model_onto_ai)
    [~, metrics_onto] = compute_metrics(y_pred_onto, y_val);
    comparison_result.models{end+1} = 'Ontology+AI (Hybrid)';
    comparison_result.metrics{end+1} = metrics_onto;
end

if use_pure && ~isempty(model_pure_ai)
    [~, metrics_pure] = compute_metrics(y_pred_pure, y_val);
    comparison_result.models{end+1} = 'Pure AI (Baseline)';
    comparison_result.metrics{end+1} = metrics_pure;
end

% Generate comparison summary
comparison_result.summary_text = generate_comparison_summary(comparison_result.models, comparison_result.metrics);

end

function [accuracy, metrics] = compute_metrics(y_pred, y_val)
% Compute evaluation metrics
%   Inputs: y_pred (predictions), y_val (ground truth)
%   Outputs: accuracy (scalar), metrics (struct with precision, recall, F1, confusion matrix)

accuracy = mean(y_pred == y_val);

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

end

function summary = generate_comparison_summary(models, metrics)
% Generate text summary of model comparison
%   Inputs: models (cell array of names), metrics (cell array of metric structs)
%   Outputs: summary (formatted text string)

summary = sprintf('\n%s\n', repmat('=', 1, 60));
summary = sprintf('%sMODEL COMPARISON RESULTS\n', summary);
summary = sprintf('%s%s\n', summary, repmat('=', 1, 60));

for i = 1:numel(models)
    m = metrics{i};
    summary = sprintf('%s\n%s:\n', summary, models{i});
    summary = sprintf('%s  Accuracy:  %.4f\n', summary, m.accuracy);
    summary = sprintf('%s  Precision: %.4f\n', summary, m.precision);
    summary = sprintf('%s  Recall:    %.4f\n', summary, m.recall);
    summary = sprintf('%s  F1-Score:  %.4f\n', summary, m.f1);
    summary = sprintf('%s  TP: %d, FP: %d, FN: %d, TN: %d\n', ...
        summary, m.TP, m.FP, m.FN, m.TN);
end

% Winner determination
if numel(metrics) >= 2
    acc1 = metrics{1}.accuracy;
    acc2 = metrics{2}.accuracy;
    
    summary = sprintf('%s\n%s\n', summary, repmat('-', 1, 60));
    if acc1 > acc2 + 1e-6
        summary = sprintf('%sWINNER: %s (+%.2f%% improvement)\n', ...
            summary, models{1}, (acc1-acc2)*100);
    elseif acc2 > acc1 + 1e-6
        summary = sprintf('%sWINNER: %s (+%.2f%% improvement)\n', ...
            summary, models{2}, (acc2-acc1)*100);
    else
        summary = sprintf('%sRESULT: Tie (both %.4f accuracy)\n', summary, acc1);
    end
end

summary = sprintf('%s%s\n', summary, repmat('=', 1, 60));

end

function y_pred = autlSafePredictLabels(classifier, X_val, n_rows)
% Safely run classifier prediction and normalize output to numeric 0/1 labels.

y_pred = zeros(n_rows, 1);

if isempty(classifier)
    return;
end

try
    y_raw = predict(classifier, X_val);
catch ME
    fprintf('[autlCompareModels] WARNING: predict failed (%s). Using all-zero fallback labels.\n', ME.message);
    return;
end

if islogical(y_raw)
    y_pred = double(y_raw(:));
elseif isnumeric(y_raw)
    y_pred = double(y_raw(:));
elseif iscategorical(y_raw)
    y_pred = double(string(y_raw) == "1");
elseif isstring(y_raw) || ischar(y_raw) || iscellstr(y_raw)
    y_pred = double(string(y_raw(:)) == "1");
else
    % Unknown label type; keep robust all-zero default.
    return;
end

if numel(y_pred) ~= n_rows
    y_pred = y_pred(1:min(end, n_rows));
    if numel(y_pred) < n_rows
        y_pred(end+1:n_rows, 1) = 0;
    end
end

% Normalize arbitrary numeric labels to binary class labels.
y_pred = double(y_pred > 0.5);

end
