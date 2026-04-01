function comparison_result = autlCompareModels(model_onto_ai, model_pure_ai, X_val, y_val, use_onto, use_pure)
% autlCompareModels
% Compare two models and generate comparison metrics

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
            y_pred_onto = predict(model_onto_ai.classifier, X_val);
        end
    end
end

if use_pure && ~isempty(model_pure_ai)
    if isfield(model_pure_ai, 'y_pred_val')
        y_pred_pure = model_pure_ai.y_pred_val;
    else
        y_pred_pure = predict(model_pure_ai.classifier, X_val);
    end
end

% Compute metrics for each model
comparison_result.models = {};
comparison_result.metrics = {};

if use_onto && ~isempty(model_onto_ai)
    [acc_onto, metrics_onto] = compute_metrics(y_pred_onto, y_val);
    comparison_result.models{end+1} = 'Ontology+AI (Hybrid)';
    comparison_result.metrics{end+1} = metrics_onto;
end

if use_pure && ~isempty(model_pure_ai)
    [acc_pure, metrics_pure] = compute_metrics(y_pred_pure, y_val);
    comparison_result.models{end+1} = 'Pure AI (Baseline)';
    comparison_result.metrics{end+1} = metrics_pure;
end

% Generate comparison summary
comparison_result.summary_text = generate_comparison_summary(comparison_result.models, comparison_result.metrics);

fprintf('%s\n', comparison_result.summary_text);

end

function [accuracy, metrics] = compute_metrics(y_pred, y_val)
% Compute evaluation metrics

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
    diff = abs(acc1 - acc2);
    
    summary = sprintf('%s\n%s\n', summary, repmat('-', 1, 60));
    if acc1 > acc2
        summary = sprintf('%sWINNER: %s (+%.2f%% improvement)\n', ...
            summary, models{1}, (acc1-acc2)*100);
    elseif acc2 > acc1
        summary = sprintf('%sWINNER: %s (+%.2f%% improvement)\n', ...
            summary, models{2}, (acc2-acc1)*100);
    else
        summary = sprintf('%sRESULT: Tie (both %.4f accuracy)\n', summary, acc1);
    end
end

summary = sprintf('%s%s\n', summary, repmat('=', 1, 60));

end

function autlGenerateComparisonPlots(model_onto_ai, model_pure_ai, X_val, y_val, ...
                                     comparison_result, rootDir, use_onto, use_pure)
% Generate comparison plots

fprintf('[Plots] Generating comparison visualizations...\n');

% Create output directory
plot_dir = fullfile(rootDir, 'data', 'plots');
if ~exist(plot_dir, 'dir')
    mkdir(plot_dir);
end

% Figure 1: Accuracy Comparison
fig1 = figure('Name', 'Model Comparison - Accuracy', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

if numel(comparison_result.metrics) > 0
    num_models = numel(comparison_result.metrics);
    
    % Subplot 1: Accuracy bars
    subplot(2, 3, 1);
    accuracies = [];
    model_names = {};
    for i = 1:num_models
        accuracies = [accuracies; comparison_result.metrics{i}.accuracy];
        model_names{i} = strrep(comparison_result.models{i}, ' ', '\n');
    end
    bar(accuracies, 'FaceColor', [0.2 0.6 0.9]);
    set(gca, 'XTickLabel', model_names);
    ylabel('Accuracy');
    ylim([0 1]);
    title('Validation Accuracy Comparison');
    grid on;
    
    % Subplot 2: Precision bars
    subplot(2, 3, 2);
    precisions = [];
    for i = 1:num_models
        precisions = [precisions; comparison_result.metrics{i}.precision];
    end
    bar(precisions, 'FaceColor', [0.2 0.8 0.4]);
    set(gca, 'XTickLabel', model_names);
    ylabel('Precision');
    ylim([0 1]);
    title('Precision Comparison');
    grid on;
    
    % Subplot 3: Recall bars
    subplot(2, 3, 3);
    recalls = [];
    for i = 1:num_models
        recalls = [recalls; comparison_result.metrics{i}.recall];
    end
    bar(recalls, 'FaceColor', [0.8 0.2 0.4]);
    set(gca, 'XTickLabel', model_names);
    ylabel('Recall');
    ylim([0 1]);
    title('Recall Comparison');
    grid on;
    
    % Subplot 4: F1-Score bars
    subplot(2, 3, 4);
    f1s = [];
    for i = 1:num_models
        f1s = [f1s; comparison_result.metrics{i}.f1];
    end
    bar(f1s, 'FaceColor', [0.9 0.7 0.2]);
    set(gca, 'XTickLabel', model_names);
    ylabel('F1-Score');
    ylim([0 1]);
    title('F1-Score Comparison');
    grid on;
    
    % Subplot 5: Confusion matrix for model 1
    if num_models >= 1
        subplot(2, 3, 5);
        m1 = comparison_result.metrics{1};
        cm1 = [m1.TN, m1.FP; m1.FN, m1.TP];
        imagesc(cm1);
        colorbar;
        set(gca, 'XTickLabel', {'Negative', 'Positive'});
        set(gca, 'YTickLabel', {'Negative', 'Positive'});
        title(sprintf('Confusion Matrix\n%s', comparison_result.models{1}));
        for i = 0:1
            for j = 0:1
                text(j+1, i+1, sprintf('%d', cm1(i+1,j+1)), ...
                    'HorizontalAlignment', 'center', 'Color', 'white', 'FontSize', 12);
            end
        end
    end
    
    % Subplot 6: Confusion matrix for model 2 (if exists)
    if num_models >= 2
        subplot(2, 3, 6);
        m2 = comparison_result.metrics{2};
        cm2 = [m2.TN, m2.FP; m2.FN, m2.TP];
        imagesc(cm2);
        colorbar;
        set(gca, 'XTickLabel', {'Negative', 'Positive'});
        set(gca, 'YTickLabel', {'Negative', 'Positive'});
        title(sprintf('Confusion Matrix\n%s', comparison_result.models{2}));
        for i = 0:1
            for j = 0:1
                text(j+1, i+1, sprintf('%d', cm2(i+1,j+1)), ...
                    'HorizontalAlignment', 'center', 'Color', 'white', 'FontSize', 12);
            end
        end
    end
end

% Save figure
plot_file1 = fullfile(plot_dir, 'model_comparison.fig');
savefig(fig1, plot_file1);
fprintf('[Plots] Saved: %s\n', plot_file1);

% Save as image
plot_file1_png = fullfile(plot_dir, 'model_comparison.png');
saveas(fig1, plot_file1_png);
fprintf('[Plots] Saved: %s\n', plot_file1_png);

close(fig1);

end
