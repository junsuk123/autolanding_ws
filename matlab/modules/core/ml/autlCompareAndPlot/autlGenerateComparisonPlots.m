function autlGenerateComparisonPlots(model_onto_ai, model_pure_ai, X_val, y_val, ...
                                     comparison_result, rootDir, use_onto, use_pure)
% autlGenerateComparisonPlots
% Generate comparison visualizations for ontology+AI vs pure AI models
%
% Inputs:
%   model_onto_ai         - Trained ontology+AI hybrid model struct
%   model_pure_ai         - Trained pure AI baseline model struct
%   X_val                 - Validation feature matrix (N x M)
%   y_val                 - Validation labels (N x 1)
%   comparison_result     - Comparison results struct from autlCompareModels
%   rootDir               - Root directory for saving plots
%   use_onto              - Boolean: whether hybrid model was trained
%   use_pure              - Boolean: whether pure AI model was trained
%
% DESCRIPTION:
%   Creates multi-panel comparison figures showing:
%   - Accuracy, Precision, Recall, F1-Score bars
%   - Confusion matrices for each model
%   Saves figures in both .fig and .png formats.

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
