function y_pred = autlPredictHybrid(model, X_test)
% autlPredictHybrid
% Make predictions using hybrid (ontology+AI) model
%
% Inputs:
%   model     - Trained hybrid model struct with classifier
%   X_test    - Test feature matrix (N x M)
%
% Outputs:
%   y_pred    - Predicted labels (N x 1)

if isempty(model) || isempty(model.classifier)
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
