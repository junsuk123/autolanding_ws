function pSafe = autlPredictAiConfidence(features, model)
% autlPredictAiConfidence
% Predict safe-landing confidence from AI model.

x = double(features(:))';
if ~isfield(model, 'type')
    error('autlPredictAiConfidence:InvalidModel', 'Model type is missing.');
end

switch lower(string(model.type))
    case "logistic_linear"
        w = double(model.weights(:))';
        if numel(x) ~= numel(w)
            error('autlPredictAiConfidence:DimMismatch', 'Feature dimension mismatch.');
        end
        b = double(model.bias);
        z = sum(w .* x) + b;
        pSafe = 1.0 ./ (1.0 + exp(-z));
    otherwise
        error('autlPredictAiConfidence:UnknownModel', 'Unknown model type: %s', string(model.type));
end

pSafe = autlClamp(double(pSafe), 0.0, 1.0);
end
