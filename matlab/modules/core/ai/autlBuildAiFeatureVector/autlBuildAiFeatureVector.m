function feat = autlBuildAiFeatureVector(semantic, cfg)
% autlBuildAiFeatureVector
% Build AI feature vector from semantic features by configured names.

names = string(cfg.ai.feature_names(:));
feat = zeros(1, numel(names));
for i = 1:numel(names)
    key = char(names(i));
    if isfield(semantic, key)
        val = semantic.(key);
        if isnumeric(val) && isscalar(val) && isfinite(val)
            feat(i) = double(val);
        else
            feat(i) = 0.0;
        end
    else
        feat(i) = 0.0;
    end
end
end
