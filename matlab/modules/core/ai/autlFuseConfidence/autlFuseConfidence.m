function fused = autlFuseConfidence(semantic, cfg, rootDir)
% autlFuseConfidence
% Fuse ontology semantic safety with AI model confidence.

if nargin < 3 || strlength(string(rootDir)) == 0
    rootDir = pwd;
end

semanticRisk = semantic.semantic_risk;
semanticSafety = semantic.semantic_safety;

aiFeatures = autlBuildAiFeatureVector(semantic, cfg);
model = autlLoadAiModel(rootDir, cfg);
aiConfidence = autlPredictAiConfidence(aiFeatures, model);

fused = struct();
fused.model_name = string(model.name);
fused.ai_features = aiFeatures;
fused.ai_confidence = aiConfidence;
fused.semantic_safety = semanticSafety;
fused.fused_confidence = autlClamp( ...
    cfg.fusion.semantic_weight * semanticSafety + cfg.fusion.ai_weight * aiConfidence, ...
    0.0, 1.0);
end
