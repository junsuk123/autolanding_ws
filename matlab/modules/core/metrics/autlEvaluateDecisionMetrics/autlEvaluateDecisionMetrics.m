function metrics = autlEvaluateDecisionMetrics(semantic, fused, cfg)
% autlEvaluateDecisionMetrics
% Compute decision-level metrics for current trajectory generation step.

thr = cfg.validation.decision_threshold;

metrics = struct();
metrics.threshold = thr;
metrics.semantic_risk = semantic.semantic_risk;
metrics.ai_confidence = fused.ai_confidence;
metrics.fused_confidence = fused.fused_confidence;
metrics.is_safe_to_land = fused.fused_confidence >= thr;
if metrics.is_safe_to_land
    metrics.decision = "AttemptLanding";
else
    metrics.decision = "HoldLanding";
end
end
