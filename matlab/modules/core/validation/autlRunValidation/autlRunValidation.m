function summary = autlRunValidation(rootDir, semanticInputPath)
% autlRunValidation
% Run pipeline and collect validation metrics in a dedicated artifact.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = pwd;
end
if nargin < 2 || strlength(string(semanticInputPath)) == 0
    semanticInputPath = fullfile(rootDir, 'data', 'samples', 'semantic_input_example.json');
end

cfg = autlDefaultConfig();
semanticInput = autlLoadJson(char(semanticInputPath));
contextPath = fullfile(rootDir, semanticInput.context_file);
context = autlLoadJson(contextPath);

semantic = autlExtractSemanticFeatures(context, cfg);
fused = autlFuseConfidence(semantic, cfg, rootDir);
trajTbl = autlGenerateTrajectory(semanticInput.initial_state, semanticInput.target, fused, cfg);

decisionMetrics = autlEvaluateDecisionMetrics(semantic, fused, cfg);
trajectoryMetrics = autlEvaluateTrajectoryMetrics(trajTbl, semanticInput.target);

validation = struct();
validation.decision_metrics = decisionMetrics;
validation.trajectory_metrics = trajectoryMetrics;
validation.meta = struct('semantic_input_path', char(semanticInputPath), 'context_path', contextPath);

outPath = fullfile(rootDir, 'data', 'processed', 'validation_report_matlab.json');
autlSaveJson(outPath, validation);

summary = struct();
summary.validation_report = outPath;
summary.decision = decisionMetrics.decision;
summary.fused_confidence = decisionMetrics.fused_confidence;
summary.rmse_xy = trajectoryMetrics.rmse_xy;
summary.touchdown_error_xy = trajectoryMetrics.touchdown_error_xy;
end
