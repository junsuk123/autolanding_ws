function summary = autlRunPipeline(rootDir, semanticInputPath)
% autlRunPipeline
% End-to-end ontology->semantic->AI fusion->trajectory pipeline.

if nargin < 1 || strlength(string(rootDir)) == 0
    rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
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

outJson = fullfile(rootDir, 'data', 'processed', 'landing_trajectory_matlab.json');
outCsv = fullfile(rootDir, 'data', 'processed', 'landing_trajectory_matlab.csv');
outValidation = fullfile(rootDir, 'data', 'processed', 'landing_validation_matlab.json');

result = struct();
result.semantic_features = semantic;
result.fusion = fused;
result.trajectory = table2struct(trajTbl);
result.decision_metrics = decisionMetrics;
result.trajectory_metrics = trajectoryMetrics;
result.meta = struct('semantic_input_path', char(semanticInputPath), 'context_path', contextPath);

autlSaveJson(outJson, result);
autlWriteTrajectoryCsv(outCsv, trajTbl);
autlSaveJson(outValidation, struct('decision_metrics', decisionMetrics, 'trajectory_metrics', trajectoryMetrics));

summary = struct();
summary.out_json = outJson;
summary.out_csv = outCsv;
summary.out_validation_json = outValidation;
summary.landing_trajectory = trajTbl;
summary.initial_state = [semanticInput.initial_state.x, semanticInput.initial_state.y, semanticInput.initial_state.z];
summary.target_state = [semanticInput.target.x, semanticInput.target.y, semanticInput.target.z];
summary.trajectory_points = height(trajTbl);
summary.semantic_risk = semantic.semantic_risk;
summary.fused_confidence = fused.fused_confidence;
summary.decision = string(decisionMetrics.decision);
summary.touchdown_error_xy = trajectoryMetrics.touchdown_error_xy;
end
