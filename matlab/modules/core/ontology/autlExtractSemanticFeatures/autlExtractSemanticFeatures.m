function semantic = autlExtractSemanticFeatures(context, cfg)
% autlExtractSemanticFeatures
% Build semantic risk features from object/attribute/relation context.

objects = context.objects;
relations = context.relations;

nObj = numel(objects);
nRel = numel(relations);

windSpeed = [];
gustRisk = [];
tagConfidence = [];
tagError = [];
yawError = [];

for i = 1:nObj
    attrs = objects(i).attributes;
    if isfield(attrs, 'wind_speed'), windSpeed(end+1) = double(attrs.wind_speed); end %#ok<AGROW>
    if isfield(attrs, 'gust_risk'), gustRisk(end+1) = double(attrs.gust_risk); end %#ok<AGROW>
    if isfield(attrs, 'tag_confidence'), tagConfidence(end+1) = double(attrs.tag_confidence); end %#ok<AGROW>
    if isfield(attrs, 'tag_error'), tagError(end+1) = double(attrs.tag_error); end %#ok<AGROW>
    if isfield(attrs, 'yaw_error'), yawError(end+1) = double(attrs.yaw_error); end %#ok<AGROW>
end

windSpeedMean = localMean(windSpeed);
gustRiskMean = localMean(gustRisk);
tagConfMean = localMean(tagConfidence);
tagErrMean = localMean(tagError);
yawErrMean = localMean(yawError);

windRisk = autlClamp(cfg.risk.wind_scale * windSpeedMean + cfg.risk.gust_scale * gustRiskMean, 0.0, 1.0);
visionRisk = autlClamp((1.0 - tagConfMean) + cfg.risk.vision_err_scale * tagErrMean, 0.0, 1.0);
alignmentRisk = autlClamp(abs(yawErrMean) / pi, 0.0, 1.0);
semanticRisk = autlClamp(0.5 * windRisk + 0.3 * visionRisk + 0.2 * alignmentRisk, 0.0, 1.0);

semantic = struct();
semantic.object_count = nObj;
semantic.relation_count = nRel;
semantic.relation_density = nRel / max(nObj, 1);
semantic.wind_risk = windRisk;
semantic.vision_risk = visionRisk;
semantic.alignment_risk = alignmentRisk;
semantic.semantic_risk = semanticRisk;
semantic.semantic_safety = 1.0 - semanticRisk;
end

function m = localMean(x)
if isempty(x)
    m = 0.0;
else
    x = x(isfinite(x));
    if isempty(x)
        m = 0.0;
    else
        m = mean(x);
    end
end
end
