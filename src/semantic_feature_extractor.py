import math
from typing import Dict, List


def _safe_mean(values: List[float]) -> float:
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def extract_semantic_features(context: Dict) -> Dict[str, float]:
    objects = context.get("objects", [])
    relations = context.get("relations", [])

    object_count = float(len(objects))
    relation_count = float(len(relations))
    relation_density = relation_count / max(object_count, 1.0)

    wind_speeds = []
    gust_risks = []
    tag_confidences = []
    tag_errors = []
    yaw_errors = []

    for obj in objects:
        attributes = obj.get("attributes", {})
        if "wind_speed" in attributes:
            wind_speeds.append(float(attributes["wind_speed"]))
        if "gust_risk" in attributes:
            gust_risks.append(float(attributes["gust_risk"]))
        if "tag_confidence" in attributes:
            tag_confidences.append(float(attributes["tag_confidence"]))
        if "tag_error" in attributes:
            tag_errors.append(float(attributes["tag_error"]))
        if "yaw_error" in attributes:
            yaw_errors.append(float(attributes["yaw_error"]))

    wind_speed_mean = _safe_mean(wind_speeds)
    gust_risk_mean = _safe_mean(gust_risks)
    tag_confidence_mean = _safe_mean(tag_confidences)
    tag_error_mean = _safe_mean(tag_errors)
    yaw_error_mean = _safe_mean(yaw_errors)

    wind_risk = min(1.0, 0.1 * wind_speed_mean + 0.6 * gust_risk_mean)
    vision_risk = min(1.0, max(0.0, 1.0 - tag_confidence_mean) + 0.5 * tag_error_mean)
    alignment_risk = min(1.0, abs(yaw_error_mean) / math.pi)

    semantic_risk = min(1.0, 0.5 * wind_risk + 0.3 * vision_risk + 0.2 * alignment_risk)

    return {
        "object_count": object_count,
        "relation_count": relation_count,
        "relation_density": relation_density,
        "wind_risk": wind_risk,
        "vision_risk": vision_risk,
        "alignment_risk": alignment_risk,
        "semantic_risk": semantic_risk,
        "semantic_safety": 1.0 - semantic_risk,
    }
