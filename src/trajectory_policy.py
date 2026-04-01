from typing import Dict, List


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def generate_trajectory(
    initial_state: Dict[str, float],
    target: Dict[str, float],
    semantic_features: Dict[str, float],
    config: Dict,
) -> List[Dict[str, float]]:
    policy_cfg = config["policy"]
    fusion_cfg = config["fusion"]

    horizon_s = float(policy_cfg["horizon_s"])
    dt = float(policy_cfg["dt"])
    steps = max(1, int(horizon_s / dt))

    max_xy_speed = float(policy_cfg["max_xy_speed"])
    max_z_descent = float(policy_cfg["max_z_descent_speed"])
    min_z_descent = float(policy_cfg["min_z_descent_speed"])

    semantic_weight = float(fusion_cfg["semantic_weight"])
    ai_weight = float(fusion_cfg["ai_weight"])

    x = float(initial_state["x"])
    y = float(initial_state["y"])
    z = float(initial_state["z"])

    tx = float(target["x"])
    ty = float(target["y"])
    tz = float(target["z"])

    semantic_safety = float(semantic_features.get("semantic_safety", 0.5))
    semantic_risk = float(semantic_features.get("semantic_risk", 0.5))

    # Placeholder AI confidence. Replace with a learned model output.
    ai_confidence = _clip(1.0 - 0.8 * semantic_risk, 0.0, 1.0)

    fused_confidence = _clip(
        semantic_weight * semantic_safety + ai_weight * ai_confidence,
        0.0,
        1.0,
    )

    trajectory = []
    for step in range(steps + 1):
        t = step * dt

        dx = tx - x
        dy = ty - y
        dz = z - tz

        vx_cmd = _clip(0.8 * dx, -max_xy_speed, max_xy_speed)
        vy_cmd = _clip(0.8 * dy, -max_xy_speed, max_xy_speed)

        base_descent = min_z_descent + (max_z_descent - min_z_descent) * fused_confidence
        vz_cmd = -_clip(base_descent, min_z_descent, max_z_descent)

        if dz < 0.2:
            vz_cmd = -min_z_descent * 0.4

        x += vx_cmd * dt
        y += vy_cmd * dt
        z = max(tz, z + vz_cmd * dt)

        trajectory.append(
            {
                "t": t,
                "x": x,
                "y": y,
                "z": z,
                "vx_cmd": vx_cmd,
                "vy_cmd": vy_cmd,
                "vz_cmd": vz_cmd,
                "fused_confidence": fused_confidence,
            }
        )

        if z <= tz + 0.02 and abs(x - tx) < 0.05 and abs(y - ty) < 0.05:
            break

    return trajectory
