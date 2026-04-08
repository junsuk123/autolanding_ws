# Deprecated Shell Scripts

These shell scripts were replaced by MATLAB batch entrypoints for the main pipeline. The Python launchers below are legacy helper scripts kept for reference and auxiliary tooling.

## Replacement Mapping

| Deprecated Shell Script | Active Replacement | Notes |
|------------------------|-------------------|-------|
| `launch_multi_drone_sitl.sh` | `matlab/scripts/run_autolanding_pipeline.m` | Main pipeline entrypoint |
| `launch_multi_mavros.sh` | `matlab/run_autolanding_mission.m` | Mission execution entrypoint |
| `launch_rviz_monitor.sh` | `matlab/run_autolanding_sim.m` | Simulation/monitoring entrypoint |

## Why MATLAB?

1. **Single orchestration path**: Pipeline, collection, validation, and mission control now share the same MATLAB entrypoints
2. **Config consistency**: One orchestration config drives the full workflow
3. **Modular deployment**: Core logic stays in MATLAB modules, while Python remains optional for auxiliary tooling

## Migration Status

All main functionality is now driven by MATLAB batch entrypoints:
- [run_autolanding_pipeline.m](../../matlab/scripts/run_autolanding_pipeline.m)
- [run_autolanding_validation.m](../../matlab/scripts/run_autolanding_validation.m)
- [run_autolanding_mission.m](../../matlab/run_autolanding_mission.m)
- [run_autolanding_sim.m](../../matlab/run_autolanding_sim.m)

The legacy Python helpers remain available only for auxiliary visualization and non-primary tooling.

## Archival

These scripts are kept for reference only. Do not use them in production.
