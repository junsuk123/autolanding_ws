# MATLAB Modular Structure

This MATLAB implementation follows a modular layout inspired by IICC26_ws.

## Design rule

Each function has its own folder and the function file name matches the folder name:

### Core modules (20 existing)

- modules/core/config/autlDefaultConfig/autlDefaultConfig.m
- modules/core/io/autlLoadJson/autlLoadJson.m
- modules/core/io/autlSaveJson/autlSaveJson.m
- modules/core/io/autlWriteTrajectoryCsv/autlWriteTrajectoryCsv.m
- modules/core/ontology/autlExtractSemanticFeatures/autlExtractSemanticFeatures.m
- modules/core/ai/autlBuildAiFeatureVector/autlBuildAiFeatureVector.m
- modules/core/ai/autlLoadAiModel/autlLoadAiModel.m
- modules/core/ai/autlPredictAiConfidence/autlPredictAiConfidence.m
- modules/core/ai/autlTrainAiModel/autlTrainAiModel.m
- modules/core/ai/autlFuseConfidence/autlFuseConfidence.m
- modules/core/trajectory/autlGenerateTrajectory/autlGenerateTrajectory.m
- modules/core/metrics/autlEvaluateDecisionMetrics/autlEvaluateDecisionMetrics.m
- modules/core/metrics/autlEvaluateTrajectoryMetrics/autlEvaluateTrajectoryMetrics.m
- modules/core/validation/autlRunValidation/autlRunValidation.m
- modules/core/ros_io/autlCreateRosContext/autlCreateRosContext.m
- modules/core/ros_io/autlReadRosObservation/autlReadRosObservation.m
- modules/core/ros_io/autlPublishTrajectory/autlPublishTrajectory.m
- modules/core/ros_io/autlReleaseRosContext/autlReleaseRosContext.m
- modules/core/utils/autlClamp/autlClamp.m
- modules/core/pipeline/autlRunPipeline/autlRunPipeline.m

### Mission control modules (NEW - Options 2,3)

- modules/core/mavproxy/autlMavproxyControl/autlMavproxyControl.m
  - Interfaces with ArduPilot SITL via MAVProxy
  - Actions: arm, disarm, takeoff, land, set_velocity, set_mode, status

- modules/core/mission/autlAutonomousMission/autlAutonomousMission.m
  - Executes autonomous landing mission
  - Flow: arm → takeoff → trajectory follow → land → disarm
  - Integrates autlMavproxyControl for vehicle commands

## Entry points

### Pipeline mode
- AutoLandingMain.m (or AutoLandingMain('pipeline'))
- scripts/run_autolanding_pipeline.m
- scripts/run_pipeline_matlab.sh

### Mission mode
- AutoLandingMain('mission')
- scripts/run_autolanding_mission.m
- scripts/run_mission_matlab.sh

### Simulation mode
- AutoLandingMain('sim')
- scripts/run_autolanding_sim.m
- scripts/run_sim_matlab.sh

### Validation mode
- scripts/run_autolanding_validation.m
- scripts/run_validation_matlab.sh

### ROS demo mode
- scripts/run_autolanding_ros_demo.m

## Run options

In MATLAB:

```matlab
% Pipeline only (generate trajectory)
AutoLandingMain()
% or
AutoLandingMain('pipeline')

% Run autonomous mission (requires Gazebo + ArduPilot SITL)
AutoLandingMain('mission')

% Full simulation mode (pipeline + mission)
AutoLandingMain('sim')
```

In shell:

```bash
# Pipeline
bash scripts/run_pipeline_matlab.sh

# Autonomous mission
bash scripts/run_mission_matlab.sh

# Full simulation
bash scripts/run_sim_matlab.sh

# Validation
bash scripts/run_validation_matlab.sh
```

Validation run:

```bash
bash scripts/run_validation_matlab.sh
```

ROS2 demo run in MATLAB:

```matlab
run('matlab/scripts/run_autolanding_ros_demo.m')
```
