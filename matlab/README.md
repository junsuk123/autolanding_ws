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

## MAVLink <-> ROS Round-Trip

When working with MAVROS message bridges, a MAVLink packet can be converted to a ROS message and then converted back again without forwarding through a live topic path. The conversion flow is:

1. Read a MAVLink message with `recv_msg()`.
2. Convert it to a ROS message with `mavlink.convert_to_rosmsg(msg)`.
3. Convert the ROS message back to bytes with `mavlink.convert_to_bytes(rosmsg)`.
4. Decode the bytes again with the local MAVLink instance.

One important detail: if the ROS message is going to be repacked or re-encoded locally, call `msg.pack(conn.mav)` before conversion. That refreshes the message against the local MAVLink instance and avoids CRC mismatches during the round-trip.

Minimal example:

```python
from pymavlink import mavutil
from mavros import mavlink

conn = mavutil.mavlink_connection("tcp:192.168.0.2:5760")

while True:
  msg = conn.recv_msg()
  msg.pack(conn.mav)
  rosmsg = mavlink.convert_to_rosmsg(msg)
  payload = mavlink.convert_to_bytes(rosmsg)
  original_message = conn.mav.decode(payload)
  print(original_message)
```

This is useful when you need to inspect, transform, or relay MAVLink traffic independently of the GCS or drone transport layer.

## Current Runtime Behavior

- `AutoLandingMainFull()` is the main automatic entrypoint for pipeline, validation, collection, mission, and full modes.
- `run_autolanding_pipeline()` forwards into `AutoLandingMainFull('full')`.
- Collection outputs are written into date/time stamped folders under `data/collected/` and model runs are grouped under timestamped folders under `data/models/`.
- Validation defaults to the latest saved models, but can be pointed at specific model paths from `ai/configs/orchestration_config.yaml`.
