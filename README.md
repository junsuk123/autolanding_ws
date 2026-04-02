# autolanding_ws

Semantic autonomous landing workspace based on ontology-driven features + AI trajectory policy.

Primary implementation is now MATLAB script-based with modular function folders.

## 1. Goal

This workspace combines:

- Ontology layer: classify observations into object/attribute/relation
- Semantic feature layer: convert graph semantics into numeric AI input
- Policy layer: generate landing trajectory from semantic features
- Simulation layer: run with Gazebo Sim (gzsim) + ArduPilot SITL

## 2. Research formulation

Semantic context graph:

G = (V, E, A)

- V: objects (drone, pad, wind, marker, obstacle)
- E: relations (near, aligned_with, affected_by, visible_to)
- A: attributes (wind_speed, tag_error, tilt, uncertainty)

Semantic feature embedding:

z_sem = Pool(phi(v_i, a_i), psi(e_ij))

AI fusion:

x_fused = [x_sensor ; z_sem]

Landing policy:

pi_theta(x_fused) -> u_t

Trajectory rollout:

s_{t+1} = f(s_t, u_t)

A practical optimization view:

J = sum_t (w_p ||p_t - p_ref||^2 + w_v ||v_t||^2 + w_u ||u_t||^2) + w_r R_sem

where R_sem penalizes semantically unsafe conditions (high wind risk, low visual confidence, unstable state transitions).

ArUco-pad constrained initialization used by the MATLAB pipeline:

- Marker center is fixed at p_m = [x_m, y_m, z_m]
- Drone spawn is constrained by ||p_spawn_xy - p_m_xy|| <= 1.0
- Marker size uses s_marker = s_drone * (2^2)

Hover-to-land trajectory objective with marker target:

J_land = sum_t (w_xy ||p_xy,t - p_m,xy||^2 + w_z (z_t - z_m)^2 + w_v ||v_t||^2)

## 2.1 Paper-oriented experimental protocol

This workspace supports the following experimental flow for thesis/paper reporting:

1. Baseline vs proposal:
- Baseline: Pure AI model
- Proposal: Ontology+AI hybrid model

2. Multi-worker simulation data collection:
- Worker-wise scenario collection is supported.
- Each scenario starts from reset pose/state to reduce inter-scenario leakage.

3. Scenario start behavior:
- ArduPilot takes off and reaches hover before effective data capture.
- Landing pad context is published and logged for downstream interpretation.

4. Model-missing fallback:
- If both learned models are unavailable, baseline tracking trajectory generation is used.
- This guarantees mission continuity and dataset bootstrapping.

5. Training after collection:
- Collected raw data are converted to feature/label sets.
- Pure AI and Ontology+AI models are trained in the same pipeline.

6. Trajectory outputs:
- Both models produce landing trajectories when available.
- A compatibility output is also provided as a single preferred trajectory file.

7. Quantitative validation for paper tables:
- touchdown error (XY/Z), landing time, descent speed, lateral stability
- trajectory RMSE and path length
- cross-model comparison summary

8. Explainability intent:
- Semantic context (wind/vision/state relation) is represented as structured features.
- The intent is to improve human-interpretability over purely black-box decision paths.

## 3. Directory layout

- src: semantic feature extraction and trajectory policy code
- matlab: MATLAB modular implementation (function-per-folder)
- ontology/schema: ontology schema files
- ontology/instances: sample ontology instances
- ai/configs: policy and fusion configurations
- simulation: gzsim + ardupilot helper launch scripts/configs
- data/samples: sample semantic input and outputs
- scripts: workspace bootstrap and pipeline runners
- tests: smoke tests

## 4. Quick start (MATLAB)

### 4.1 Pipeline only (generate trajectory)

Run in shell:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
bash scripts/run_pipeline_matlab.sh
```

Run inside MATLAB:

```matlab
cd('/home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws')
AutoLandingMain('pipeline')
```

Pipeline outputs:

- data/processed/landing_trajectory_matlab.csv
- data/processed/landing_trajectory_matlab.json
- data/processed/landing_validation_matlab.json
- data/processed/validation_report_matlab.json

Additional ArUco hover-start output (when running `AutoLandingMainFull`):

- data/processed/landing_trajectory_aruco_hover.csv
- data/processed/landing_trajectory_aruco_hover.json

Model-specific outputs (when available):

- data/processed/landing_trajectory_ontology_ai.csv
- data/processed/landing_trajectory_ontology_ai.json
- data/processed/landing_trajectory_pure_ai.csv
- data/processed/landing_trajectory_pure_ai.json

Fallback output (when both models are unavailable):

- data/processed/landing_trajectory_baseline_tracking.csv
- data/processed/landing_trajectory_baseline_tracking.json

Quantitative trajectory validation report:

- data/processed/landing_trajectory_model_validation.json

ArUco setup tutorial and ROS2 bridge commands:

- docs/ARUCO_LANDING_PAD_SETUP.md

### 4.2 Validation helper

```bash
bash scripts/run_validation_matlab.sh
```

### 4.3 Raw sensor data collection

Collect raw sensor data (position, velocity, attitude, gyro, accel, battery, thrust) **without** ontology processing or AI fusion. Used for offline learning pipelines.

**Prerequisites:**
- Gazebo Sim running: `gz sim -s iris_runway.sdf`
- ArduPilot SITL running

**Single collection session (120 sec at 50 Hz):**

```matlab
AutoLandingMain('collect')
```

Outputs:
- data/collected/[session_id]/raw_data.mat
- data/collected/[session_id]/raw_data.csv
- data/collected/[session_id]/metadata.json

**Parallel multi-worker collection (4 workers, 10 scenarios per worker):**

```matlab
AutoLandingMain('collect_parallel', 4, 10)
```

Outputs:
- data/collected/parallel/[session_id]/worker_1/scenario_1/raw_data.mat
- data/collected/parallel/[session_id]/worker_2/scenario_2/raw_data.mat
- ...
- data/collected/parallel/[session_id]/session_metadata.json

Note: Parallel mode collects raw data WITHOUT ontology or semantic processing (suitable for pre-processing pipelines).

### 4.4 Autonomous mission with real-time visualization

Real-time 3D trajectory monitor + telemetry dashboard.

**Prerequisites:**
- Gazebo Sim running: `gz sim -s iris_runway.sdf`
- ArduPilot SITL running with valid defaults path (example in Section 7.1)
- MAVProxy available: `mavproxy.py`

**Launch mission with real-time viz:**

```bash
bash scripts/run_mission_matlab.sh
```

Or inside MATLAB:

```matlab
AutoLandingMain('mission')
```

Visualization shows (real-time updated):
- 3D drone trajectory
- Distance to landing target
- Altitude profile
- Vertical velocity
- Battery voltage
- Motor thrust percentage
- Mission phase progress (ARM → TAKEOFF → TRAJECTORY → LAND → DISARM)
- Current vehicle state (position, velocity, mode, GPS status)

Mission execution flow:
1. Arm vehicle
2. Takeoff to 2m altitude
3. Follow MATLAB-generated trajectory waypoints
4. Land
5. Disarm

Mission outputs:
- data/processed/mission_result_matlab.json
- data/processed/mission_log.txt
- data/processed/mission_telemetry_log.mat

### 4.5 Full simulation mode

Full end-to-end simulation (pipeline → mission → telemetry):

```bash
bash scripts/run_sim_matlab.sh
```

Or inside MATLAB:

```matlab
AutoLandingMain('sim')
```

This mode:
- Generates trajectory from semantic features
- Executes autonomous mission with vehicle control
- Logs all telemetry and results

### 4.6 Graceful interrupt and cleanup

**User Interrupt Handling (Ctrl+C)**

All modes support safe interruption:

```
AutoLandingMain('collect')               # Press Ctrl+C anytime
AutoLandingMain('collect_parallel', 4)   # Safe shutdown with data recovery
AutoLandingMain('mission')                # Immediate cleanup
```

**What happens on interrupt:**
1. Current data is immediately saved (streaming save)
2. All background processes killed (gz sim, SITL, MAVProxy)
3. Partial collection results recovered
4. Parallel pool terminated gracefully
5. Session metadata saved with "interrupted" status

**Example interrupt:**
```matlab
% Terminal 1: Start collection
>> AutoLandingMain('collect_parallel', 4, 20)
[AutoLandingDataParallel] Session ID: 20260331_120000
[AutoLandingDataParallel] Output: /path/to/data/collected/parallel/20260331_120000

% Terminal 2: After 30 seconds, press Ctrl+C
^C

% Output shows:
[AutoLandingDataCollection] User interrupted. Saving collected data...
[AutoLandingDataCollection] Partial data saved (1234 samples)
[AutoLandingDataParallel.cleanup] Killing gazebo and ArduPilot processes...
[AutoLandingDataParallel.cleanup] Cleanup complete.

% Partial data is available in: data/collected/parallel/20260331_120000/worker_1/scenario_1/raw_data.mat
```

**Data Recovery Features:**
- Real-time checkpoint saves every 0.5 seconds (collection)
- Partial data preserved even on force-quit (Ctrl+C)
- Crash-safe log files in `data/logs/parallel/[session_id]/`
- Session metadata marks collection as "interrupted" (not "error")

### 4.3 Autonomous mission execution

**Prerequisites:**
- Gazebo Sim running: `gz sim -s iris_runway.sdf`
- ArduPilot SITL running with valid defaults path (example in Section 7.1)
- MAVProxy available: `mavproxy.py`

**Launch mission:**

```bash
bash scripts/run_mission_matlab.sh
```

Or inside MATLAB:

```matlab
AutoLandingMain('mission')
```

Mission execution flow:
1. Arm vehicle
2. Takeoff to 2m altitude
3. Follow MATLAB-generated trajectory waypoints
4. Land
5. Disarm

Mission outputs:
- data/processed/mission_result_matlab.json
- data/processed/mission_log.txt
- data/processed/mission_telemetry_log.mat

### 4.4 Full simulation mode

Full end-to-end simulation (pipeline → mission → telemetry):

```bash
bash scripts/run_sim_matlab.sh
```

Or inside MATLAB:

```matlab
AutoLandingMain('sim')
```

This mode:
- Generates trajectory from semantic features
- Executes autonomous mission with vehicle control
- Logs all telemetry and results

### 4.7 Complete Pipeline: Parallel Collection + Training + Validation + Comparison (NEW)

**Purpose**: Automated end-to-end workflow (recommended default mode)

Full pipeline that executes in sequence:
1. **Parallel data collection** (raw sensor data, no ontology)
2. **Feature extraction & normalization** (14-feature vectors)
3. **Hybrid model training** (Ontology+AI with neural network)
4. **Pure AI baseline training** (Decision tree classifier)
5. **Model comparison and evaluation** (accuracy, precision, recall, F1)
6. **Visualization** (real-time plots during collection, comparison figures)

**Launch:**

```matlab
AutoLandingMainFull()
```

**Configuration Parameters (edit at top of AutoLandingMainFull.m):**

```matlab
% Simulation Settings
gazebo_server_mode = true;          % true = Server (headless), false = GUI
enable_visualization = true;        % true = Real-time plots/monitoring
ros_domain_id = '0';                % Unified default ROS domain for MATLAB/workers/RViz

% Data Collection Settings
num_workers = 4;                    % Number of parallel workers
scenarios_per_worker = 10;          % Scenarios per worker (total = 40)
drone_spawn_distance_m = 0.8;       % Drone distance from landing pad center
drone_spawn_angle_deg = 35.0;      % Drone spawn angle around the landing pad
landing_pad_distance_m = 1.15;     % Landing pad offset distance from each worker drone base spawn

% Train/Validation Split
total_training_samples = 100;       % Total samples for training
train_ratio = 0.8;                  % 80% train, 20% validation

% Model Configuration
use_ontology_model = true;          % Train Ontology+AI hybrid model
use_pure_ai_model = true;           % Train Pure AI baseline model

% Visualization
enable_plots = true;                % Generate comparison plots
```

**Gazebo Mode Selection:**

- **Server Mode (headless, default)**: `gazebo_server_mode = true`
  ```
  [AutoLandingDataCollection] Gazebo Mode: Server (Headless)
  ```
  Use for:
  - Remote/CI pipelines
  - Faster execution (no X11)
  - Parallel workers (avoid display conflicts)

- **GUI Mode (interactive)**: `gazebo_server_mode = false`
  ```
  [AutoLandingDataCollection] Gazebo Mode: GUI
  ```
  Use for:
  - Visual debugging
  - Monitoring simulation in real-time
  - Single worker scenarios

**Real-time Visualization Features:**

When `enable_visualization = true`, each worker displays:
- **3D Trajectory**: Real-time drone path
- **Altitude Profile**: Current and max altitude
- **Velocity Norm**: Speed magnitude
- **Battery Voltage**: Power monitoring

Visualization saves automatically to: `data/collected/parallel/[session_id]/worker_*/scenario_*/realtime_collection_viz.png`

**Execution Flow:**

```
[Pipeline] STEP 1: Parallel Data Collection
  └─ Worker 1: scenario 1-10 (parallel)
  └─ Worker 2: scenario 1-10 (parallel)
  └─ Worker 3: scenario 1-10 (parallel)
  └─ Worker 4: scenario 1-10 (parallel)
    Total: 40 scenarios, raw sensor data only

[Pipeline] STEP 2: Load and Prepare Data
  └─ Extract 14 features per scenario
  └─ Normalize with z-score
  └─ Split 80/20 train/validation

[Pipeline] STEP 3A: Training Ontology+AI Hybrid Model
  └─ Neural network (10-5-1 architecture)
  └─ Validation Accuracy: 0.8250

[Pipeline] STEP 3B: Training Pure AI Baseline Model
  └─ Decision tree (max 10 splits)
  └─ Validation Accuracy: 0.7840

[Pipeline] STEP 4: Model Comparison and Analysis
  └─ Accuracy gap: +4.10%
  └─ WINNER: Ontology+AI (Hybrid)

[Pipeline] STEP 5: Generating Plots
  └─ Confusion matrices
  └─ Accuracy comparison bars
  └─ Precision/Recall/F1 comparison
  └─ Saved: data/plots/model_comparison.png
```

**Output Files:**

```
data/
├── collected/parallel/[session_id]/
│   ├── worker_1/scenario_1/
│   │   ├── raw_data.mat
│   │   ├── raw_data.csv
│   │   ├── metadata.json
│   │   └── realtime_collection_viz.png
│   ├── worker_2/...
│   └── session_metadata.json
├── models/
│   ├── model_hybrid_ontology_ai.mat
│   └── model_pure_ai.mat
├── plots/
│   ├── model_comparison.fig
│   └── model_comparison.png
└── analysis/
    └── comparison_analysis_[timestamp].json
```

**Example: Server Mode (Default)**

```matlab
% Terminal: Start Gazebo and ArduPilot first (optional - auto-managed)
>> cd ~/ardupilot
>> ./build/sitl/bin/arducopter --model JSON --speedup 1 ...

% Terminal: MATLAB
>> AutoLandingMainFull()

[Pipeline] STEP 1: Parallel Data Collection
[AutoLandingDataCollection] Gazebo Mode: Server (Headless)
[AutoLandingDataCollection] Real-time Visualization: ENABLED
[AutoLandingDataParallel] 4 workers collecting 40 scenarios...
[Worker 1] Collecting scenario 1/10... [3D trajectory plot updates...]
[Worker 2] Collecting scenario 1/10...
[Worker 3] Collecting scenario 1/10...
[Worker 4] Collecting scenario 1/10...

[Pipeline] STEP 2: Load and Prepare Data
[DataLoader] Found 40 scenarios
[DataLoader] Loaded training data: 32 samples
[DataLoader] Loaded validation data: 8 samples

[Pipeline] STEP 3A: Training Ontology+AI Hybrid Model
[HybridModel] Training neural network (3 layers)...
[HybridModel] Training Accuracy: 0.8750
[HybridModel] Validation Accuracy: 0.8250

[Pipeline] STEP 3B: Training Pure AI Baseline Model
[PureAI] Training decision tree (no semantic features)...
[PureAI] Training Accuracy: 0.8125
[PureAI] Validation Accuracy: 0.7840

[Pipeline] STEP 4: Model Comparison and Analysis
============================================================
MODEL COMPARISON RESULTS
============================================================

Ontology+AI (Hybrid):
  Accuracy:  0.8250
  Precision: 0.8571
  Recall:    0.7500
  F1-Score:  0.8000
  TP: 3, FP: 0, FN: 1, TN: 4

Pure AI (Baseline):
  Accuracy:  0.7840
  Precision: 0.7143
  Recall:    0.7500
  F1-Score:  0.7317
  TP: 3, FP: 1, FN: 1, TN: 3

------------------------------------------------------------
WINNER: Ontology+AI (Hybrid) (+4.10% improvement)
============================================================

[Pipeline] STEP 5: Generating Plots
[Plots] Generating comparison visualizations...
[Plots] Saved: .../data/plots/model_comparison.fig
[Plots] Saved: .../data/plots/model_comparison.png

Pipeline Complete!
OUTPUT FILES:
  - Raw Data: /path/to/data/collected/parallel/20260331_143022/
  - Models: /path/to/data/models/
  - Plots: /path/to/data/plots/
  - Analysis: /path/to/data/analysis/
```

**Example: GUI Mode (Visual Debugging)**

```matlab
% Edit AutoLandingMainFull.m
gazebo_server_mode = false;    % Enable GUI
enable_visualization = true;   % Show real-time plots

>> AutoLandingMainFull()

% Gazebo window opens with iris_runway.sdf
% ArduPilot SITL starts with 3D visuals
% MATLAB figure shows 4 real-time subplots per worker
% No port conflicts (sequential fallback if parpool unavailable)
```

## 5. Python legacy

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
bash scripts/run_pipeline.sh
```

Pipeline outputs:

- data/processed/landing_trajectory.csv
- data/processed/landing_trajectory.json

## 6. MATLAB module architecture

### 6.1 Module categories

Functions are organized by category under `matlab/modules/core/`:

- **config**: Configuration management (autlDefaultConfig)
- **io**: File I/O (autlLoadJson, autlSaveJson, autlWriteTrajectoryCsv)
- **ontology**: Semantic feature extraction (autlExtractSemanticFeatures)
- **ai**: Machine learning pipeline (autlBuildAiFeatureVector, autlLoadAiModel, autlPredictAiConfidence, autlTrainAiModel, autlFuseConfidence)
- **trajectory**: Trajectory generation (autlGenerateTrajectory)
- **metrics**: Evaluation metrics (autlEvaluateDecisionMetrics, autlEvaluateTrajectoryMetrics)
- **validation**: Validation pipeline (autlRunValidation)
- **ros_io**: ROS 2 integration stubs (autlCreateRosContext, autlReadRosObservation, autlPublishTrajectory, autlReleaseRosContext)
- **utils**: Utilities (autlClamp)
- **pipeline**: Pipeline orchestration (autlRunPipeline)
- **mavproxy**: Vehicle control via MAVProxy (autlMavproxyControl) ← NEW
- **mission**: Autonomous mission management (autlAutonomousMission) ← NEW

### 6.2 Function-per-folder policy

Each function has its own folder containing a single .m file:

```
matlab/modules/core/
├── ai/
│   ├── autlBuildAiFeatureVector/
│   │   └── autlBuildAiFeatureVector.m
│   ├── autlLoadAiModel/
│   │   └── autlLoadAiModel.m
│   └── ...
├── mavproxy/
│   └── autlMavproxyControl/
│       └── autlMavproxyControl.m
├── mission/
│   └── autlAutonomousMission/
│       └── autlAutonomousMission.m
└── ...
```

### 6.3 New mission management modules (Options 2 & 3)

#### autlMavproxyControl

**Purpose**: Interface with ArduPilot SITL via MAVProxy

**Inputs**:
- `action`: 'arm', 'disarm', 'takeoff', 'land', 'set_velocity', 'set_mode', 'status'
- `params`: struct with action-specific parameters
- `cfg`: config struct with MAVProxy connection details

**Actions**:
- `arm`: Arm the motors
- `disarm`: Disarm the motors
- `takeoff(height)`: Takeoff to specified height in meters
- `land`: Switch to LAND mode
- `set_velocity(vx, vy, vz)`: Send velocity command (requires GUIDED mode)
- `set_mode(mode)`: Set flight mode (STABILIZE, GUIDED, LAND, etc.)
- `status`: Query vehicle status and telemetry

**Example**:
```matlab
cfg = autlDefaultConfig();
result = autlMavproxyControl('arm', struct(), cfg);
result = autlMavproxyControl('takeoff', struct('height', 2.0), cfg);
result = autlMavproxyControl('status', struct(), cfg);
```

#### autlAutonomousMission

**Purpose**: Execute autonomous landing mission with full control flow

**Inputs**:
- `trajectory_table`: Table with columns (t, x, y, z, vx_cmd, vy_cmd, vz_cmd, fused_confidence, ...)
- `initial_state`: struct with x0, y0, z0, yaw0
- `target_state`: struct with x_target, y_target, z_target
- `cfg`: config struct with mission parameters
- `root_dir`: workspace root for logging

**Mission execution phases**:
1. Arm vehicle
2. Takeoff to cfg.mission.takeoff_height (default 2m)
3. Follow trajectory waypoints (with velocity scaling)
4. Land
5. Disarm

**Configuration options**:
- `cfg.mission.takeoff_height`: Takeoff altitude (default 2.0m)
- `cfg.mission.trajectory_tracking_enabled`: Enable trajectory following (default true)
- `cfg.mission.use_ros_control`: Use ROS2 instead of MAVProxy (default false)
- `cfg.mission.mission_timeout`: Maximum mission duration (default 300s)
- `cfg.mission.velocity_scale_factor`: Scale velocity commands (default 1.0)

**Outputs**:
- `mission_result.mission_status`: 'completed', 'arm_failed', 'takeoff_failed', 'land_failed', 'error', etc.
- `mission_result.phases`: Detailed status for each mission phase
- `mission_result.telemetry_log`: Telemetry records for each waypoint
- `mission_result.log`: Text log of mission execution
- `mission_result.is_success`: Boolean success indicator

**Example**:
```matlab
% Extract from pipeline output
trajectory = summary.landing_trajectory;
initial = struct('x0', 0, 'y0', 0, 'z0', 0, 'yaw0', 0);
target = struct('x_target', 0, 'y_target', 0, 'z_target', 0);

% Execute mission
cfg = autlDefaultConfig();
result = autlAutonomousMission(trajectory, initial, target, cfg, root_dir);

% Check results
disp(result.mission_status);
disp(result.phases.arm.status);
disp(result.phases.takeoff.status);
disp(result.phases.trajectory_tracking.points_sent);
```

### 6.4 Data collection (raw sensor only, no ontology)

#### autlRunDataCollection

**Purpose**: Collect raw sensor data WITHOUT ontology processing or AI fusion. Collects: position, velocity, attitude, gyro, accel, battery, thrust, GPS, barometer, rangefinder at configurable sample rate.

**Inputs**:
- `root_dir`: Workspace root
- `mission_config`: struct with collection parameters

**Configuration options**:
- `mission_config.max_duration`: Collection duration in seconds (default 120)
- `mission_config.sample_rate`: Sampling frequency in Hz (default 50)
- `mission_config.output_dir`: Output directory path
- `mission_config.session_id`: Unique session identifier

**Outputs**: Raw data files in session directory:
- `raw_data.mat`: MATLAB binary with raw sensor arrays
- `raw_data.csv`: CSV file for inspection
- `metadata.json`: Collection session metadata

**Data fields collected**:
```
timestamp, position_xyz, velocity_xyz, attitude_rpy, attitude_quat,
thrust_percent, rotor_speeds, gps_fix, imu_accel_xyz, imu_gyro_xyz,
battery_voltage, battery_current, barometer_alt, rangefinder_dist,
armed_state, flight_mode
```

**Example**:
```matlab
cfg = struct();
cfg.max_duration = 120;  % 2 minutes
cfg.sample_rate = 50;    % 50 Hz
result = autlRunDataCollection(root_dir, cfg);

disp(result.session_dir);
disp(result.sample_count);
```

### 6.5 Real-time visualization

#### autlVisualizeMissionRealtime

**Purpose**: Display real-time mission progress dashboard (6-panel layout) with 3D trajectory, altitude profile, battery status, and vehicle state.

**Display panels**:
1. **3D Trajectory**: Real-time drone path in 3D space
2. **Phase Progress**: Mission phase completion bars (ARM, TAKEOFF, TRAJECTORY, LAND, DISARM)
3. **Distance to Target**: Euclidean distance vs time
4. **Altitude Profile**: Z position vs time
5. **Velocity Magnitude**: Speed vs time
6. **Battery Voltage**: Supply voltage vs time
7. **Motor Thrust**: Throttle percentage vs time
8. **Status Panel**: Text display of current vehicle state

**Example**:
```matlab
% Launch visualization before or during mission
autlVisualizeMissionRealtime('mission_20260331_120000', 0.5);

% Will display 6-panel tiledlayout with animated line plots
% Updates every 0.5 seconds from MAVProxy telemetry
```

### 6.6 Parallel data collection workers

#### autlDataCollectorWorker

**Purpose**: Single worker process for parallel multi-scenario raw data collection.

**Usage**: Spawned by `AutoLandingDataParallel` entrypoint.

**Configuration**:
- `worker_id`: Unique worker identifier (1 to N)
- `config.scenarios_per_worker`: Number of scenarios to collect per worker
- `config.sample_rate`: Telemetry sample rate (Hz)
- `config.scenario_duration`: Duration per scenario (seconds)

#### AutoLandingDataParallel

**Purpose**: Orchestrate multiple workers collecting raw data in parallel (inspired by IICC26 `AutoSimMain`).

**Usage**:
```matlab
% 4 workers, 10 scenarios per worker (40 total scenarios)
AutoLandingDataParallel(4, 10);

% Default: 4 workers
AutoLandingDataParallel();
```

**Output structure**:
```
data/collected/parallel/[session_id]/
├── worker_1/
│   ├── scenario_1_[timestamp]/raw_data.mat
│   ├── scenario_2_[timestamp]/raw_data.mat
│   └── ...
├── worker_2/
│   ├── scenario_1_[timestamp]/raw_data.mat
│   └── ...
└── session_metadata.json
```

**Features**:
- Multi-MATLAB worker spawning via `parpool` (MATLAB 9.9+)
- Fallback to sequential workers on older MATLAB
- Per-worker logging to `data/logs/parallel/[session_id]/worker_N.log`
- Session-level metadata with collection parameters
- **Key design**: Raw data only (no ontology applied during collection)

## 7. Simulation and vehicle integration

### 7.1 Gazebo Sim + ArduPilot SITL stack

To verify the complete simulation stack is operational, run in separate terminals:

**Terminal 1: Gazebo server (headless)**
```bash
gz sim -s iris_runway.sdf -v4
```

**Terminal 2: ArduPilot SITL**
```bash
cd "$HOME/ardupilot"
./build/sitl/bin/arducopter --model JSON --speedup 1 --slave 0 \
  --defaults "$HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm" \
  --sim-address=127.0.0.1 -I0
```

**Terminal 3: MATLAB mission (after both above are running)**
```bash
bash scripts/run_mission_matlab.sh
```

### 7.2 Integration flow

```
Semantic Context JSON
        ↓
   [MATLAB Pipeline]
   - Extract semantic features (ontology)
   - Fuse with AI model (confidence)
   - Generate trajectory
     ↓
   [Trajectory Table]
   - Position: (x, y, z)
   - Commands: (vx_cmd, vy_cmd, vz_cmd, fused_confidence)
   - 80+ waypoints
     ↓
   [Autonomous Mission]
   - ARM vehicle
   - TAKEOFF to 2m
   - FOLLOW trajectory waypoints
   - LAND
   - DISARM
     ↓
   [MAVProxy Bridge]
   - TCP → tcp:127.0.0.1:5760
   - Commands: arm, takeoff, velocity, land, disarm
     ↓
   [ArduPilot SITL]
   - Parse commands
   - Run flight dynamics
   - Update simulated telemetry
     ↓
   [Gazebo Sim]
   - Render iris_with_gimbal model
   - Apply rotor forces
   - Update physics
```

### 7.3 Output files

After mission execution, check:

```
autolanding_ws/data/processed/
├── landing_trajectory_matlab.json       # Trajectory waypoints
├── landing_trajectory_matlab.csv        # Same in CSV format
├── landing_validation_matlab.json       # Validation metrics
├── validation_report_matlab.json        # Full validation report
├── mission_result_matlab.json           # Mission execution result
├── mission_log.txt                      # Text log of mission phases
└── mission_telemetry_log.mat            # MATLAB structure with telemetry
```

### 7.4 RViz observability and multi-drone parallel collection

Use these utilities to visualize camera/ArUco/drone state and run meaningful parallel collection with worker-specific states.

```bash
# Terminal 1: Gazebo world
gz sim -v4 -r iris_runway.sdf

# Terminal 2: Multi-SITL instances (I0..I2 by default)
bash scripts/launch_multi_drone_sitl.sh 3

# Terminal 3: RViz + bridge + aruco detector (default domain 0)
bash scripts/launch_rviz_monitor.sh --domain 0

# Terminal 4: MATLAB full pipeline
bash scripts/run_pipeline_matlab.sh
```

Notes:

- `scripts/launch_rviz_monitor.sh` sources local overlays if present (`/opt/ros/humble`, `~/gz_ros2_aruco_ws`, `~/SynologyDrive/INCSL/devel/INCSL/IICC26_ws`) and exports `ROS_DOMAIN_ID=0` by default.
- `AutoLandingMainFull` now builds worker-specific profiles (`worker_profiles`) with different MAVLink ports, spawn offsets, and motion profiles (`aggressive`, `conservative`, `orbit-heavy`, `balanced`).
- Worker MAVLink mapping follows ArduPilot instance offsets: worker `i` uses `tcp:127.0.0.1:(5762 + 10*(i-1))` with fallback `5760 + 10*(i-1)`.
- RViz monitor launches a MAVLink-to-ROS odometry publisher and exposes namespaced topics: `/drone1/odom`, `/drone2/odom`, ...
- World generation is now multi-drone aware: generated world includes one iris model per worker using profile-based spawn/yaw/model-name values.
- RViz preset includes per-drone color odometry, path trails (`/droneN/path`), and per-drone camera image displays (`/droneN/camera`).
- Bridge launcher now starts per-worker camera bridges automatically for `/droneN/camera` topics.
- Drone spawns are placed on a fixed-spacing grid (`3.0 m`) to prevent overlap during multi-worker runs.
- ArUco markers are spawned per drone near each spawn point using `landing_pad_distance_m`, and observer cameras are positioned to face each marker to reduce cross-camera overlap.

## 8. Next integration targets

- Replace heuristic semantic encoder with GNN/transformer encoder
- Learn policy weights from trajectory dataset
- Add online replanning with ontology state updates
- Connect ROS 2 bridge topics for real-time closed loop landing
- Implement direct velocity control via mavros (not just MAVProxy)
