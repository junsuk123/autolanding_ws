# autolanding_ws

MATLAB-first semantic autonomous landing workspace.

The primary orchestration layer is MATLAB for:
- pipeline execution
- simulation and mission control
- data collection and model training
- validation and paper-style plot generation

Python remains available for auxiliary visualization and research utilities, but it is no longer the default launcher path.

## 1. Research Formulation

Semantic context graph:

G = (V, E, A)

- V: objects (drone, landing pad, marker, wind, obstacle)
- E: relations (near, aligned_with, visible_to, affected_by)
- A: attributes (wind, uncertainty, marker confidence, state stability)

Semantic embedding and fusion:

z_sem = Pool(phi(v_i, a_i), psi(e_ij))

x_fused = [x_sensor ; z_sem]

Policy and dynamics:

pi_theta(x_fused) -> u_t

s_(t+1) = f(s_t, u_t)

Optimization objective used for analysis and reporting:

J = sum_t (w_p ||p_t - p_ref||^2 + w_v ||v_t||^2 + w_u ||u_t||^2) + w_r R_sem

Landing-specific objective:

J_land = sum_t (w_xy ||p_xy,t - p_m,xy||^2 + w_z (z_t - z_m)^2 + w_v ||v_t||^2)

## 1.1 통합 연구/실행 정리 (2026-04-07)

아래 항목은 본 워크스페이스의 기본 운영 정책입니다.

1) 카메라는 하향(지면) 시야를 유지하며 ArUco 마커를 검출합니다.
2) 마커 위치, 검출 빈도, 연속 검출 안정도를 함께 수집하여 의미 기반 피처를 구성합니다.
3) 마커 위치는 착륙 목표점으로 해석하며, 누적 평균으로 절대좌표를 고정합니다. 1회 이상 검출 후 일시 미검출이면 최근 누적 평균을 추정값으로 사용합니다.
4) 제안모델은 온톨로지 기반 의미 피처 + 센서 피처를 AI 입력으로 사용해 착륙 궤적을 생성합니다.
5) 평가 시 제안모델(온톨로지+AI)과 단일 AI(온톨로지 없음)를 동일 조건에서 비교합니다.
6) 의미 피처는 해석 가능성을 위해 물리량/기하량을 반영한 수식 기반 함수로 설계합니다.
7) 시뮬레이션 수집은 n회 반복하며, 날짜/시간 기반 폴더에 센서·GT·착륙 경로를 저장합니다. 기본 분할은 학습:검증 = 8:2이며 설정값으로 조정 가능합니다.
8) 수집 종료 후 제안모델과 단일모델 학습을 순차 실행합니다.
9) 학습 산출물은 날짜/시간/시나리오 정보가 포함된 폴더명으로 저장합니다.
10) 검증은 기본적으로 최신 모델을 사용하며, 설정 파일로 특정 모델을 고정 지정할 수 있습니다.
11) 검증 완료 후 논문용 플롯을 자동 생성합니다.
12) 전체 과정은 단일 실행 스크립트로 자동 수행합니다.
13) 설정은 단일 파일([ai/configs/orchestration_config.yaml](ai/configs/orchestration_config.yaml)) 기준으로 관리하고, 기능은 모듈화된 패키지 단위로 유지합니다.
14) 시각화는 RViz2와 Python 모니터 도구를 병행 사용합니다.

### 네임스페이스 통합 규칙

- 드론 토픽 기준 네임스페이스 접두사: `/autolanding/drone`
- 기본 1번 드론 네임스페이스: `/autolanding/drone1`
- MAVROS 접두사: `/autolanding/mavros_w`
- 단일 소스 설정 파일: [ai/configs/orchestration_config.yaml](ai/configs/orchestration_config.yaml)

토픽 파생 기본 규칙:

- `drone_namespace = drone_ns_prefix + primary_drone_index`
- `camera_image_topic = drone_namespace + /camera`
- `camera_info_topic = drone_namespace + /camera_info`
- `aruco_markers_topic = drone_namespace + /aruco_markers`
- `aruco_poses_topic = drone_namespace + /aruco_poses`

필요 시 위 개별 토픽 키만 별도로 override할 수 있습니다.

### Multi-UAV Scalable Protocol (N <= 10)

This repository now includes a fully isolated `/uavX` architecture for Gazebo + SITL + MAVROS + MATLAB.

Isolation rules per UAV index `X`:

- Namespace: `/uavX/mavros/*`
- SYSID: `SYSID_THISMAV = X`
- SITL MAVLink UDP out: `14539 + X` (UAV1 -> 14540, UAV2 -> 14541, ...)
- MAVROS bind UDP: `14639 + X`
- MAVROS FCU URL: `udp://127.0.0.1:(14539+X)@(14639+X)`

Communication bandwidth planning (recommended):

- State stream rate: 10 Hz
- Control stream rate: 20 Hz
- Approximate aggregate message rate for `N` UAVs:
	`R_total ~= N * (R_state + R_control)`

Launch artifacts:

- `scripts/start_multi_uav_sitl.py`
- `scripts/start_multi_uav_mavros.py`
- `simulation/launch/multi_uav_mavros.launch.py`
- `scripts/start_multi_uav_stack.sh`
- MATLAB demo: `matlab/scripts/run_multi_uav_mavros_demo.m`
- Detailed design: `docs/MULTI_UAV_SYSTEM_DESIGN.md`

Quick start (3 UAVs):

```bash
cd autolanding_ws
bash scripts/start_multi_uav_stack.sh 3
```

Quick checks:

```bash
ros2 topic list | grep '^/uav[1-3]/mavros/state$'
ros2 service list | grep '/uav[1-3]/mavros/cmd/arming'
```

MATLAB multi-UAV control example:

```matlab
addpath(genpath('matlab'));
run_multi_uav_mavros_demo(3);
```

### 2.1 Simulation Recovery: Gazebo Soft Reset

**Drone Hover Instability Auto-Recovery:**

When drone hover is unstable during data collection, the system automatically performs a Gazebo soft reset:

```matlab
% Automatic in autlRunDataCollection:
% - Detects hover stabilization timeout (20s)
% - Calls Gazebo WorldControl reset service
% - Logs reset status
% - Waits 1.5s for physics stabilization
```

**Manual Gazebo soft reset from MATLAB:**

```matlab
% One-time reset
[ok, msg] = autlResetGazeboSimulation('default', '[MyScript] ');
fprintf('Reset: %s\n', msg);

% With world name
[ok, msg] = autlResetGazeboSimulation('myworld', '[Prefix] ');
```

**Manual reset via command line:**

```bash
# WorldControl method (Gazebo v8+)
gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'

# Or via ROS2
ros2 service call /world/default/control gz_msgs/srv/WorldControl '{reset: {all: true}}'

# Alternative reset services
ros2 service call /gazebo/reset_simulation std_srvs/srv/Empty '{}'
ros2 service call /reset_world std_srvs/srv/Empty '{}'
```

**Gazebo GUI troubleshooting:**

- Force GUI mode: `AUTOLANDING_FORCE_GUI=1 AUTOLANDING_FORCE_HEADLESS=0 bash simulation/launch/start_gz_ardupilot.sh`
- Force a specific display (if :0 is not your desktop): `AUTOLANDING_DISPLAY=:1 AUTOLANDING_FORCE_GUI=1 AUTOLANDING_FORCE_HEADLESS=0 bash simulation/launch/start_gz_ardupilot.sh`
- If OpenGL init fails (`libEGL ... dri2`), force software rendering: `AUTOLANDING_GUI_SOFTWARE=1 AUTOLANDING_FORCE_GUI=1 AUTOLANDING_FORCE_HEADLESS=0 bash simulation/launch/start_gz_ardupilot.sh`
- If DISPLAY is empty: `export DISPLAY=:0`
- If Xauthority is missing: `export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority` (or `~/.Xauthority`)

## 2. MATLAB Orchestrator

Main entrypoint:
- matlab/AutoLandingMainFull.m

Supported modes:
- full
- pipeline
- validation
- mission
- sim
- collect
- collect_parallel

Examples:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws

matlab -batch "run('matlab/scripts/run_autolanding_pipeline.m')"
matlab -batch "run('matlab/scripts/run_autolanding_validation.m')"
matlab -batch "run('matlab/run_autolanding_mission.m')"
matlab -batch "run('matlab/run_autolanding_sim.m')"
```

### 2.x 연구 벤치마크 (제안모델 vs 베이스라인)

아래 명령은 MATLAB full pipeline(수집 + 학습 + 검증 + 플롯)을 한 번에 수행합니다.

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
matlab -batch "run('matlab/scripts/run_autolanding_pipeline.m')"
```

산출물:
- data/processed/landing_trajectory_matlab.json
- data/processed/landing_trajectory_matlab.csv
- data/processed/validation_report_matlab.json
- data/processed/workspace_summary_<timestamp>.json
- data/models/<timestamp>_scenarios_<N>/training_summary.json

`training_summary.json`에는 다음이 포함됩니다.
- 제안모델(온톨로지+AI) 검증 지표
- 베이스라인(AI-only) 검증 지표
- train/validation 분할 정보
- 모델 저장 경로 및 비교 요약

### 2.0 시뮬레이션 시작: Gazebo + ArduPilot

#### 가제보 중복 생성 방지 기능

가제보 프로세스 중복 생성을 자동으로 방지합니다. 동시에 여러 가제보 인스턴스가 시작되는 것을 막습니다.

**사용 명령어:**

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws

# 1) 기본 가제보 시작 (중복 체크 활성화)
#    - 가제보가 이미 실행 중이면 오류 메시지 표시
./simulation/launch/start_gz_ardupilot.sh

# 2) 기존 가제보 종료 후 새로 시작
#    - 실행 중인 모든 Gazebo(gz sim) 프로세스를 종료하고 새 인스턴스 시작
./simulation/launch/start_gz_ardupilot.sh --kill-existing

# 3) 중복 검사 무시 (권장하지 않음)
#    - 기존 가제보가 실행 중이어도 무시하고 새로 시작
./simulation/launch/start_gz_ardupilot.sh --no-duplicate-check

# 4) 드라이런 모드 (실제 시작 안 함)
#    - 명령어를 출력만 하고 실행하지 않음 (테스트용)
./simulation/launch/start_gz_ardupilot.sh --dry-run

# 5) 헬프 보기
./simulation/launch/start_gz_ardupilot.sh --help
```

**중요 (경로 규칙):**
- `./simulation/launch/start_gz_ardupilot.sh` 는 반드시 `autolanding_ws` 디렉터리에서 실행해야 합니다.
- 상위 루트(`.../INCSL`)에서 상대경로로 실행하면 `No such file or directory`(exit 127)가 발생할 수 있습니다.
- 루트에서 실행해야 할 경우 절대경로를 사용하세요.

```bash
/home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/simulation/launch/start_gz_ardupilot.sh --kill-existing
```

### 2.0.1 포그라운드 검증 명령어 (실사용)

아래 순서대로 실행하면 포그라운드에서 검증할 수 있습니다.

```bash
# 0) 작업 디렉터리 이동 + 가상환경 활성화
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
source .venv/bin/activate

# 1) 시작 스크립트 포그라운드 실행 (실제 스택 기동)
/home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/simulation/launch/start_gz_ardupilot.sh --kill-existing

# 2) 장시간 블로킹 없이 실행 경로만 검증 (포그라운드 드라이런)
/home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/simulation/launch/start_gz_ardupilot.sh --kill-existing --dry-run --no-duplicate-check

# 3) 파이프라인 완료형 검증 (산출물 생성 확인)
matlab -batch "run('matlab/scripts/run_autolanding_pipeline.m')"
```

검증 완료 후 확인할 출력 파일:
- `data/processed/landing_trajectory_matlab.json`
- `data/processed/landing_trajectory_matlab.csv`
- `data/processed/validation_report_matlab.json`

**옵션 조합 예제:**

```bash
# 다른 예제 모드로 시작 (기본값: iris_runway)
./simulation/launch/start_gz_ardupilot.sh --example iris_maze

# 기존 프로세스 종료 후 다른 예제로 시작
./simulation/launch/start_gz_ardupilot.sh --kill-existing --example iris_maze

# 로컬 폴백 런처(Python) 사용
./simulation/launch/start_gz_ardupilot.sh --local-fallback
```

**기능:**
- ✅ **프로세스 감지**: 실행 중인 Gazebo(`gz sim`) 자동 감지
- ✅ **배타적 락**: PID 기반 락 파일로 동시 실행 방지
- ✅ **안전한 종료**: 정상 종료 후 강제 종료까지 지원
- ✅ **자동 정리**: 스크립트 종료 시 락 파일 자동 정리

**트러블슈팅:**

```bash
# 가제보 프로세스 확인
pgrep -f "gz sim"

# 실행 중인 가제보 수동 종료
pkill -f "gz sim"

# 또는 포스 킬
pkill -9 -f "gz sim"

# 락 파일 위치 확인 및 제거
ls -la $XDG_RUNTIME_DIR/gazebo_ardupilot_locks/
rm -f $XDG_RUNTIME_DIR/gazebo_ardupilot_locks/gazebo_launch.lock
```

---

### 2.1 하나만 실행해서 전체 파이프라인 돌리기

아래 명령 1개로 전체 파이프라인(full 모드)을 실행합니다.

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
bash scripts/run_full_pipeline.sh
```

루트에서 바로 실행하려면 아래 래퍼 명령도 동일하게 사용할 수 있습니다.

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL
bash scripts/run_full_pipeline.sh
```

기본 동작은 완료 후 Gazebo/SITL를 자동 정리(cleanup)합니다. 시뮬레이션을 유지하려면 `--keep-sim` 옵션을 사용하세요.

```bash
bash scripts/run_full_pipeline.sh --keep-sim
```

이 명령은 내부적으로 다음을 수행합니다.
- Gazebo/SITL/RViz 런처 실행
- 파이프라인 실행
- 이륙 데모 후 궤적 추종 기반 착륙 실행 (`auto_landing_execute: true`일 때)
- 검증 JSON 생성
- 논문용 플롯 생성

착륙 실행 시 목표점 선택 우선순위:
- `semantic_input.aruco.marker_center` (ArUco 기반)
- 없으면 `semantic_input.target`

착륙 제어기 선택:
- `landing_controller: auto` -> 학습 모델(`data/models/model_hybrid_ontology_ai.mat`)이 있으면 `model`, 없으면 `pid`
- `landing_controller: model|pid|png` 강제 지정 가능

착륙 데이터 수집 산출물:
- `data/collected/<timestamp>/worker_<n>/session_<timestamp>/raw_data.mat`

변경 가능한 실행 파라미터 파일:
- ai/configs/orchestration_config.yaml

수집 시나리오 기본값:
- `scenarios_per_worker: 10` (설정 파일에서 바로 수정 가능)

파일에 주석으로 각 파라미터 의미를 설명해두었습니다.

### 2.2 빠른 실행 순서 (권장)

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
matlab -batch "run('matlab/scripts/run_autolanding_pipeline.m')"
```

### 2.3 "Configuring a Python Environment"에서 멈춘 것처럼 보일 때

원인 요약:
- VS Code의 Python 환경 구성 단계는 내부적으로 사용자 선택(인터프리터/환경)을 기다리기 때문에, 화면상으로는 멈춘 것처럼 보일 수 있습니다.
- 실제 실행 멈춤과 별개로, 런처 import 경로 문제(ModuleNotFoundError)가 있으면 즉시 실패하며 진행이 멈춘 것처럼 느껴질 수 있습니다.

우회 방법:
- VS Code 자동 구성 없이 위 "빠른 실행 순서"를 그대로 사용하면 즉시 실행 가능합니다.
- 인터프리터는 .venv/bin/python을 직접 선택해 사용하면 안정적입니다.

진단 명령:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
which matlab
matlab -batch "disp('deps_ok')"
```

플롯 단계가 환경 문제로 실패하면 임시로 플롯을 끄고 실행할 수 있습니다.

```bash
matlab -batch "run('matlab/scripts/run_autolanding_validation.m')"
```

## 3. MATLAB Compatibility

The MATLAB entrypoints are now the primary execution path:
- matlab/AutoLandingMain.m
- matlab/AutoLandingMainFull.m
- matlab/AutoLandingDataParallel.m

Legacy Python launcher scripts were removed, and the runtime path is MATLAB-first.

## 4. Paper Plot Outputs

MATLAB comparison plots are generated during the training/validation stage and copied into the paper plot directory:
- data/plots/paper/paper_model_comparison.png
- data/plots/paper/paper_model_comparison.fig

Additional MATLAB outputs:
- data/plots/model_comparison.png
- data/plots/model_comparison.fig

Validation output:
- data/processed/validation_report_matlab.json

Collection orchestration summary output:
- data/processed/workspace_summary_<timestamp>.json

## 5. Scripts

Helper scripts call MATLAB batch entrypoints:
- scripts/run_pipeline.sh
- scripts/run_pipeline_matlab.sh
- scripts/run_validation_matlab.sh
- scripts/run_mission_matlab.sh
- scripts/run_sim_matlab.sh

## 6. Testing

Run foreground smoke validation:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
bash scripts/run_full_pipeline.sh
```

For ROS2 package build checks, see the ROS2 scaffold section below.

## 7. ROS2 C++ Scaffold

A lightweight ROS2 C++ node scaffold is added under IICC26_ws/src/sjtu_drone-ros2/autolanding_orchestrator_cpp.

Build example:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/IICC26_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select autolanding_orchestrator_cpp
```
