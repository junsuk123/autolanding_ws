# autolanding_ws

Python-first semantic autonomous landing workspace.

This repository now uses Python as the primary orchestration layer for:
- pipeline execution
- simulation stack launch
- collection/parallel collection orchestration
- paper figure generation

MATLAB entry files are retained only as compatibility wrappers that forward to the Python launcher.

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

## 2. Python Orchestrator

Main entrypoint:
- scripts/autolanding_launcher.py

Supported modes:
- pipeline
- validation
- plot
- mission
- sim
- stack
- collect
- collect_parallel

Examples:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws

python3 scripts/autolanding_launcher.py pipeline \
  --workspace-root "$PWD" \
  --semantic-input data/samples/semantic_input_example.json \
  --config ai/configs/policy_config.yaml

python3 scripts/autolanding_launcher.py sim --workspace-root "$PWD" --gui
python3 scripts/autolanding_launcher.py collect_parallel --workspace-root "$PWD" --workers 3 --scenarios-per-worker 2
```

### 2.0 하나만 실행해서 전체 파이프라인 돌리기

아래 명령 1개로 전체 파이프라인(full 모드)을 실행합니다.

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
bash scripts/run_full_pipeline.sh
```

이 명령은 내부적으로 다음을 수행합니다.
- Gazebo/SITL/RViz 런처 실행
- 파이프라인 실행
- 검증 JSON 생성
- 논문용 플롯 생성

변경 가능한 실행 파라미터 파일:
- ai/configs/orchestration_config.yaml

파일에 주석으로 각 파라미터 의미를 설명해두었습니다.

### 2.1 빠른 실행 순서 (권장)

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws

# 1) 가상환경 생성
python3 -m venv .venv
source .venv/bin/activate

# 2) 의존성 설치
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

# 3) 파이프라인 실행
python3 scripts/autolanding_launcher.py pipeline \
  --workspace-root "$PWD" \
  --semantic-input data/samples/semantic_input_example.json \
  --config ai/configs/policy_config.yaml
```

### 2.2 "Configuring a Python Environment"에서 멈춘 것처럼 보일 때

원인 요약:
- VS Code의 Python 환경 구성 단계는 내부적으로 사용자 선택(인터프리터/환경)을 기다리기 때문에, 화면상으로는 멈춘 것처럼 보일 수 있습니다.
- 실제 실행 멈춤과 별개로, 런처 import 경로 문제(ModuleNotFoundError)가 있으면 즉시 실패하며 진행이 멈춘 것처럼 느껴질 수 있습니다.

우회 방법:
- VS Code 자동 구성 없이 위 "빠른 실행 순서"를 그대로 사용하면 즉시 실행 가능합니다.
- 인터프리터는 .venv/bin/python을 직접 선택해 사용하면 안정적입니다.

진단 명령:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
which python3
python3 --version
python3 -c "import yaml, matplotlib; print('deps_ok')"
```

플롯 단계가 환경 문제로 실패하면 임시로 플롯을 끄고 실행할 수 있습니다.

```bash
python3 scripts/autolanding_launcher.py pipeline \
  --workspace-root "$PWD" \
  --semantic-input data/samples/semantic_input_example.json \
  --config ai/configs/policy_config.yaml \
  --no-plots
```

## 3. MATLAB Compatibility

The following MATLAB entrypoints now only forward to Python:
- matlab/AutoLandingMain.m
- matlab/AutoLandingMainFull.m
- matlab/AutoLandingDataParallel.m

If scripts/autolanding_launcher.py is unavailable, these wrappers fail fast with a clear error.

## 4. Paper Plot Outputs

Python plot module:
- src/paper_plots.py

Generated outputs:
- data/plots/paper/paper_trajectory_summary.png
- data/plots/paper/paper_trajectory_summary.pdf
- data/plots/paper/paper_model_comparison.png
- data/plots/paper/paper_model_comparison.pdf
- data/plots/paper/paper_model_comparison_metrics.csv
- data/plots/paper/paper_model_comparison_metrics.json

Validation output:
- data/processed/landing_trajectory_model_validation.json

Collection orchestration summary output:
- data/processed/collection_orchestration_summary.json

## 5. Scripts

Helper scripts call the Python launcher:
- scripts/run_pipeline.sh
- scripts/run_pipeline_matlab.sh
- scripts/run_validation_matlab.sh
- scripts/run_mission_matlab.sh
- scripts/run_sim_matlab.sh

## 6. Testing

Run smoke tests:

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws
python3 -m unittest tests/test_pipeline_smoke.py
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
