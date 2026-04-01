# Architecture

## Pipeline

1. Ontology context ingestion
2. Object/attribute/relation parsing
3. Semantic feature extraction
4. Fusion with AI confidence
5. Trajectory generation
6. Simulation/control execution

## Main interfaces

- Input: ontology/instances/*.json
- Feature output: semantic_features dict
- Policy output: sequence of commanded positions and velocities
- Artifacts: data/processed/landing_trajectory.json and .csv

## Extension points

- Replace semantic encoder in src/semantic_feature_extractor.py
- Replace placeholder AI confidence in src/trajectory_policy.py
- Add ROS 2 bridge in simulation layer for closed-loop control
