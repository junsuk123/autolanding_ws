#!/usr/bin/env python3
"""Build a single-Gazebo multi-drone + multi-pad world SDF."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def _grid_xy(index: int, spacing: float) -> tuple[float, float]:
    cols = max(1, math.ceil(math.sqrt(index + 1)))
    row = index // cols
    col = index % cols
    x = (col - (cols - 1) / 2.0) * spacing
    y = (row - (cols - 1) / 2.0) * spacing
    return x, y


def build_world(drone_count: int, pad_count: int, spacing_m: float, pad_offset_x_m: float) -> str:
    drone_blocks: list[str] = []
    pad_blocks: list[str] = []

    for i in range(drone_count):
        idx = i + 1
        x, y = _grid_xy(i, spacing_m)
        drone_blocks.append(
            f"""
    <include>
      <name>iris_with_gimbal_w{idx}</name>
      <uri>model://iris_with_gimbal_w{idx}</uri>
      <pose>{x:.3f} {y:.3f} 0.195 0 0 0</pose>
    </include>"""
        )

    for i in range(pad_count):
        idx = i + 1
        x, y = _grid_xy(i, spacing_m)
        pad_blocks.append(
            f"""
    <include>
      <name>aruco_landing_box_{idx}</name>
      <uri>model://aruco_box</uri>
      <pose>{x + pad_offset_x_m:.3f} {y:.3f} 0.562 0 0 0</pose>
    </include>"""
        )

    world = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.9\">
  <world name=\"iris_runway\">
    <physics name=\"1ms\" type=\"ignore\">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\" />
    <plugin filename=\"gz-sim-sensors-system\" name=\"gz::sim::systems::Sensors\">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\" />
    <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\" />
    <plugin filename=\"gz-sim-imu-system\" name=\"gz::sim::systems::Imu\" />
    <plugin filename=\"gz-sim-navsat-system\" name=\"gz::sim::systems::NavSat\" />

    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
      <sky></sky>
    </scene>

    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type=\"directional\" name=\"sun\">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>150 150</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>150 150</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

{''.join(drone_blocks)}
{''.join(pad_blocks)}
  </world>
</sdf>
"""
    return world


def main() -> int:
    parser = argparse.ArgumentParser(description='Build single-Gazebo multi-spawn world')
    parser.add_argument('--drone-count', type=int, default=3)
    parser.add_argument('--pad-count', type=int, default=3)
    parser.add_argument('--spacing-m', type=float, default=6.0)
    parser.add_argument('--pad-offset-x-m', type=float, default=1.5)
    parser.add_argument('--out', type=str, default='/tmp/iris_runway_multi_spawn.sdf')
    args = parser.parse_args()

    drone_count = max(1, int(args.drone_count))
    pad_count = max(1, int(args.pad_count))
    spacing_m = max(1.0, float(args.spacing_m))

    payload = build_world(drone_count, pad_count, spacing_m, float(args.pad_offset_x_m))
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(payload, encoding='utf-8')
    print(f'[multi-spawn] world={out_path}')
    print(f'[multi-spawn] drones={drone_count} pads={pad_count} spacing_m={spacing_m}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
