#!/usr/bin/env bash
set -eo pipefail

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
episode_count=${1:-hundred}
agenda="$workspace/data/eval_$episode_count/large/agenda_large.yaml"
cd "$workspace"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

for world in cylinder_garden maze; do
    for controller in nullspace_mpc mppi_h; do
        ros2 run nullspace_mpc_bringup run_evaluation.py --agenda_yaml_path "$agenda" \
            --controller "$controller" --world_name "$world"
    done
done
