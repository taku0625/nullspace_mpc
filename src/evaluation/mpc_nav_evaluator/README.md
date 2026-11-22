# MPC Navigation Evaluator

The Jazzy evaluator publishes goals on `/goal_pose` and records telemetry from
`/mpc/eval_info` in a CSV. Use the bringup launcher to run an entire episode.

```bash
ros2 launch nullspace_mpc_bringup navigation.launch.py \
  scenario_config_path:="$PWD/data/eval_demo/episode_0000.yaml" eval_result_dir:="$PWD/result"
```

Use the evaluation runner for an agenda with multiple episodes.

```bash
ros2 run nullspace_mpc_bringup run_evaluation.py \
  --agenda_yaml_path data/eval_demo/agenda.yaml --controller nullspace_mpc
```
