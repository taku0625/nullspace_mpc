#!/usr/bin/env python3

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

import numpy as np
import pandas as pd
import yaml


CONTROLLERS = ("nullspace_mpc", "mppi_3d_a", "mppi_3d_b", "mppi_4d", "mppi_h", "mppi_unicycle")
WORLDS = ("maze", "empty_garden", "cylinder_garden")


def run_single_episode(config_path, result_dir, command, timeout):
    args = [
        *command, f"scenario_config_path:={config_path}", f"eval_result_dir:={result_dir}",
    ]
    print(f"Starting {config_path.name}", flush=True)
    with subprocess.Popen(args, start_new_session=True) as episode:
        try:
            code = episode.wait(timeout=timeout)
            if code:
                print(f"{config_path.name} exited with status {code}.", flush=True)
        except subprocess.TimeoutExpired:
            print(f"{config_path.name} reached its time limit.", flush=True)
        finally:
            if episode.poll() is None:
                os.killpg(episode.pid, signal.SIGINT)
                try:
                    episode.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(episode.pid, signal.SIGKILL)
                    episode.wait()


def run_evaluation(agenda_yaml_path, controller, world_name, headless):
    agenda_path = Path(agenda_yaml_path).expanduser().resolve()
    with agenda_path.open() as source:
        agenda = yaml.safe_load(source)
    scenarios = []
    for name in agenda["scenarios"]:
        scenario = agenda_path.parent / Path(name).expanduser()
        if not scenario.is_file() and name.startswith("~/nullspace_mpc/"):
            # The bundled agendas store episode paths under the previous checkout location.
            scenario = agenda_path.parent / Path(name).name
        scenarios.append(scenario)
    total_episodes = agenda["total_number_of_episodes"]
    if not 0 < total_episodes <= len(scenarios):
        raise ValueError("The episode count must be between 1 and the number of scenarios.")
    for scenario in scenarios[:total_episodes]:
        if not scenario.is_file():
            raise FileNotFoundError(scenario)

    result_root = Path(agenda["result_dir"]).expanduser()
    if agenda["result_dir"] == "~/nullspace_mpc/result":
        result_root = Path.cwd() / "result"
    result_dir = result_root.resolve() / datetime.now().strftime("eval_%Y%m%d_%H%M%S_%f")
    result_dir.mkdir(parents=True)
    shutil.copyfile(agenda_path, result_dir / f"agenda_{controller}.yaml")

    command = [
        "ros2", "launch", "nullspace_mpc_bringup", "navigation.launch.py",
        f"local_planner:={controller}", f"gazebo_world_name:={world_name}",
    ]
    if headless:
        command.extend(("open_rviz:=false", "show_gazebo_gui:=false", "gazebo_headless:=true"))

    # make a csv to save summary of the evaluation
    summary_csv_filename = f'summary_{controller}.csv'
    summary_df = pd.DataFrame(columns=[
        'episode',
        'mean_state_cost',
        'min_calc_time',
        'mean_calc_time',
        'max_calc_time',
        'steer_cmd_diff',
        'rotor_cmd_diff',
        'vehicle_cmd_diff',
        'trajectory_length',
        'max_goal_reached',
        'episode_time',
        'all_goals_reached',
    ])
    summary_df.to_csv(os.path.join(result_dir, summary_csv_filename), index=False)

    for episode, scenario in enumerate(scenarios[:total_episodes]):
        with scenario.open() as source:
            total_goal_count = len(yaml.safe_load(source)["goals"])

        run_single_episode(scenario, result_dir, command, agenda["maximum_time_per_single_episode"])
        episode_csv = result_dir / f"{scenario.stem}.csv"
        if not episode_csv.is_file():
            raise RuntimeError(f"No evaluation CSV was written for {scenario.name}.")
        df = pd.read_csv(episode_csv)
        if episode + 1 < total_episodes:
            time.sleep(agenda["cooling_time_between_episodes"])

        # add new column and initialize it with 0
        df['rotor_cmd_diff'] = 0.0
        df['steer_cmd_diff'] = 0.0
        df['vehicle_cmd_diff'] = 0.0

        # add column of control input change
        for i in range(1, len(df)):
            # calculate the difference of control input
            rotor_diff = np.abs(df.at[i, 'cmd_rotor_fl'] - df.at[i-1, 'cmd_rotor_fl']) + \
                         np.abs(df.at[i, 'cmd_rotor_fr'] - df.at[i-1, 'cmd_rotor_fr']) + \
                         np.abs(df.at[i, 'cmd_rotor_rl'] - df.at[i-1, 'cmd_rotor_rl']) + \
                         np.abs(df.at[i, 'cmd_rotor_rr'] - df.at[i-1, 'cmd_rotor_rr'])

            # note: steering angle is [-pi ~ pi]
            diff_steer_fl = np.fmod(np.abs(df.at[i, 'cmd_steer_fl'] - df.at[i-1, 'cmd_steer_fl']), (2.0 * np.pi))
            diff_steer_fl = np.min([diff_steer_fl, 2.0*np.pi - diff_steer_fl])
            diff_steer_fr = np.fmod(np.abs(df.at[i, 'cmd_steer_fr'] - df.at[i-1, 'cmd_steer_fr']), (2.0 * np.pi))
            diff_steer_fr = np.min([diff_steer_fr, 2.0*np.pi - diff_steer_fr])
            diff_steer_rl = np.fmod(np.abs(df.at[i, 'cmd_steer_rl'] - df.at[i-1, 'cmd_steer_rl']), (2.0 * np.pi))
            diff_steer_rl = np.min([diff_steer_rl, 2.0*np.pi - diff_steer_rl])
            diff_steer_rr = np.fmod(np.abs(df.at[i, 'cmd_steer_rr'] - df.at[i-1, 'cmd_steer_rr']), (2.0 * np.pi))
            diff_steer_rr = np.min([diff_steer_rr, 2.0*np.pi - diff_steer_rr])
            steer_diff = diff_steer_fl + diff_steer_fr + diff_steer_rl + diff_steer_rr

            df.at[i, 'rotor_cmd_diff'] = rotor_diff
            df.at[i, 'steer_cmd_diff'] = steer_diff
            df.at[i, 'vehicle_cmd_diff'] =  rotor_diff + steer_diff

        # get summary of the episode
        mean_state_cost = df["state_cost"].mean()
        min_calc_time = df["calc_time_ms"].min()
        mean_calc_time = df["calc_time_ms"].mean()
        max_calc_time = df["calc_time_ms"].max()
        max_goal_reached = df["goal_reached"].max()
        episode_time = df["time_stamp"].max() - df["time_stamp"].min()
        trajectory_length = np.sum(np.sqrt(np.diff(df["global_x"])**2 + np.diff(df["global_y"])**2))
        all_goals_reached = 1 if max_goal_reached == total_goal_count else 0
        sum_steer_cmd_diff = df["steer_cmd_diff"].sum()
        sum_rotor_cmd_diff = df["rotor_cmd_diff"].sum()
        sum_vehicle_cmd_diff = df["vehicle_cmd_diff"].sum()

        # update the summary csv
        summary_df.loc[len(summary_df)] = {
            'episode': episode,
            'mean_state_cost': mean_state_cost,
            'min_calc_time': min_calc_time,
            'mean_calc_time': mean_calc_time,
            'max_calc_time': max_calc_time,
            'steer_cmd_diff': sum_steer_cmd_diff,
            'rotor_cmd_diff': sum_rotor_cmd_diff,
            'vehicle_cmd_diff': sum_vehicle_cmd_diff,
            'trajectory_length': trajectory_length,
            'max_goal_reached': max_goal_reached,
            'episode_time': episode_time,
            'all_goals_reached': all_goals_reached,
        }
        summary_df.to_csv(os.path.join(result_dir, summary_csv_filename), index=False)

    # Here, all episodes are finished. Now, calculate the success rate and the summary of the whole evaluation

    # calculate the success rate
    success_rate = summary_df['all_goals_reached'].sum() / len(summary_df)

    # remove failed episode from the summary_df
    successful_episodes_df = summary_df[summary_df['all_goals_reached'] == 1]

    # get summary of the evaluation
    mean_state_cost        = successful_episodes_df["mean_state_cost"].mean()
    min_calc_time          = successful_episodes_df["min_calc_time"].min()  # get minimum
    mean_calc_time         = successful_episodes_df["mean_calc_time"].mean()
    max_calc_time          = successful_episodes_df["max_calc_time"].max()  # get maximum
    mean_steer_cmd_diff    = successful_episodes_df["steer_cmd_diff"].mean()
    mean_rotor_cmd_diff    = successful_episodes_df["rotor_cmd_diff"].mean()
    mean_vehicle_cmd_diff  = successful_episodes_df["vehicle_cmd_diff"].mean()
    mean_trajectory_length = successful_episodes_df["trajectory_length"].mean()
    mean_goal_reached      = successful_episodes_df["max_goal_reached"].mean()
    mean_episode_time      = successful_episodes_df["episode_time"].mean()

    # append row to the summary csv, that shows total summary of the evaluation
    summary_df.loc[len(summary_df)] = {
        'episode': 'summary',
        'mean_state_cost': mean_state_cost,
        'min_calc_time': min_calc_time,
        'mean_calc_time': mean_calc_time,
        'max_calc_time': max_calc_time,
        'steer_cmd_diff': mean_steer_cmd_diff,
        'rotor_cmd_diff': mean_rotor_cmd_diff,
        'vehicle_cmd_diff': mean_vehicle_cmd_diff,
        'trajectory_length': mean_trajectory_length,
        'max_goal_reached': mean_goal_reached,
        'episode_time': mean_episode_time,
        'all_goals_reached': success_rate,
    }
    summary_df.to_csv(os.path.join(result_dir, summary_csv_filename), index=False)

    # announce the evaluation is finished
    print("\n\n")
    print("#"*30)
    print(f'[INFO] All {total_episodes} episodes finished. The result is saved in {result_dir}')
    print("#"*30)
    print("\n\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--agenda_yaml_path", required=True, help="Evaluation agenda YAML file")
    parser.add_argument("--controller", choices=CONTROLLERS, default="nullspace_mpc")
    parser.add_argument("--world_name", choices=WORLDS, default="maze")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    run_evaluation(args.agenda_yaml_path, args.controller, args.world_name, args.headless)
