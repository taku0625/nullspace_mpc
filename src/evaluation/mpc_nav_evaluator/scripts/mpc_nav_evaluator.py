#!/usr/bin/env python3

import csv
from math import cos, hypot, sin
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from mpc_eval_msgs.msg import MPCEval
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import yaml


MPC_FIELDS = (
    "state_cost", "global_x", "global_y", "global_yaw",
    "cmd_vx", "cmd_vy", "cmd_yawrate",
    "cmd_steer_fl", "cmd_steer_fr", "cmd_steer_rl", "cmd_steer_rr",
    "cmd_rotor_fl", "cmd_rotor_fr", "cmd_rotor_rl", "cmd_rotor_rr", "calc_time_ms",
)


class MPCEvaluator(Node):
    def __init__(self):
        super().__init__("mpc_nav_evaluator")
        scenario = Path(self.declare_parameter("scenario_config_path", "").value).expanduser()
        result_dir = Path(self.declare_parameter("eval_result_dir", "result").value).expanduser()
        goal_topic = self.declare_parameter("goal_pose_topic", "/goal_pose").value
        eval_topic = self.declare_parameter("mpc_eval_topic", "/mpc/eval_info").value
        self._goal_tolerance = self.declare_parameter("goal_tolerance", 0.5).value
        with scenario.open() as config:
            self._goals = yaml.safe_load(config)["goals"]
        if not self._goals:
            raise ValueError(f"{scenario} has no goals.")
        result_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = result_dir / f"{scenario.stem}.csv"
        with self._csv_path.open("w", newline="") as output:
            csv.writer(output).writerow(("time_stamp", *MPC_FIELDS, "goal_reached"))

        self._reached = 0
        self._published_at = None
        self._goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self._sub = self.create_subscription(MPCEval, eval_topic, self._record, 10)
        self._timer = self.create_timer(0.5, self._start)

    def _start(self):
        if self._goal_pub.get_subscription_count() > 0:
            self._timer.cancel()
            self._publish_goal()

    def _publish_goal(self):
        goal = self._goals[self._reached]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        self._published_at = self.get_clock().now()
        pose.header.stamp = self._published_at.to_msg()
        pose.pose.position.x = float(goal["goal_x"])
        pose.pose.position.y = float(goal["goal_y"])
        yaw = float(goal.get("goal_yaw", 0.0))
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        self._goal_pub.publish(pose)
        self.get_logger().info(f"Published goal {self._reached + 1} of {len(self._goals)}.")

    def _record(self, msg):
        if self._published_at is None:
            return
        sample_time = Time.from_msg(msg.header.stamp)
        goal = self._goals[self._reached]
        distance = hypot(msg.global_x - goal["goal_x"], msg.global_y - goal["goal_y"])
        reached = (
            msg.goal_reached and distance <= self._goal_tolerance
            and (sample_time - self._published_at).nanoseconds > 100_000_000
        )
        if reached:
            self._reached += 1

        with self._csv_path.open("a", newline="") as output:
            csv.writer(output).writerow((
                sample_time.nanoseconds / 1e9, *(getattr(msg, field) for field in MPC_FIELDS), self._reached,
            ))

        if self._reached == len(self._goals):
            self.get_logger().info(f"All {self._reached} goals reached.")
            self.context.try_shutdown()
        elif reached:
            self._publish_goal()


def main():
    rclpy.init()
    node = MPCEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
