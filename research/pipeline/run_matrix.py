#!/usr/bin/env python3

import argparse
import csv
import re
import shlex
import subprocess
import time
import tomllib
from pathlib import Path


CONTAINER = "remani-noetic"
WORKSPACE = "/ws"
PLANNER_CONFIG = "/repo/remani_planner/plan_manage/config/remani_planner_param.yaml"
PIPELINE_DIR = Path(__file__).resolve().parent
CONFIG_DIR = PIPELINE_DIR.parent / "configs"
METRIC_RE = re.compile(
    r"Planner wall time: ([0-9.]+) ms, init: ([0-9.]+) ms, "
    r"optimize: ([0-9.]+) ms, trajectory duration: ([0-9.]+) s, "
    r"recovery margin: (-?[0-9.]+) m, "
    r"recovery slack: (-?[0-9.]+) s"
)
DYNAMIC_RE = re.compile(r"dynamic obstacles: (\d+)")
GENERATED_RE = re.compile(
    r"Generated (\d+) ground and (\d+) floating random obstacles"
)

def load_toml_directory(directory):
    return {
        path.stem: tomllib.loads(path.read_text())
        for path in sorted(directory.glob("*.toml"))
    }


MODE_CONFIGS = load_toml_directory(CONFIG_DIR / "modes")
SCENARIO_CONFIGS = load_toml_directory(CONFIG_DIR / "scenarios")
MODES = {
    name: config["overrides"]
    for name, config in MODE_CONFIGS.items()
}
SCENARIOS = {
    name: config["container_param_file"]
    for name, config in SCENARIO_CONFIGS.items()
}


def run(cmd, timeout, check=True):
    result = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    if check and result.returncode != 0:
        raise RuntimeError(result.stdout.strip())
    return result.stdout


def docker(args, timeout, check=True):
    return run(["docker", *args], timeout, check)


def ensure_container():
    running = docker(
        ["inspect", "-f", "{{.State.Running}}", CONTAINER],
        timeout=10,
        check=False,
    ).strip()
    if running != "true":
        docker(["start", CONTAINER], timeout=30)


def prepare_mode(mode):
    mode_values = MODES[mode]
    expressions = []
    for key, value in mode_values.items():
        expression = (
            f"s/^([[:space:]]*{key}[[:space:]]*:).*/\\1 {value}/"
        )
        expressions.extend(["-e", expression])
    target = f"/tmp/remani_ablation_{mode}.yaml"
    command = [
        "sed",
        "-E",
        *expressions,
        PLANNER_CONFIG,
    ]
    quoted = " ".join(shlex.quote(part) for part in command)
    docker(
        [
            "exec",
            CONTAINER,
            "bash",
            "-lc",
            f"{quoted} > {target}",
        ],
        timeout=10,
    )
    return target


def stop_previous():
    docker(
        [
            "exec",
            CONTAINER,
            "pkill",
            "-TERM",
            "-f",
            "roslaunch remani_planner run_in_sim.launch",
        ],
        timeout=10,
        check=False,
    )
    time.sleep(1)


def run_trial(
    mode,
    scenario,
    seed,
    planner_config,
    obs_num,
    float_obs_num,
    dynamic_obs_num,
    dynamic_obs_speed,
    dynamic_obs_radius,
    dynamic_obs_cross_x,
    dynamic_obs_cross_y,
    dynamic_obs_z,
    dynamic_obs_cross_offset,
    dynamic_obs_spawn_delay,
    startup_wait,
    run_seconds,
    log_directory,
    stop_after_second_plan,
):
    log_path = f"/tmp/remani_{mode}_{scenario}_{seed}.log"
    stop_previous()
    docker(
        [
            "exec",
            CONTAINER,
            "rm",
            "-f",
            log_path,
        ],
        timeout=5,
    )
    launch = (
        f"source /opt/ros/noetic/setup.bash; "
        f"source {WORKSPACE}/devel/setup.bash; "
        f"exec timeout {run_seconds}s roslaunch remani_planner run_in_sim.launch "
        f"exp_param_file:={SCENARIOS[scenario]} "
        f"planner_param_file:={planner_config} "
        f"map_seed:={seed} "
        f"obs_num:={obs_num} "
        f"float_obs_num:={float_obs_num} "
        f"dynamic_obs_num:={dynamic_obs_num} "
        f"dynamic_obs_speed:={dynamic_obs_speed} "
        f"dynamic_obs_radius:={dynamic_obs_radius} "
        f"dynamic_obs_cross_x:={dynamic_obs_cross_x} "
        f"dynamic_obs_cross_y:={dynamic_obs_cross_y} "
        f"dynamic_obs_z:={dynamic_obs_z} "
        f"dynamic_obs_cross_offset:={dynamic_obs_cross_offset} "
        f"dynamic_obs_spawn_delay:={dynamic_obs_spawn_delay} "
        f"> {log_path} 2>&1"
    )
    launcher = subprocess.Popen(
        [
            "docker",
            "exec",
            "-e",
            "DISABLE_ROS1_EOL_WARNINGS=1",
            CONTAINER,
            "bash",
            "-lc",
            launch,
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    time.sleep(startup_wait)
    docker(
        [
            "exec",
            CONTAINER,
            "bash",
            "-lc",
            "source /opt/ros/noetic/setup.bash; "
            f"source {WORKSPACE}/devel/setup.bash; "
            "timeout 8s rostopic pub -1 /move_base_simple/goal "
            "geometry_msgs/PoseStamped "
            "\"{header: {frame_id: world}, pose: {position: "
            "{x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"",
        ],
        timeout=12,
    )
    remaining = max(1.0, run_seconds - startup_wait)
    if stop_after_second_plan:
        completion_expr = (
            "awk '/Planner wall time/ {count++} "
            "count >= 2 {print; exit}'"
        )
    else:
        completion_expr = "grep -m1 'open gripper'"
    completion = docker(
        [
            "exec",
            CONTAINER,
            "bash",
            "-lc",
            f"timeout {remaining:.1f}s tail -F -n +1 {log_path} 2>/dev/null "
            f"| {completion_expr}",
        ],
        timeout=int(remaining) + 8,
        check=False,
    )
    completed_early = bool(completion.strip())

    stop_previous()
    try:
        launcher.communicate(timeout=8)
    except subprocess.TimeoutExpired:
        launcher.kill()
        launcher.communicate(timeout=5)
    launch_exit_code = launcher.returncode
    log = docker(
        ["exec", CONTAINER, "cat", log_path],
        timeout=5,
        check=False,
    )
    log_directory.mkdir(parents=True, exist_ok=True)
    (log_directory / f"{mode}_{scenario}_{seed}.log").write_text(log)
    metrics = METRIC_RE.findall(log)
    dynamic_seen = [int(value) for value in DYNAMIC_RE.findall(log)]
    generated = GENERATED_RE.findall(log)
    generated_ground, generated_floating = (
        tuple(map(int, generated[-1])) if generated else (0, 0)
    )
    row = {
        "mode": mode,
        "scenario": scenario,
        "seed": seed,
        "obs_num": obs_num,
        "float_obs_num": float_obs_num,
        "generated_ground": generated_ground,
        "generated_floating": generated_floating,
        "dynamic_obs_num": dynamic_obs_num,
        "dynamic_obs_speed": dynamic_obs_speed,
        "dynamic_obs_radius": dynamic_obs_radius,
        "dynamic_obs_cross_x": dynamic_obs_cross_x,
        "dynamic_obs_cross_y": dynamic_obs_cross_y,
        "dynamic_obs_z": dynamic_obs_z,
        "dynamic_obs_cross_offset": dynamic_obs_cross_offset,
        "dynamic_obs_spawn_delay": dynamic_obs_spawn_delay,
        "dynamic_obstacles_seen": max(dynamic_seen, default=0),
        "launch_exit_code": launch_exit_code,
        "completed_early": int(completed_early),
        "stop_after": (
            "second_plan" if stop_after_second_plan else "mission"
        ),
        "map_loaded": int("Loaded global cloud" in log),
        "first_goal_completed": log.count("close gripper"),
        "second_goal_completed": log.count("open gripper"),
        "goal_collision_attempts": log.count("is not free!"),
        "plans": len(metrics),
        "wall_ms_1": "",
        "wall_ms_2": "",
        "init_ms_1": "",
        "init_ms_2": "",
        "opt_ms_1": "",
        "opt_ms_2": "",
        "duration_s_1": "",
        "duration_s_2": "",
        "margin_m_1": "",
        "margin_m_2": "",
        "recovery_slack_s_1": "",
        "recovery_slack_s_2": "",
        "replanning_events": log.count("Try to replan a safe trajectory"),
        "recovery_threats": log.count("Recoverability threat"),
        "temporal_repairs": log.count("Temporal safety repair"),
        "collision_warnings": log.count("collision at"),
        "emergency_stops": log.count("Emergency stop"),
    }
    for index, values in enumerate(metrics[:2], start=1):
        wall, init, opt, duration, margin, slack = values
        row[f"wall_ms_{index}"] = wall
        row[f"init_ms_{index}"] = init
        row[f"opt_ms_{index}"] = opt
        row[f"duration_s_{index}"] = duration
        row[f"margin_m_{index}"] = margin
        row[f"recovery_slack_s_{index}"] = slack
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--modes",
        nargs="+",
        default=["remani", "full", "static_shell"],
        choices=sorted(MODES),
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=["exp0", "exp1_random"],
        choices=sorted(SCENARIOS),
    )
    parser.add_argument(
        "--seeds", nargs="+", type=int, default=list(range(30, 50))
    )
    parser.add_argument("--obs-num", type=int, default=20)
    parser.add_argument("--float-obs-num", type=int, default=30)
    parser.add_argument("--dynamic-obs-num", type=int, default=0)
    parser.add_argument("--dynamic-speed", type=float, default=0.9)
    parser.add_argument("--dynamic-radius", type=float, default=0.45)
    parser.add_argument("--dynamic-cross-x", type=float, default=0.4)
    parser.add_argument("--dynamic-cross-y", type=float, default=0.0)
    parser.add_argument("--dynamic-z", type=float, default=0.62)
    parser.add_argument("--dynamic-cross-offset", type=float, default=10.0)
    parser.add_argument("--dynamic-spawn-delay", type=float, default=0.0)
    parser.add_argument("--startup-wait", type=float, default=5.0)
    parser.add_argument("--run-seconds", type=int, default=40)
    parser.add_argument("--stop-after-second-plan", action="store_true")
    parser.add_argument("--save-logs-dir", default="")
    parser.add_argument(
        "--output",
        default="research/results/canonical/recovery_large.csv",
    )
    args = parser.parse_args()

    ensure_container()
    planner_configs = {mode: prepare_mode(mode) for mode in args.modes}
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    log_directory = (
        Path(args.save_logs_dir)
        if args.save_logs_dir
        else output_path.parent / "logs"
    )

    fields = [
        "mode",
        "scenario",
        "seed",
        "obs_num",
        "float_obs_num",
        "generated_ground",
        "generated_floating",
        "dynamic_obs_num",
        "dynamic_obs_speed",
        "dynamic_obs_radius",
        "dynamic_obs_cross_x",
        "dynamic_obs_cross_y",
        "dynamic_obs_z",
        "dynamic_obs_cross_offset",
        "dynamic_obs_spawn_delay",
        "dynamic_obstacles_seen",
        "launch_exit_code",
        "completed_early",
        "stop_after",
        "map_loaded",
        "first_goal_completed",
        "second_goal_completed",
        "goal_collision_attempts",
        "plans",
        "wall_ms_1",
        "wall_ms_2",
        "init_ms_1",
        "init_ms_2",
        "opt_ms_1",
        "opt_ms_2",
        "duration_s_1",
        "duration_s_2",
        "margin_m_1",
        "margin_m_2",
        "recovery_slack_s_1",
        "recovery_slack_s_2",
        "replanning_events",
        "recovery_threats",
        "temporal_repairs",
        "collision_warnings",
        "emergency_stops",
    ]

    with output_path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for mode in args.modes:
            for scenario in args.scenarios:
                for seed in args.seeds:
                    row = run_trial(
                        mode,
                        scenario,
                        seed,
                        planner_configs[mode],
                        args.obs_num,
                        args.float_obs_num,
                        args.dynamic_obs_num,
                        args.dynamic_speed,
                        args.dynamic_radius,
                        args.dynamic_cross_x,
                        args.dynamic_cross_y,
                        args.dynamic_z,
                        args.dynamic_cross_offset,
                        args.dynamic_spawn_delay,
                        args.startup_wait,
                        args.run_seconds,
                        log_directory,
                        args.stop_after_second_plan,
                    )
                    writer.writerow(row)
                    output.flush()
                    print(
                        f"{mode:10s} {scenario} seed={seed} "
                        f"plans={row['plans']} "
                        f"margin={row['margin_m_1']}/{row['margin_m_2']} "
                        f"slack={row['recovery_slack_s_1']}/{row['recovery_slack_s_2']} "
                        f"replans={row['replanning_events']} "
                        f"repairs={row['temporal_repairs']} "
                        f"stops={row['emergency_stops']}",
                        flush=True,
                    )


if __name__ == "__main__":
    main()
