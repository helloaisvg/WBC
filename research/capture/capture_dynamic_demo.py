#!/usr/bin/env python3

import argparse
import subprocess
import time
from pathlib import Path


CONTAINER = "remani-noetic"
WORKSPACE = "/ws"
RECORDER = "/repo/tools/record_dynamic_demo.py"
RENDERER = Path(__file__).with_name("render_whole_body_video.py")


def stop_scene():
    subprocess.run(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            "pkill -TERM -f 'record_dynamic_demo.py'; "
            "pkill -TERM -f 'roslaunch remani_planner dynamic_exp'; true",
        ],
        timeout=10,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        choices=("exp0", "exp1", "exp1_random"),
        required=True,
    )
    parser.add_argument("--seed", type=int, default=32)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--raw-output", type=Path)
    parser.add_argument("--mission-timeout", type=float, default=50.0)
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--render-fps", type=int, default=30)
    parser.add_argument("--render-step", type=float, default=1.0 / 30.0)
    args = parser.parse_args()

    stop_scene()
    remote_log = "/tmp/remani_dynamic_demo.log"
    remote_trace = "/tmp/remani_dynamic_demo.jsonl"
    subprocess.run(
        ["docker", "exec", CONTAINER, "rm", "-f", remote_log, remote_trace],
        timeout=5,
        check=True,
    )

    launch = (
        f"source /opt/ros/noetic/setup.bash; "
        f"source {WORKSPACE}/devel/setup.bash; "
        f"exec timeout {args.mission_timeout:.1f}s roslaunch remani_planner "
        f"dynamic_{args.scenario}.launch map_seed:={args.seed} "
        f"> {remote_log} 2>&1"
    )
    launcher = subprocess.Popen(
        ["docker", "exec", CONTAINER, "bash", "-lc", launch],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    time.sleep(2)
    recorder = subprocess.Popen(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            f"source /opt/ros/noetic/setup.bash; "
            f"source {WORKSPACE}/devel/setup.bash; "
            f"exec timeout {max(1.0, args.mission_timeout - 2.0):.1f}s "
            f"python3 {RECORDER} --output {remote_trace}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )

    time.sleep(args.startup_wait)
    subprocess.run(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            f"source /opt/ros/noetic/setup.bash; "
            f"source {WORKSPACE}/devel/setup.bash; "
            "timeout 8s rostopic pub -1 /move_base_simple/goal "
            "geometry_msgs/PoseStamped "
            "\"{header: {frame_id: world}, pose: {position: "
            "{x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"",
        ],
        timeout=12,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )

    completion_timeout = max(
        1.0, args.mission_timeout - args.startup_wait - 2.0
    )
    subprocess.run(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            f"timeout {completion_timeout:.1f}s tail -F -n +1 "
            f"{remote_log} "
            "| grep -m1 'open gripper'",
        ],
        timeout=completion_timeout + 8.0,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    stop_scene()
    for process in (launcher, recorder):
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

    args.raw_output = args.raw_output or args.output.with_suffix(".jsonl")
    args.raw_output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "docker", "cp",
            f"{CONTAINER}:{remote_trace}",
            str(args.raw_output),
        ],
        timeout=30,
        check=True,
    )
    subprocess.run(
        [
            "python3",
            str(RENDERER),
            str(args.raw_output),
            str(args.output),
            "--title",
            f"REMANI-R | {args.scenario} seed {args.seed} | whole-body view",
            "--fps",
            str(args.render_fps),
            "--step",
            str(args.render_step),
        ],
        timeout=600,
        check=True,
    )


if __name__ == "__main__":
    main()
