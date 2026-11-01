#!/usr/bin/env python3

import argparse
import subprocess
import time
from pathlib import Path


CONTAINER = "remani-noetic"
WORKSPACE = "/ws"
DISPLAY = ":102"
RVIZ_CONFIG = "/repo/remani_planner/plan_manage/launch/exp0.rviz"


def run(args, timeout, check=True):
    return subprocess.run(
        args,
        timeout=timeout,
        check=check,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def docker_exec(command, timeout, check=True):
    return run(
        ["docker", "exec", CONTAINER, "bash", "-lc", command],
        timeout,
        check,
    )


def stop_processes():
    docker_exec(
        "pkill -TERM -x rviz; "
        "pkill -TERM -f '^/usr/bin/python3 "
        "/opt/ros/noetic/bin/roslaunch remani_planner'; "
        "pkill -TERM -x ffmpeg; "
        "pkill -TERM -f '^Xvfb :102'; true",
        12,
        check=False,
    )
    time.sleep(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="exp0")
    parser.add_argument("--seed", type=int, default=30)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mission-timeout", type=float, default=100.0)
    parser.add_argument("--map-wait", type=float, default=12.0)
    parser.add_argument("--rviz-wait", type=float, default=5.0)
    args = parser.parse_args()

    stop_processes()
    remote_log = f"/tmp/remani_rviz_{args.seed}.log"
    remote_video = f"/tmp/remani_rviz_{args.seed}.mp4"
    docker_exec(
        f"rm -f {remote_log} {remote_video} /tmp/rviz_ffmpeg_{args.seed}.log",
        10,
    )
    run(
        [
            "docker", "exec", "-d", CONTAINER, "bash", "-lc",
            f"exec Xvfb {DISPLAY} -screen 0 640x360x24 -ac "
            "> /tmp/xvfb_rviz_video.log 2>&1",
        ],
        15,
    )
    time.sleep(2)

    launch = (
        f"source /opt/ros/noetic/setup.bash; "
        f"source {WORKSPACE}/devel/setup.bash; "
        f"exec roslaunch remani_planner dynamic_{args.scenario}.launch "
        f"map_seed:={args.seed} > {remote_log} 2>&1"
    )
    run(
        [
            "docker", "exec", "-d",
            "-e", "DISABLE_ROS1_EOL_WARNINGS=1",
            CONTAINER, "bash", "-lc", launch,
        ],
        15,
    )
    time.sleep(args.map_wait)

    rviz = (
        f"source /opt/ros/noetic/setup.bash; "
        f"source {WORKSPACE}/devel/setup.bash; "
        f"exec rviz -geometry 640x360 --opengl 210 "
        f"--disable-anti-aliasing -d {RVIZ_CONFIG} "
        "> /tmp/remani_rviz_video_ui.log 2>&1"
    )
    run(
        [
            "docker", "exec", "-d",
            "-e", f"DISPLAY={DISPLAY}",
            "-e", "LIBGL_ALWAYS_SOFTWARE=1",
            "-e", "LP_NUM_THREADS=4",
            "-e", "mesa_glthread=true",
            "-e", "vblank_mode=0",
            "-e", "DISABLE_ROS1_EOL_WARNINGS=1",
            CONTAINER, "bash", "-lc", rviz,
        ],
        15,
    )
    time.sleep(args.rviz_wait)

    ffmpeg = (
        f"exec ffmpeg -y -f x11grab -framerate 15 "
        f"-video_size 640x360 -i {DISPLAY} "
        f"-t {args.mission_timeout:.1f} "
        "-c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p "
        f"{remote_video} > /tmp/rviz_ffmpeg_{args.seed}.log 2>&1"
    )
    run(
        [
            "docker", "exec", "-d",
            "-e", f"DISPLAY={DISPLAY}",
            "-e", "LIBGL_ALWAYS_SOFTWARE=1",
            "-e", "LP_NUM_THREADS=4",
            "-e", "mesa_glthread=true",
            "-e", "vblank_mode=0",
            CONTAINER, "bash", "-lc", ffmpeg,
        ],
        15,
    )
    time.sleep(1)

    remaining = max(
        5.0,
        args.mission_timeout - args.map_wait - args.rviz_wait - 3.0,
    )
    completion = subprocess.Popen(
        [
            "docker", "exec", CONTAINER, "bash", "-lc",
            f"source /opt/ros/noetic/setup.bash; "
            f"source {WORKSPACE}/devel/setup.bash; "
            f"exec timeout {remaining + 5.0:.1f}s "
            "python3 /repo/tools/wait_gripper_cycle.py",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    docker_exec(
        "source /opt/ros/noetic/setup.bash; "
        f"source {WORKSPACE}/devel/setup.bash; "
        "timeout 8s rostopic pub -1 /move_base_simple/goal "
        "geometry_msgs/PoseStamped "
        "\"{header: {frame_id: world}, pose: {position: "
        "{x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"",
        12,
        check=False,
    )

    try:
        completion_returncode = completion.wait(timeout=remaining + 5.0)
        completed = completion_returncode == 0
    except subprocess.TimeoutExpired:
        completion.terminate()
        completed = False
    time.sleep(1)
    docker_exec("pkill -INT -x ffmpeg; true", 10, check=False)
    time.sleep(2)
    stop_processes()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    run(
        [
            "docker", "cp",
            f"{CONTAINER}:{remote_video}",
            str(args.output),
        ],
        60,
    )
    run(
        [
            "docker", "cp",
            f"{CONTAINER}:{remote_log}",
            str(args.output.with_suffix(".log")),
        ],
        30,
    )
    print(f"output={args.output}")
    print(f"mission_complete={int(completed)}")


if __name__ == "__main__":
    main()
